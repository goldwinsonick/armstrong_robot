#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import csv
import time
import os
from pathlib import Path

class OLTFTestNode(Node):
    def __init__(self):
        super().__init__('oltf_test_node')

        # Declare parameters with default values (though YAML will override)
        self.declare_parameter('joint_name_to_test', 'joint1')
        self.declare_parameter('effort_command_topic', '/joint1_effort_controller/commands')
        self.declare_parameter('step_effort_magnitude', 1.0)
        self.declare_parameter('initial_settle_duration', 2.0)
        self.declare_parameter('step_duration', 5.0)
        self.declare_parameter('post_step_duration', 2.0)
        self.declare_parameter('log_frequency', 50.0)
        self.declare_parameter('log_file_path', 'oltf_log.csv')

        # Get parameters
        self.joint_to_test = self.get_parameter('joint_name_to_test').get_parameter_value().string_value
        self.command_topic = self.get_parameter('effort_command_topic').get_parameter_value().string_value
        self.step_effort = self.get_parameter('step_effort_magnitude').get_parameter_value().double_value
        self.settle_duration = self.get_parameter('initial_settle_duration').get_parameter_value().double_value
        self.step_active_duration = self.get_parameter('step_duration').get_parameter_value().double_value
        self.post_step_log_duration = self.get_parameter('post_step_duration').get_parameter_value().double_value
        self.log_period = 1.0 / self.get_parameter('log_frequency').get_parameter_value().double_value
        self.log_file_path_str = self.get_parameter('log_file_path').get_parameter_value().string_value
        
        self.get_logger().info(f"--- OLTF Test Node Configuration ---")
        self.get_logger().info(f"Testing joint: {self.joint_to_test}")
        self.get_logger().info(f"Command topic: {self.command_topic}")
        self.get_logger().info(f"Step effort: {self.step_effort} Nm")
        self.get_logger().info(f"Settle duration: {self.settle_duration} s")
        self.get_logger().info(f"Step duration: {self.step_active_duration} s")
        self.get_logger().info(f"Post-step log duration: {self.post_step_log_duration} s")
        self.get_logger().info(f"Log frequency: {1.0/self.log_period} Hz")
        self.get_logger().info(f"Log file: {self.log_file_path_str}")
        self.get_logger().info(f"------------------------------------")

        # Ensure log directory exists
        log_file_path_obj = Path(self.log_file_path_str)
        if log_file_path_obj.parent != Path("."): # Check if there's a parent directory specified
            log_file_path_obj.parent.mkdir(parents=True, exist_ok=True)

        # Publisher for effort commands
        self.effort_pub = self.create_publisher(Float64MultiArray, self.command_topic, 10)

        # Subscriber for joint states
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_states_callback, rclpy.qos.qos_profile_sensor_data)

        # CSV file setup
        self.log_file = open(self.log_file_path_str, 'w', newline='')
        self.csv_writer = csv.writer(self.log_file)
        self.csv_writer.writerow(['sim_time_sec', 'commanded_effort_Nm', 'joint_position_rad', 'joint_velocity_rad_per_sec'])

        # State variables
        self.latest_joint_position = 0.0
        self.latest_joint_velocity = 0.0
        self.joint_idx = -1
        self.current_commanded_effort = 0.0

        # Test sequence management
        self.test_start_time = self.get_clock().now()
        self.test_phase = "INITIAL_SETTLE" # INITIAL_SETTLE, STEP_APPLIED, POST_STEP, FINISHED

        # Timer for the main test sequence logic and publishing commands
        self.control_timer_period = 0.02  # Publish commands at 50 Hz
        self.control_timer = self.create_timer(self.control_timer_period, self.run_test_sequence)
        
        # Timer for logging data at the specified frequency
        self.logging_timer = self.create_timer(self.log_period, self.log_data)

        self.get_logger().info("OLTF Test Node initialized. Starting test sequence...")

    def joint_states_callback(self, msg: JointState):
        if self.joint_idx == -1: # Find index only once
            try:
                self.joint_idx = msg.name.index(self.joint_to_test)
            except ValueError:
                self.get_logger().warn(f"Joint '{self.joint_to_test}' not found in /joint_states. Available: {msg.name}", once=True)
                return
        
        if self.joint_idx < len(msg.position) and self.joint_idx < len(msg.velocity):
            self.latest_joint_position = msg.position[self.joint_idx]
            self.latest_joint_velocity = msg.velocity[self.joint_idx]
        else:
            self.get_logger().warn(f"Index {self.joint_idx} for '{self.joint_to_test}' out of bounds for position/velocity arrays.", once=True)


    def log_data(self):
        if self.test_phase == "FINISHED":
            return

        current_sim_time_sec = self.get_clock().now().nanoseconds / 1e9
        self.csv_writer.writerow([
            current_sim_time_sec,
            self.current_commanded_effort,
            self.latest_joint_position,
            self.latest_joint_velocity
        ])

    def run_test_sequence(self):
        current_ros_time = self.get_clock().now()
        elapsed_time_sec = (current_ros_time - self.test_start_time).nanoseconds / 1e9
        
        next_commanded_effort = 0.0

        if self.test_phase == "INITIAL_SETTLE":
            next_commanded_effort = 0.0
            if elapsed_time_sec >= self.settle_duration:
                self.test_phase = "STEP_APPLIED"
                self.test_start_time = current_ros_time # Reset timer for the step phase
                self.get_logger().info(f"INITIAL_SETTLE finished. Applying step effort: {self.step_effort} Nm")
        
        elif self.test_phase == "STEP_APPLIED":
            next_commanded_effort = self.step_effort
            # Note: elapsed_time_sec is now relative to the start of STEP_APPLIED phase
            if elapsed_time_sec >= self.step_active_duration:
                self.test_phase = "POST_STEP"
                self.test_start_time = current_ros_time # Reset timer for post-step phase
                self.get_logger().info("STEP_APPLIED finished. Returning effort to 0 for post-step logging.")
        
        elif self.test_phase == "POST_STEP":
            next_commanded_effort = 0.0
            if elapsed_time_sec >= self.post_step_log_duration:
                self.test_phase = "FINISHED"
                self.get_logger().info("POST_STEP finished. Test complete.")
        
        elif self.test_phase == "FINISHED":
            next_commanded_effort = 0.0 # Ensure effort remains zero
            self.effort_pub.publish(Float64MultiArray(data=[next_commanded_effort])) # Publish one last zero
            self.current_commanded_effort = next_commanded_effort # For final log entry if any
            self.get_logger().info("Shutting down OLTF Test Node.")
            self.control_timer.cancel()
            self.logging_timer.cancel()
            self.log_file.close()
            rclpy.shutdown() # Shutdown the rclpy context
            return

        self.current_commanded_effort = next_commanded_effort
        effort_msg = Float64MultiArray()
        effort_msg.data = [self.current_commanded_effort]
        self.effort_pub.publish(effort_msg)

    def destroy_node(self):
        self.get_logger().info("OLTF Test Node shutting down.")
        if hasattr(self, 'log_file') and not self.log_file.closed:
            self.log_file.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        oltf_test_node = OLTFTestNode()
        rclpy.spin(oltf_test_node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        if 'oltf_test_node' in locals() and oltf_test_node:
            oltf_test_node.get_logger().error(f"Unhandled exception: {e}")
    finally:
        # Node might already be destroyed if shutdown was called internally
        if 'oltf_test_node' in locals() and oltf_test_node and rclpy.ok():
             if hasattr(oltf_test_node, 'log_file') and not oltf_test_node.log_file.closed:
                oltf_test_node.log_file.close()
             oltf_test_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()