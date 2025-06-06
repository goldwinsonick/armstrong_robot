#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
import csv
import time

class OltfDataAcqNode(Node):
    def __init__(self):
        super().__init__('oltf_data_acq_node')
        self.declare_parameter('joint_name_to_test', 'joint2')
        self.declare_parameter('effort_command_topic', '/joint2_effort_controller/commands')
        self.declare_parameter('initial_settle_duration', 2.0)
        self.declare_parameter('torque_sequence', [160.0, 5.0, 100.0, 2.5])
        self.declare_parameter('final_settle_duration', 3.0)
        self.declare_parameter('log_frequency', 50.0)
        self.declare_parameter('log_file_path', './test1.csv')

        self.joint_name = self.get_parameter('joint_name_to_test').get_parameter_value().string_value
        self.effort_command_topic = self.get_parameter('effort_command_topic').get_parameter_value().string_value
        self.initial_settle_duration = self.get_parameter('initial_settle_duration').get_parameter_value().double_value
        self.torque_sequence = self.get_parameter('torque_sequence').get_parameter_value().double_array_value
        self.final_settle_duration = self.get_parameter('final_settle_duration').get_parameter_value().double_value
        self.log_frequency = self.get_parameter('log_frequency').get_parameter_value().double_value
        self.log_file_path = self.get_parameter('log_file_path').get_parameter_value().string_value

        self.torque_steps = []
        seq = list(self.torque_sequence)
        for i in range(0, len(seq), 2):
            self.torque_steps.append((seq[i], seq[i+1]))

        self.effort_pub = self.create_publisher(Float64MultiArray, self.effort_command_topic, 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        self.current_joint_angle = 0.0
        self.joint_state_received = False

        self.csv_file = open(self.log_file_path, 'w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['time', 'torque', 'joint_angle'])

        self.get_logger().info('Waiting for joint state...')
        while not self.joint_state_received:
            rclpy.spin_once(self, timeout_sec=0.1)

        self.get_logger().info('Starting initial settle...')
        self.apply_torque_and_log(0.0, self.initial_settle_duration)

        for torque, duration in self.torque_steps:
            self.get_logger().info(f'Applying {torque} Nm for {duration} s')
            self.apply_torque_and_log(torque, duration)

        self.get_logger().info('Final settle...')
        self.apply_torque_and_log(0.0, self.final_settle_duration)

        self.get_logger().info('Experiment finished. Closing file.')
        self.csv_file.close()
        rclpy.shutdown()

    def joint_state_callback(self, msg):
        if self.joint_name in msg.name:
            idx = msg.name.index(self.joint_name)
            self.current_joint_angle = msg.position[idx]
            self.joint_state_received = True

    def apply_torque_and_log(self, torque, duration):
        msg = Float64MultiArray()
        msg.data = [torque]
        start_time = time.time()
        end_time = start_time + duration
        period = 1.0 / self.log_frequency
        while True:
            now = time.time()
            if now >= end_time:
                break
            self.effort_pub.publish(msg)
            self.csv_writer.writerow([now, torque, self.current_joint_angle])
            self.csv_file.flush()
            rclpy.spin_once(self, timeout_sec=0)
            time.sleep(period)

def main(args=None):
    rclpy.init(args=args)
    node = OltfDataAcqNode()

if __name__ == '__main__':
    main()