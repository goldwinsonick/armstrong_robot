#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from pynput import keyboard
import threading
import time

# Instructions to be displayed
MSG_TEMPLATE = """
-----------------------------------
Armstrong Arm Effort Teleop (pynput)
-----------------------------------
Joint 1:
'{j1_pos}' : positive effort ({eff_j1} Nm)
'{j1_neg}' : negative effort (-{eff_j1} Nm)

Joint 2:
'{j2_pos}' : positive effort ({eff_j2} Nm)
'{j2_neg}' : negative effort (-{eff_j2} Nm)
-----------------------------------
Press '{quit_key}' to quit.
Effort is applied while a key is held.
Releasing a key stops effort for that action.
Publishing at {rate} Hz.
"""

class EffortTeleopNode(Node):
    def __init__(self):
        super().__init__('effort_teleop_node')

        # Declare and get parameters
        self.declare_parameter('joint1_command_topic', '/joint1_effort_controller/commands')
        self.declare_parameter('joint2_command_topic', '/joint2_effort_controller/commands')
        self.declare_parameter('effort_magnitude_j1', 2.0)
        self.declare_parameter('effort_magnitude_j2', 1.0)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('key_mappings.joint1_positive', 'w')
        self.declare_parameter('key_mappings.joint1_negative', 's')
        self.declare_parameter('key_mappings.joint2_positive', 'o')
        self.declare_parameter('key_mappings.joint2_negative', 'l')
        self.declare_parameter('key_mappings.quit', 'q')

        self.j1_cmd_topic = self.get_parameter('joint1_command_topic').get_parameter_value().string_value
        self.j2_cmd_topic = self.get_parameter('joint2_command_topic').get_parameter_value().string_value
        self.effort_j1 = self.get_parameter('effort_magnitude_j1').get_parameter_value().double_value
        self.effort_j2 = self.get_parameter('effort_magnitude_j2').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.key_map = {
            'j1_pos': self.get_parameter('key_mappings.joint1_positive').get_parameter_value().string_value,
            'j1_neg': self.get_parameter('key_mappings.joint1_negative').get_parameter_value().string_value,
            'j2_pos': self.get_parameter('key_mappings.joint2_positive').get_parameter_value().string_value,
            'j2_neg': self.get_parameter('key_mappings.joint2_negative').get_parameter_value().string_value,
            'quit': self.get_parameter('key_mappings.quit').get_parameter_value().string_value,
        }

        # Create publishers
        self.j1_effort_pub = self.create_publisher(Float64MultiArray, self.j1_cmd_topic, 10)
        self.j2_effort_pub = self.create_publisher(Float64MultiArray, self.j2_cmd_topic, 10)

        # Effort states
        self.target_effort_j1 = 0.0
        self.target_effort_j2 = 0.0
        self.active_keys = set() # Set to store currently pressed relevant keys

        # Keyboard listener setup
        self.listener_thread = threading.Thread(target=self.keyboard_listener_thread_func, daemon=True)
        self.listener_thread.start()
        
        # Timer for publishing commands
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_efforts_callback)

        self.get_logger().info(MSG_TEMPLATE.format(
            j1_pos=self.key_map['j1_pos'], eff_j1=self.effort_j1,
            j1_neg=self.key_map['j1_neg'],
            j2_pos=self.key_map['j2_pos'], eff_j2=self.effort_j2,
            j2_neg=self.key_map['j2_neg'],
            quit_key=self.key_map['quit'],
            rate=self.publish_rate
        ))
        self.get_logger().info("Teleop node started. Press keys in this terminal.")
        
        # Register shutdown hook
        rclpy.get_default_context().on_shutdown(self._on_shutdown)


    def _key_to_char(self, key):
        """Converts pynput key object to character if possible."""
        if hasattr(key, 'char'):
            return key.char
        elif isinstance(key, keyboard.Key): # Special keys like Key.esc
            if key == keyboard.Key.esc and self.key_map['quit'] == 'Key.esc': # If 'quit' is configured as ESC
                 return 'Key.esc' 
            return None # Or some other representation for special keys if needed
        return None

    def on_press(self, key):
        char_key = self._key_to_char(key)
        if char_key:
            self.active_keys.add(char_key)
            if char_key == self.key_map['quit']:
                self.get_logger().info("Quit key pressed, shutting down...")
                rclpy.try_shutdown() # Request shutdown

    def on_release(self, key):
        char_key = self._key_to_char(key)
        if char_key in self.active_keys:
            self.active_keys.remove(char_key)
        if char_key == self.key_map['quit']: # Ensure quit works on release too if held
            self.get_logger().info("Quit key released, ensuring shutdown...")
            rclpy.try_shutdown()


    def keyboard_listener_thread_func(self):
        with keyboard.Listener(on_press=self.on_press, on_release=self.on_release) as listener:
            listener.join() # Blocks until listener stops (e.g., on error or explicitly)
        self.get_logger().info("Keyboard listener thread stopped.")


    def update_and_publish_efforts(self):
        # Determine target efforts based on active keys
        new_target_effort_j1 = 0.0
        if self.key_map['j1_pos'] in self.active_keys:
            new_target_effort_j1 = self.effort_j1
        elif self.key_map['j1_neg'] in self.active_keys:
            new_target_effort_j1 = -self.effort_j1
        
        new_target_effort_j2 = 0.0
        if self.key_map['j2_pos'] in self.active_keys:
            new_target_effort_j2 = self.effort_j2
        elif self.key_map['j2_neg'] in self.active_keys:
            new_target_effort_j2 = -self.effort_j2

        self.target_effort_j1 = new_target_effort_j1
        self.target_effort_j2 = new_target_effort_j2

        # Publish
        j1_msg = Float64MultiArray()
        j1_msg.data = [self.target_effort_j1]
        self.j1_effort_pub.publish(j1_msg)

        j2_msg = Float64MultiArray()
        j2_msg.data = [self.target_effort_j2]
        self.j2_effort_pub.publish(j2_msg)

    def publish_efforts_callback(self):
        """Called by the ROS2 timer."""
        if not rclpy.ok(): # If shutdown initiated by quit key
            return
        self.update_and_publish_efforts()
        
    def _on_shutdown(self):
        self.get_logger().info("Node shutting down, sending zero efforts.")
        # Stop the listener if it's still running (pynput might require this)
        # keyboard.Listener.stop() # This needs to be called on the listener instance
                                  # However, the listener thread is daemonized, should exit with main.
                                  # For cleaner exit, listener.stop() might be better if we keep listener instance.
                                  # For simplicity now, rely on daemon thread.
        
        self.target_effort_j1 = 0.0
        self.target_effort_j2 = 0.0
        # Try to publish zero efforts a few times to ensure they are sent
        for _ in range(5): 
            j1_msg = Float64MultiArray()
            j1_msg.data = [0.0]
            self.j1_effort_pub.publish(j1_msg)
            j2_msg = Float64MultiArray()
            j2_msg.data = [0.0]
            self.j2_effort_pub.publish(j2_msg)
            time.sleep(0.01) # Small delay


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = EffortTeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt: # For Ctrl+C from terminal running the node
        if node:
            node.get_logger().info("Ctrl+C detected, shutting down teleop node.")
    except Exception as e:
        if node:
            node.get_logger().error(f"Unhandled exception: {e}")
        else:
            print(f"Exception before node init: {e}")
    finally:
        # The shutdown hook and listener thread daemonization should handle cleanup
        if rclpy.ok():
             rclpy.shutdown()

if __name__ == '__main__':
    main()