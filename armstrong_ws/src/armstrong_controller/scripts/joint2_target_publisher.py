#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import math

class Joint2TargetPublisher(Node):
    def __init__(self):
        super().__init__('joint2_target_publisher')

        self.declare_parameter('amplitude', 0.8)
        self.declare_parameter('period', 2.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('topic', '/joint2_target_angle')

        self.amplitude = self.get_parameter('amplitude').get_parameter_value().double_value
        self.period = self.get_parameter('period').get_parameter_value().double_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.topic = self.get_parameter('topic').get_parameter_value().string_value

        self.publisher = self.create_publisher(Float64, self.topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_target)

        self.start_time = self.get_clock().now().nanoseconds * 1e-9
        self.get_logger().info(
            f"Publishing square wave to {self.topic} with amplitude {self.amplitude}, period {self.period}s"
        )

    def publish_target(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        t = now - self.start_time
        # Square wave: alternate every half period
        half_period = self.period / 2.0
        value = self.amplitude if (int(t / half_period) % 2 == 0) else -self.amplitude
        msg = Float64()
        msg.data = value
        self.publisher.publish(msg)
        self.get_logger().debug(f"Published: {value:.3f}")

def main(args=None):
    rclpy.init(args=args)
    node = Joint2TargetPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()