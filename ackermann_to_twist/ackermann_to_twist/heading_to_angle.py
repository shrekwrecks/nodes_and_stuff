import rclpy
import math
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32

class HeadingToAckermannNode(Node):
    def __init__(self):
        super().__init__('final_heading_to_ackermann')

        # Subscriber for the final_heading_angle topic (Float32)
        self.create_subscription(
            Float32,
            '/final_heading_angle',  # Topic name for final heading angle
            self.final_heading_callback,
            10  # QoS
        )

        # Publisher for the AckermannDriveStamped message
        self.ackermann_publisher = self.create_publisher(
            AckermannDriveStamped,
            'ackermann_fart',  # Topic name for AckermannDriveStamped
            10  # QoS
        )

    def final_heading_callback(self, msg: Float32):
        # Create AckermannDriveStamped message
        ackermann_msg = AckermannDriveStamped()

        # Set the header with the current time and frame_id (optional)
        ackermann_msg.header.stamp = self.get_clock().now().to_msg()
        ackermann_msg.header.frame_id = ''  # You can set a frame_id if needed

        speed = 1.1
        max_speed = 1.5  # Maximum speed
        min_speed = 0.1  # Minimum speed
        steering_factor = 15 # A constant to adjust the sharpness of the curve
        # # Sigmoid function to modulate speed (change to constant for real test, very slow)
       # speed = min_speed + (max_speed - min_speed) * (1 / (1 + math.exp(steering_factor * (abs(msg.data) - 0.5))))
        # speed = max_speed
        # Max speed and min speed values

        # Normalize angle (assuming it's in degrees, convert to radians if needed)
        # Ensure that we map -max_steering_angle to +max_steering_angle to the speed range
        # speed = max(min_speed, max_speed * (1 - abs(msg.data) / 0.5))

        # Apply the speed to the Ackermann message
        ackermann_msg.drive.speed = speed


        # Set steering_angle to the final heading angle received from the topic
        ackermann_msg.drive.steering_angle = msg.data

        # Log the conversion
      #  self.get_logger().info(f"Received final_heading_angle: {msg.data}, Publishing AckermannDriveStamped: speed={ackermann_msg.drive.speed}, steering_angle={ackermann_msg.drive.steering_angle}")

        # Publish the AckermannDriveStamped message
        self.ackermann_publisher.publish(ackermann_msg)


def main(args=None):
    rclpy.init(args=args)

    node = HeadingToAckermannNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
