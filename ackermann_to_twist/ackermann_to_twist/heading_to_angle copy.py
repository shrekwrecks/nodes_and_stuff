import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDrive
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

        # Publisher for the AckermannDrive message
        self.ackermann_publisher = self.create_publisher(
            AckermannDrive,
            'ackermann_fart',  # Topic name for AckermannDrive
            10  # QoS
        )

    def final_heading_callback(self, msg: Float32):
        # Create AckermannDrive message
        ackermann_msg = AckermannDrive()

        # Set speed to a constant value (3)
        ackermann_msg.speed = 2.0

        # Set steering_angle to the final heading angle received from the topic
        ackermann_msg.steering_angle = msg.data

        # Log the conversion
        self.get_logger().info(f"Received final_heading_angle: {msg.data}, Publishing AckermannDrive: speed={ackermann_msg.speed}, steering_angle={ackermann_msg.steering_angle}")

        # Publish the AckermannDrive message
        self.ackermann_publisher.publish(ackermann_msg)


def main(args=None):
    rclpy.init(args=args)

    node = HeadingToAckermannNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
