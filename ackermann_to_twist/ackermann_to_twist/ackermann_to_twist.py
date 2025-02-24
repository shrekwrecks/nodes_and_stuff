import math
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist

class AckermannToTwistNode(Node):
    def __init__(self):
        super().__init__('ackermann_to_twist')

        # Subscriber for the AckermannDrive message
        self.create_subscription(
            AckermannDriveStamped,
            'ackermann_fart',  # The topic name you are subscribing to
            self.ackermann_callback,
            10  # QoS
        )

        # Publisher for the Twist message
        self.twist_publisher = self.create_publisher(
            Twist,
            'ack_vel',  # The topic name you will publish to
            10  # QoS
        )

    def ackermann_callback(self, msg: AckermannDriveStamped):
        # Converting AckermannDrive to Twist
        twist = Twist()

        # Forward velocity (linear.x) is the speed
        twist.linear.x = msg.drive.speed

        # Angular velocity (angular.z) is based on steering angle and speed
        wheelbase = 0.335  # meters, adjust to your robot's wheelbase
        if msg.drive.steering_angle != 0:
            # Using the tangent of the steering angle to calculate angular velocity
            twist.angular.z = (msg.drive.speed / wheelbase) * math.tan(msg.drive.steering_angle)
        else:
            twist.angular.z = 0.0

        # Log the conversion
      #   self.get_logger().info(f"Converted Ackermann: speed={msg.drive.speed}, steering_angle={msg.drive.steering_angle} to Twist: linear.x={twist.linear.x}, angular.z={twist.angular.z}")

        # Publish the Twist message
        self.twist_publisher.publish(twist)


def main(args=None):
    rclpy.init(args=args)

    node = AckermannToTwistNode()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
