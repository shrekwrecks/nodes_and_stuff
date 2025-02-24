import rclpy
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import math

class OdomToTFPublisher(Node):
    def __init__(self):
        super().__init__('odom_to_tf_broadcaster')

        # Access the existing parameter 'use_sim_time' (don't declare it again)
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value

        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to the /odom/filtered topic for dynamic pose
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odometry/filtered',  # Subscribe to the odom/filtered topic
            self.odom_callback,
            10
        )

        self.get_logger().info("Publishing odom/filtered->base_link transform to /tf")

    def odom_callback(self, msg):
        # Extract position and orientation from Odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = self.euler_from_quaternion(
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        )

        # Log the received odometry data
        # self.get_logger().info(f"Received Odometry - X: {x}, Y: {y}, Theta: {theta}")

        # Create a TransformStamped message to broadcast the dynamic transform
        transform = TransformStamped()

        # Use simulated time if it's enabled, otherwise use real time
        if self.use_sim_time:
            transform.header.stamp = self.get_clock().now().to_msg()  # Simulated time
        else:
            transform.header.stamp = self.get_clock().now().to_msg()  # Real time

        transform.header.frame_id = 'odom'  # The parent frame (odom)
        transform.child_frame_id = 'base_link'  # The child frame (robot base)

        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0  # Assuming flat (2D) surface

        # Convert Euler angle to quaternion for rotation
        q = self.euler_to_quaternion(theta)
        transform.transform.rotation.x = q[0]
        transform.transform.rotation.y = q[1]
        transform.transform.rotation.z = q[2]
        transform.transform.rotation.w = q[3]

        # Publish the updated transform to /tf
        self.tf_broadcaster.sendTransform(transform)

    def euler_from_quaternion(self, x, y, z, w):
        # Convert quaternion to Euler angles (yaw, pitch, roll)
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def euler_to_quaternion(self, theta):
        # Convert Euler angle to quaternion
        qx = 0.0
        qy = 0.0
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)

    node = OdomToTFPublisher()

    # You don't need to set the 'use_sim_time' parameter here since it is already
    # declared in the system. It will be available if use_sim_time is set in the ROS 2 launch file.

    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
