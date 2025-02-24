import os

from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

from launch_ros.actions import Node



def generate_launch_description():


    # Include the robot_state_publisher launch file, provided by our own package. Force sim time to be enabled
    # !!! MAKE SURE YOU SET THE PACKAGE NAME CORRECTLY !!!

    package_name='ackermann_to_twist' #<--- CHANGE ME

 
    ackermann_to_twist = Node(
        package="ackermann_to_twist",
        executable="ackermann_to_twist",
        parameters=[],
    )

    heading_to_angle = Node(
        package="ackermann_to_twist",
        executable="heading_to_angle",
        parameters=[],
    )

    odom_tf_pub = Node(
        package="ackermann_to_twist",
        executable="odom_tf_pub",
        name="odom_to_tf",
        parameters=[{'use_sim_time': True}],
    )
   

    # Launch them all!
    return LaunchDescription([
        ackermann_to_twist,
        heading_to_angle,
      #  odom_tf_pub,
    ])