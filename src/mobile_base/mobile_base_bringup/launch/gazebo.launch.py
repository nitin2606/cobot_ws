from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    package_share_directory = get_package_share_directory("mobile_base_description")
    sdf_file_path = os.path.join(package_share_directory, "worlds", "amr_world.sdf")

    return LaunchDescription([
        # Launch Ignition Gazebo with the specified SDF file
        ExecuteProcess(
            cmd=['ign', 'gazebo', sdf_file_path],
            output='screen'
        ),

        #Bridge for /cmd_vel topic
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            name='cmd_vel_bridge',
            output='screen',
            arguments=['/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist']
        ),

        # Bridge for /camera topic
        Node(
            package='ros_ign_bridge',
            executable='parameter_bridge',
            name='camera_bridge',
            output='screen',
            arguments=['/front_camera@sensor_msgs/msg/Image@ignition.msgs.Image']
        ),
    ])
