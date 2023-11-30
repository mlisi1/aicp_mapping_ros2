from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ros2pkg.api import get_prefix_path
import os


def generate_launch_description():
    return LaunchDescription([
        
     
        # Add other DeclareLaunchArgument for each argument in the ROS 1 launch file
        
        Node(
            package='aicp_ros',
            executable='aicp_ros_node',
            output='screen',
            arguments= [
                "--ros-args",
                "--log-level",
                "aicp_ros_node:=info",
            ],
            parameters=[{
                'registration_config_file': os.path.join(get_prefix_path("aicp_core"), 'share', "aicp_core", "config/icp/icp_autotuned.yaml"),
                'aicp_config_file' :  os.path.join(get_prefix_path("aicp_core"), 'share', "aicp_core", "config/aicp_config.yaml"),
                'registration_default_config_file' : os.path.join(get_prefix_path("aicp_core"), 'share', "aicp_core", "/config/icp/icp_autotuned_default.yaml"),
                 'verbose' : True,
            },]
        ),

        Node(
            package='velodyne_streamer',
            executable='streamer',
            remappings=[('cloud_in', '/point_cloud_filter/velodyne/point_cloud_filtered')],
            parameters=[{
                'publish_rate': 30
            }]
        )
    ])
