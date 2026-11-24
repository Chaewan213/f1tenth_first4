"""전체 주행 스택 실행.

예)
  실차:        ros2 launch race_bringup race.launch.py
  시뮬(라이다): ros2 launch race_bringup race.launch.py use_lane:=false force_mode:=GAP \
                    odom_topic:=/ego_racecar/odom
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('race_bringup'), 'config', 'race_params.yaml')

    params = LaunchConfiguration('params_file')
    use_lane = LaunchConfiguration('use_lane')
    force_mode = LaunchConfiguration('force_mode')
    odom_topic = LaunchConfiguration('odom_topic')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_lane', default_value='true',
                              description='카메라 없는 시뮬에서는 false'),
        DeclareLaunchArgument('force_mode', default_value='AUTO',
                              description='AUTO(자동 전환) | GAP | LANE'),
        DeclareLaunchArgument('odom_topic', default_value='/odom'),

        Node(package='gap_planner', executable='gap_planner_node', name='gap_planner',
             parameters=[params], output='screen'),
        Node(package='lane_planner', executable='lane_planner_node', name='lane_planner',
             parameters=[params], output='screen', condition=IfCondition(use_lane)),
        Node(package='race_control', executable='mode_manager_node', name='mode_manager',
             parameters=[params, {'force_mode': force_mode}], output='screen'),
        Node(package='race_control', executable='tracker_node', name='tracker',
             parameters=[params, {'odom_topic': odom_topic}], output='screen'),
        Node(package='race_control', executable='safety_filter_node', name='safety_filter',
             parameters=[params, {'odom_topic': odom_topic}], output='screen'),
    ])
