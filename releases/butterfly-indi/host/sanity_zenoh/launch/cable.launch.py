from datetime import datetime
import os
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    workspace=LaunchConfiguration('workspace')
    package=FindPackageShare('sanity_zenoh')
    return LaunchDescription([
        DeclareLaunchArgument('workspace',default_value=os.environ.get('SANITY_WORKSPACE',str(Path.home()/'Projects/drones'))),
        DeclareLaunchArgument('mission',default_value='circle_slow'),
        DeclareLaunchArgument('controller',default_value='aware'),
        DeclareLaunchArgument('enable_output',default_value='false'),
        DeclareLaunchArgument('profile',default_value=PathJoinSubstitution([package,'config','sanity_cable.json'])),
        DeclareLaunchArgument('cable_engagement_config',default_value='',description='Optional JSON settings; shadow mode only'),
        DeclareLaunchArgument('mpc_hover_only',default_value='false'),
        DeclareLaunchArgument('takeoff_land_only',default_value='false'),
        DeclareLaunchArgument('hover_duration_s',default_value='5.0'),
        DeclareLaunchArgument('payload_mass_g',default_value='-1.0',description='Payload plus markers, excluding cable; -1 uses the physical profile'),
        DeclareLaunchArgument('skip_ground_height_check',default_value='false'),
        DeclareLaunchArgument('output',default_value=PathJoinSubstitution([workspace,'outputs','sanity_cable_'+datetime.now().strftime('%Y%m%d_%H%M%S')])),
        Node(package='sanity_zenoh',executable='sanity_cable',name='sanity_cable',output='screen',
             prefix=[PathJoinSubstitution([workspace,'.venv-sanity-deploy','bin','python'])],
             parameters=[{'loco_root':PathJoinSubstitution([workspace,'loco_mjx']),
                          'profile':LaunchConfiguration('profile'),
                          'cable_engagement_config':LaunchConfiguration('cable_engagement_config'),
                          'mpc_hover_only':ParameterValue(LaunchConfiguration('mpc_hover_only'),value_type=bool),
                          'takeoff_land_only':ParameterValue(LaunchConfiguration('takeoff_land_only'),value_type=bool),
                          'hover_duration_s':ParameterValue(LaunchConfiguration('hover_duration_s'),value_type=float),
                          'payload_mass_g':ParameterValue(LaunchConfiguration('payload_mass_g'),value_type=float),
                          'skip_ground_height_check':ParameterValue(LaunchConfiguration('skip_ground_height_check'),value_type=bool),
                          'esp_collective_profile':PathJoinSubstitution([package,'config','sanity09_esp_collective.json']),
                          'mission':LaunchConfiguration('mission'),'controller':LaunchConfiguration('controller'),
                          'enable_output':ParameterValue(LaunchConfiguration('enable_output'),value_type=bool),
                          'output':LaunchConfiguration('output')}])])
