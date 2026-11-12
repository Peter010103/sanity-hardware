from datetime import datetime
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    keys=['workspace','profile','mission_profile','output','worker_python']
    booleans=['enable_output','no_transport','takeoff_land_only']
    return LaunchDescription([DeclareLaunchArgument('workspace',description='Absolute drones workspace containing loco_mjx and the Python environments'),
        DeclareLaunchArgument('profile',default_value=PathJoinSubstitution([FindPackageShare('sanity_zenoh'),'config','sanity_plate_108g.json'])),
        DeclareLaunchArgument('output',default_value=PathJoinSubstitution([LaunchConfiguration('workspace'),'outputs','sanity_plate_'+datetime.now().strftime('%Y%m%d_%H%M%S')])),
        *[DeclareLaunchArgument(k,default_value='') for k in ['mission_profile','worker_python']],
        *[DeclareLaunchArgument(k,default_value='false') for k in booleans],
        Node(package='sanity_zenoh',executable='sanity_plate',name='sanity_plate',output='screen',
             parameters=[{k:LaunchConfiguration(k) for k in keys},
                         {k:ParameterValue(LaunchConfiguration(k),value_type=bool) for k in booleans}])])
