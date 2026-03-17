#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution, PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration, IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare


from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time',  default='true')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description="Use sim time"
    )

    hw5_directory = get_package_share_directory('hw5_jn')
    bringup_dir = get_package_share_directory('nav2_bringup')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    namespace = LaunchConfiguration('namespace')
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    
    rviz_config = LaunchConfiguration('config')
    rviz_config_arg = DeclareLaunchArgument(
        'config',
        default_value=TextSubstitution(text='rviz_config'),
        description='Use empty, cave or roblab to load a TUW enviroment')
    
    def rviz_launch_configuration(context):
        file = os.path.join(
            hw5_directory,
            'config',
            context.launch_configurations['config'] + '.rviz')
        return [SetLaunchConfiguration('config', file)]

    rviz_launch_configuration_arg = OpaqueFunction(function=rviz_launch_configuration)

    turtle_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("turtlebot3_gazebo"),
                "launch",
                "turtlebot3_house.launch.py",
            ])
        ),
    )

    map_file = (
        get_package_share_directory("hw5_jn") + "/config/map.yaml"
    )

    # nav2_node = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             FindPackageShare("nav2_bringup"),
    #             "launch",
    #             # "bringup_launch.py",
    #             "navigation_launch.py",
    #         ])
    #     ),
    #     launch_arguments={
    #         "use_sim_time": "true",
    #         "params_file": os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
    #         "slam": "False",
    #         "use_map_server": "True", 
    #         "autostart": "True",
    #         "map":map_file,
    #     }.items(),
    # )

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_house.world'
    )

    nav2_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("nav2_bringup"),
                "launch",
                # "bringup_launch.py",
                "tb3_simulation_launch.py",
            ])
        ),
        launch_arguments={
            "use_sim_time": "true",
            "params_file": os.path.join(bringup_dir, 'params', 'nav2_params.yaml'),
            "slam": "False",
            "use_map_server": "True", 
            "autostart": "True",
            "map":map_file,
            "headless": "False",
            "world": world,
        }.items(),
    )

    slam_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("slam_toolbox"),
                "launch",
                "online_async_launch.py",
            ])
        )
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )



    return LaunchDescription([
        declare_namespace_cmd,
        rviz_config_arg,
        rviz_launch_configuration_arg,

        # turtle_node,
        nav2_node,
        # slam_node,

        # Node(
        #     package='rviz2',
        #     namespace=namespace,
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', [rviz_config]],
        #     parameters=[{
        #         "use_sim_time": use_sim_time}],
        # ),
        # gzclient_cmd,

    ])
