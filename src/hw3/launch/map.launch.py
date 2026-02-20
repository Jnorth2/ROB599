#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetLaunchConfiguration

from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node



def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time',  default='true')
    hw2_directory = get_package_share_directory('hw2')
    hw3_directory = get_package_share_directory('hw3')
    # launch_dir = os.path.join(this_directory, 'launch')

    stage_world_arg = DeclareLaunchArgument(
    'world',
    default_value=TextSubstitution(text='open'),
    description='World file relative to the project world file, without .world')

    is_dwa = LaunchConfiguration('is_dwa')
    is_dwa_arg = DeclareLaunchArgument(
        'is_dwa',
        default_value='True',
        description="True if running DWA controller"
    )

    save_map = LaunchConfiguration('save_map')
    save_map_arg = DeclareLaunchArgument(
        'save_map',
        default_value='False',
        description="True to periodically save the map as an image"
    )
    save_path = LaunchConfiguration('save_path')
    save_path_arg = DeclareLaunchArgument(
        'save_path',
        default_value='False',
        description="True to save the path"
    )

    use_twist_stamped = LaunchConfiguration('use_twist_stamped')
    use_twist_stamped_arg = DeclareLaunchArgument(
        'use_twist_stamped',
        default_value='False',
        description="True to use publish twist stamped messages"
    )

    enforce_prefixes = LaunchConfiguration('enforce_prefixes')
    enforce_prefixes_cmd = DeclareLaunchArgument(
        'enforce_prefixes',
        default_value='false',
        description='on true a prefixes are used for a single robot environment')
    

    one_tf_tree = LaunchConfiguration('one_tf_tree')
    one_tf_tree_cmd = DeclareLaunchArgument(
        'one_tf_tree',
        default_value='false',
        description='on true all tfs are published with a namespace on /tf and /tf_static')
   
    namespace = LaunchConfiguration('namespace')
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    
    use_static_transformations = LaunchConfiguration('use_static_transformations')
    use_static_transformations_arg = DeclareLaunchArgument(
        'use_static_transformations',
        default_value='true',
        description='Use static transformations for sensor frames!')
    
    rviz_config = LaunchConfiguration('config')
    rviz_config_arg = DeclareLaunchArgument(
        'config',
        default_value=TextSubstitution(text='rviz_config'),
        description='Use empty, cave or roblab to load a TUW enviroment')
    
    
    def stage_world_configuration(context):
        file = os.path.join(
            hw2_directory,
            'worlds',
            context.launch_configurations['world'] + '.world')
        return [SetLaunchConfiguration('world_file', file)]

    stage_world_configuration_arg = OpaqueFunction(function=stage_world_configuration)

    def rviz_launch_configuration(context):
        file = os.path.join(
            hw3_directory,
            'config',
            context.launch_configurations['config'] + '.rviz')
        return [SetLaunchConfiguration('config', file)]

    rviz_launch_configuration_arg = OpaqueFunction(function=rviz_launch_configuration)

    mapping_node = Node(
        package="hw3",
        executable="mapping",
        name="mapping",
        parameters=[{
            'use_twist_stamped': use_twist_stamped,
            'save_map': save_map

        }]

    )

    planning_node = Node(
        package="hw3",
        executable="path_planning",
        name="path_planning",
        parameters=[{
            'save_path': save_path
        }]
    )


    return LaunchDescription([
        stage_world_arg,
        declare_namespace_cmd,
        rviz_config_arg,
        enforce_prefixes_cmd,
        one_tf_tree_cmd,
        use_static_transformations_arg,
        stage_world_configuration_arg,
        rviz_launch_configuration_arg,
        save_map_arg,
        save_path_arg,
        is_dwa_arg,
        use_twist_stamped_arg,
        Node(
            package='rviz2',
            namespace=namespace,
            executable='rviz2',
            name='rviz2',
            arguments=['-d', [rviz_config]],
            parameters=[{
                "use_sim_time": use_sim_time}],
        ),
        Node(
            package='stage_ros2',
            executable='stage_ros2',
            name='stage',
            parameters=[{'one_tf_tree': one_tf_tree,
                        'enforce_prefixes': enforce_prefixes,
                        'use_static_transformations': use_static_transformations,
                "world_file": [LaunchConfiguration('world_file')]}],
        ),
        Node(
            package="hw2",
            executable="driver",
            name="driver",
            parameters=[{
                'is_dwa': is_dwa,
                'use_twist_stamped' : use_twist_stamped
            }]
        ),
        mapping_node,
        planning_node,
    ])
