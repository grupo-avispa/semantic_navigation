#!/usr/bin/env python3

'''
    Launches all the nodes required for the semantic navigation.
'''
import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Getting directories and launch-files
    semantic_goals_generator_dir = get_package_share_directory('semantic_goals_generator')
    default_params_file = os.path.join(semantic_goals_generator_dir, 'params', 'default_params.yaml')
    default_rois_params_file = os.path.join(semantic_goals_generator_dir, 'params', 'rois.yaml')

    # Input parameters declaration
    params_file = LaunchConfiguration('params_file')
    rois_params_file = LaunchConfiguration('rois_filename')
    log_level = LaunchConfiguration('log_level')

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value = default_params_file,
        description = 'Full path to the ROS2 parameters file with semantic goals generator configuration'
    )

    declare_rois_filename_arg = DeclareLaunchArgument(
        'rois_filename',
        default_value = default_rois_params_file,
        description = 'Full path to the ROS2 parameters file with the ROIs'
    )

    declare_log_level_arg = DeclareLaunchArgument(
        name = 'log_level',
        default_value = 'info',
        description = 'Logging level (info, debug, ...)'
    )

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'rois_filename': rois_params_file, 
    }

    configured_params = RewrittenYaml(
        source_file = params_file,
        root_key = '',
        param_rewrites = param_substitutions,
        convert_types = True
    )

    # Prepare the semantic goals generator node.
    semantic_goals_generator_node = Node(
        package = 'semantic_goals_generator',
        namespace = '',
        executable = 'semantic_goals_generator',
        name = 'semantic_goals_generator',
        parameters = [configured_params],
        emulate_tty = True,
        output = 'screen', 
        arguments = ['--ros-args', '--log-level', ['semantic_goals_generator:=', log_level]]
    )
    return LaunchDescription([
        declare_params_file_arg,
        declare_rois_filename_arg,
        declare_log_level_arg,
        semantic_goals_generator_node
    ])