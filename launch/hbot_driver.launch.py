import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
import launch_ros.actions
from launch_ros.descriptions import ParameterFile
import yaml

def generate_launch_description():
  package_name = 'hbot_driver_yahboom'
  node_name = 'hbot_driver_yahboom_node'

  debug =  LaunchConfiguration('debug')
  params_file = LaunchConfiguration('params_file')
  log_level = LaunchConfiguration('log_level')

  debug_arg = DeclareLaunchArgument(
    'debug',
    default_value='false',
    description='Enable debug mode'
  )
  params_file_arg = DeclareLaunchArgument('params_file',
                                         default_value=os.path.join(
                                           get_package_share_directory('hbot_driver_yahboom'),
                                           'config',
                                           'params.yaml'
                                         ),
                                          description='Path to the ROS2 parameters file to use.')

  declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info',
        description='log level')

  param_subsitutions = {
    'debug': debug
  }

  configured_params = ParameterFile(
    RewrittenYaml(
      source_file=params_file,
      root_key='',
      param_rewrites=param_subsitutions,
      convert_types=True
    ),
    allow_substs=True
  )

  node = launch_ros.actions.Node(
    package='hbot_driver_yahboom',
    executable='hbot_driver_yahboom_node',
    output='screen',
    parameters=[configured_params],
    arguments=['--ros-args', '--log-level', log_level]
  )

  ld = LaunchDescription()
  ld.add_action(debug_arg)
  ld.add_action(params_file_arg)
  ld.add_action(declare_log_level_cmd)

  ld.add_action(node)

  return ld
