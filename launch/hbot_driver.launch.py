import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from nav2_common.launch import RewrittenYaml
import launch_ros.actions
import yaml

def generate_launch_description():
  package_name = 'hbot_driver_yahboom'
  node_name = 'hbot_driver_yahboom_node'

  debug =  LaunchConfiguration('debug')
  params_file = LaunchConfiguration('params_file')

  debug_arg = DeclareLaunchArgument(
    'debug',
    default_value='false',
    description='Enable debug mode'
  )
  params_file_arg = DeclareLaunchArgument('params_file',
                                         default_value=os.path.join(
                                           get_package_share_directory(package_name),
                                           'config',
                                           'params.yaml'
                                         ),
                                          description='Path to the ROS2 parameters file to use.')

  param_subsitutions = {
    'debug': debug
  }

  configured_params = RewrittenYaml(
    source_file=params_file,
    root_key='',
    param_rewrites=param_subsitutions,
    convert_types=True
  )

  node = launch_ros.actions.Node(
    package=package_name,
    executable=node_name,
    output='screen',
    parameters=[configured_params]
  )

  ld = LaunchDescription()
  ld.add_action(debug_arg)
  ld.add_action(params_file_arg)

  ld.add_action(node)

  return ld
