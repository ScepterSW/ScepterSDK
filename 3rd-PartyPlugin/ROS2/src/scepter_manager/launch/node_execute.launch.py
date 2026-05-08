import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import LoadComposableNodes
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

from launch import LaunchContext

import yaml

def declare_configurable_parameters(parameters):
    declareLaunchArgumentList = []
    for key,val in parameters.items():
        defaultValue = "{}".format(val)
        if str(key).strip().lower() == "camera_name":
            defaultValue = "vzense_{}".format(val)
        declareLaunchArgumentList.append(DeclareLaunchArgument("{}".format(key), default_value = defaultValue, description= "{}".format(key)))
    return declareLaunchArgumentList

def set_configurable_parameters(parameters):
    return dict([("{}".format(key), LaunchConfiguration("{}".format(key))) for key,val in parameters.items()])

def launch_setup(context, camera_param_path, params):
    return [
            Node(
            package='scepter_manager',
            namespace="",
            name = context.launch_configurations['camera_name'],
            executable='scepter_manager_node_exe',
            parameters=[camera_param_path, params],
            output="screen",
            arguments=["--ros-args", "--log-level", "info"],
            emulate_tty=True,
            )
    ]

def generate_launch_description():

    context = LaunchContext()

    camera_param_path = os.path.join(
        FindPackageShare("scepter_manager").perform(context),
        "param/default.param.yaml"
    )

    with open(camera_param_path, "r") as f:
        camera_yaml_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    static_tf_node = Node(
        name="static_tf_vzense_camera_node",
        namespace = "",
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "base_link", "vzense_{}_link".format(camera_yaml_param['camera_name'])]
    )
    return LaunchDescription(
        declare_configurable_parameters(camera_yaml_param) + 
        [
            static_tf_node,
            OpaqueFunction(function=launch_setup,
            kwargs = {  'camera_param_path' : camera_param_path,
                        'params' : set_configurable_parameters(camera_yaml_param),
                    }),
        ]
    )