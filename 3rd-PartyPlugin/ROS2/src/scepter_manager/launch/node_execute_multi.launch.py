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

def declare_configurable_parameters(parameters, camera_index):
    declareLaunchArgumentList = []
    for key,val in parameters.items():
        defaultValue = "{}".format(val)
        if str(key).strip().lower() == "camera_name":
            defaultValue = "vzense_{}{}".format(val,camera_index)
        declareLaunchArgumentList.append(DeclareLaunchArgument("{}{}".format(key, camera_index), default_value=defaultValue, description= "{}".format(key)))
    return declareLaunchArgumentList

def set_configurable_parameters(parameters, camera_index):
    return dict([("{}".format(key), LaunchConfiguration("{}{}".format(key, camera_index))) for key,val in parameters.items()])

def launch_static_transform_publisher_node(context : LaunchContext):
    #print(context.launch_configurations)
    #static transformation from camera1 to camera2
    camera1_link = "{}_link".format(context.launch_configurations['camera_name1'])
    camera2_link = "{}_link".format(context.launch_configurations['camera_name2'])
    if camera2_link == camera1_link:
        print("replace camera2 link")
        camera2_link = "camera_name2_link"
    node = Node(
            name="static_tf_vzense_{}_{}".format(camera1_link, camera2_link),
            namespace = "",
            package = "tf2_ros",
            executable = "static_transform_publisher",
            arguments = ["0", "0", "0", "0", "0", "0",
                        camera1_link,
                        camera2_link]
    )
    return [node]

def launch_setup(context, camera_param_path, params, camera_index):
    return [
            Node(
            package='scepter_manager',
            namespace="",
            name=context.launch_configurations['camera_name{}'.format(camera_index)],
            executable='scepter_manager_node_exe',
            parameters=[camera_param_path, params],
            output="screen",
            arguments=["--ros-args", "--log-level", "info"],
            emulate_tty=True,
            )
    ]

def generate_launch_description():
    context = LaunchContext()
    camera_param_path_1 = os.path.join(
        FindPackageShare("scepter_manager").perform(context),
        "param/camera1.yaml"
    )

    camera_param_path_2 = os.path.join(
        FindPackageShare("scepter_manager").perform(context),
        "param/camera2.yaml"
    )
    
    with open(camera_param_path_1, "r") as f:
        camera_yaml_param_1 = yaml.safe_load(f)["/**"]["ros__parameters"]

    with open(camera_param_path_2, "r") as f:
        camera_yaml_param_2 = yaml.safe_load(f)["/**"]["ros__parameters"]

    # Define more yaml file and parse them then add more Nodes here if necessary
    return LaunchDescription(
        declare_configurable_parameters(camera_yaml_param_1, 1) + 
        declare_configurable_parameters(camera_yaml_param_2, 2) + 
        [
            OpaqueFunction(function=launch_setup,
                        kwargs = {  'camera_param_path' : camera_param_path_1,
                                    'params'           : set_configurable_parameters(camera_yaml_param_1, 1),
                                    'camera_index': 1}),
            OpaqueFunction(function=launch_setup,
                        kwargs = {  'camera_param_path' : camera_param_path_2,
                                    'params'           : set_configurable_parameters(camera_yaml_param_2, 2),
                                    'camera_index': 2}),
            OpaqueFunction(function=launch_static_transform_publisher_node)
        ]
    )