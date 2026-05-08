import os

from ament_index_python.packages import get_package_share_directory
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
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
    #static transformation from camera1 to camera2
    camera1_link = "{}_link".format(context.launch_configurations['camera_name1'])
    camera2_link = "{}_link".format(context.launch_configurations['camera_name2'])
    if camera2_link == camera1_link:
        #print("replace camera2 link")
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

def ComposableNode_setup(node_name, camera_param_path, params):
    return ComposableNode(
                package="scepter_manager",
                plugin="ScepterManager",
                name= node_name,
                namespace='',
                parameters=[camera_param_path, params],
                remappings=[
                ],
                extra_arguments=[
                    {"use_intra_process_comms": True}
                ],
            )

def launch_setup(context, camera_param_path, params_list, camera_list):
    composable_node_descriptions_list = []
    for ind in camera_list:
        composable_node_descriptions_list.append(ComposableNode_setup(context.launch_configurations['camera_name{}'.format(ind)], camera_param_path[ind - 1], params_list[ind - 1]))
    # Add your own ComposableNode here if you want to subscribe scepter_manager package's topic through intra prcocess comms way.
    container = ComposableNodeContainer(
        name="vzense_camera_components",
        namespace='',
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions = composable_node_descriptions_list,
        emulate_tty=True, # needed for display of logs
        output="screen",# or both
        arguments=["--ros-args", "--log-level", "info"]
    )
    return [container]

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
            OpaqueFunction(function=launch_static_transform_publisher_node),
            OpaqueFunction(function=launch_setup,
            kwargs = {  'camera_param_path' : [camera_param_path_1, camera_yaml_param_2],
                        'params_list'           : [set_configurable_parameters(camera_yaml_param_1, 1), set_configurable_parameters(camera_yaml_param_2, 2)],
                        'camera_list': [1,2]}),
        ]
    )
