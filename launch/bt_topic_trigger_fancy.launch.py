# This Python file uses the following encoding: utf-8
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, SetEnvironmentVariable
from launch.substitutions import FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
  config_folder = PathJoinSubstitution([FindPackageShare("holden_demo"),"config"])

  return LaunchDescription([

    SetEnvironmentVariable(name="IK_SOLVER_LOGGER_CONFIG_PATH", value=PathJoinSubstitution([config_folder,"logger_param.yaml"])),

    ExecuteProcess(
        cmd = [
          FindExecutable(name="cnr_param_server"),
          "--path-to-file",
          PathJoinSubstitution([
            config_folder,
            "bt_topic_trigger_fancy_config.yaml"  
          ]),
          "--path-to-file",
          PathJoinSubstitution([
            config_folder,
            "bt_topic_trigger_skills_fancy_config.yaml" 
          ])
        ],
        shell=False
      ),

    TimerAction(
      period=0.1,  # delay in seconds
      actions=[
        Node(
          package="btcpp_ros2_samples",
          executable="sleep_server")
      ]
    ),

    TimerAction(
      period=1.0,  # delay in seconds
      actions=[
        Node(
          package="moveit_object_attacher",
          executable="object_attacher_node",
          output="screen",
          namespace="",
          ros_arguments=["--log-level", "info"],
          parameters=[{"group_name": "ur_on_linear_guide"}]
         )
      ]
    ),
    
    #Move to and sleep server node
    TimerAction(
      period=1.0,  # delay in seconds
      actions=[
        Node(
          package="trajectory_loader",
          executable="move_to_conf_server",
          output="screen",
          namespace="",
          ros_arguments=["--log-level", "info"]
         ),
        Node(
          package="btcpp_ros2_samples",
          executable="sleep_server"
        )
      ]
    ),

    #Bt executer
    TimerAction(
      period=1.0,  # delay in seconds
      actions=[
        Node(
          package="bt_topic_trigger",
          executable="bt_topic_trigger_node",
          output="screen",
          namespace="bt_topic_trigger",
          ros_arguments=["--log-level", "info"]
        )
      ]
    )
])
