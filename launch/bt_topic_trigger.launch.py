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
            "bt_topic_trigger_config.yaml"  
          ]),
          "--path-to-file",
          PathJoinSubstitution([
            config_folder,
            "bt_topic_trigger_skills_config.yaml" 
          ])
        ],
        shell=False
      ),

    # # Poses to reach  
    # TimerAction(
    #   period=0.0,  # delay in seconds
    #   actions=[
    #      Node(
    #        package="tf2_ros",
    #        executable="static_transform_publisher",
    #        name="pick_pose_broadcaster",
    #        arguments=["-0.126", "-0.390", "0.221", "0.918", "-0.021", "-0.021", "0.395","world","pick_pose"],
    #        output="screen")
    #   ]
    # ),

    # TimerAction(
    #   period=0.0,  # delay in seconds
    #   actions=[
    #      Node(
    #        package="tf2_ros",
    #        executable="static_transform_publisher",
    #        name="place_pose_broadcaster",
    #        arguments=["0.476", "-0.439", "0.397", "0.980", "0.126", "-0.020", "0.151","world","place_pose"],
    #        output="screen")
    #   ]
    # ),

    # # IK solver
    # TimerAction(
    #   period=1.0,  # delay in seconds
    #   actions=[
    #   IncludeLaunchDescription(
    #     launch_description_source=PythonLaunchDescriptionSource(
    #       PathJoinSubstitution([FindPackageShare('ik_solver'),"launch","ik_solver.launch.py"])),
    #       launch_arguments={'config': PathJoinSubstitution([config_folder,"ik_solver_config.yaml"])}.items()
    #      )
    #   ]
    # ),

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
        #  Node(
        #   package="trajectory_loader",
        #   executable="move_to_server",
        #   output="screen",
        #   namespace="",
        #   ros_arguments=["--log-level", "debug"]
        #  ),
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
