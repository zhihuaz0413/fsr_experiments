import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    # Define xacro mappings for the robot description file
    launch_arguments = {
        "robot_ip": "192.168.1.10",
        "use_fake_hardware": "false",
        "gripper": "",
        "dof": "7",
    }

    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(
            "gen3", package_name="kinova_gen3_7dof_robotiq_2f_85_moveit_config"
        )
        .robot_description(mappings=launch_arguments)
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines("ompl", ["ompl", "chomp", "pilz_industrial_motion_planner"])
        .moveit_cpp(
            file_path=os.path.join(
                get_package_share_directory("fsr_experiments"),
                "config",
                "fsr_experiments.yaml",
            )
        )
        .to_moveit_configs()
    )
    # Load additional OMPL pipeline
    ompl_planning_pipeline_config = {
        "ompl_2": {
            "planning_plugins": [
                "ompl_interface/OMPLPlanner",
            ],
            "request_adapters": [
                "default_planning_request_adapters/ResolveConstraintFrames",
                "default_planning_request_adapters/ValidateWorkspaceBounds",
                "default_planning_request_adapters/CheckStartStateBounds",
                "default_planning_request_adapters/CheckStartStateCollision",
            ],
            "response_adapters": [
                "default_planning_response_adapters/AddTimeOptimalParameterization",
                "default_planning_response_adapters/ValidateSolution",
                "default_planning_response_adapters/DisplayMotionPath",
            ],
        }
    }
    ompl_planning_yaml = load_yaml(
        "kinova_gen3_7dof_robotiq_2f_85_moveit_config", "config/ompl_planning.yaml"
    )
    ompl_planning_pipeline_config["ompl_2"].update(ompl_planning_yaml)

    # MoveItCpp demo executable
    fsr_experiments_node = Node(
        name="fsr_experiments",
        package="fsr_experiments",
        executable="fsr_experiments_cpp",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            ompl_planning_pipeline_config,
        ],
    )

    # RViz
    rviz_config_file = os.path.join(
        get_package_share_directory("fsr_experiments"),
        "launch",
        "fsr_experiments.rviz",
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("kinova_gen3_7dof_robotiq_2f_85_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[ros2_controllers_path],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_trajectory_controller", "-c", "/controller_manager"],
    )

    twist_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["twist_controller", "--inactive", "-c", "/controller_manager"],
    )

    return LaunchDescription(
        [
            static_tf,
            robot_state_publisher,
            rviz_node,
            fsr_experiments_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            twist_controller_spawner,
        ]
    )
