import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    moveit_config = (
        MoveItConfigsBuilder("igus_rebel", package_name="igus_rebel_moveit_config")
        .robot_description(file_path=os.path.join(
            get_package_share_directory("igus_rebel_description"),
            "urdf",
            "igus_rebel_robot2.urdf.xacro"
        ))
        .robot_description_semantic(file_path="config/igus_rebel2.srdf")
        # .planning_pipelines(pipelines=["ompl"])
        .joint_limits(file_path="config/joint_limits.yaml")
        .pilz_cartesian_limits(file_path="config/pilz_cartesian_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory("igus_rebel_moveit_config")
            + "/config/planning.yaml"
        )
        .to_moveit_configs()
    )


    api_server = Node(
        name="moveit_py",
        package="igus_rebel_api",
        executable="moveit_http_server",
        output="both",
        parameters=[moveit_config.to_dict(), {"use_sim_time": True}]
    )

    return LaunchDescription([
        api_server
    ])
