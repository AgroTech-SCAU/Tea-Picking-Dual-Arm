import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def include(package, launch_file, condition=None, launch_arguments=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(package), "launch", launch_file)
        ),
        condition=condition,
        launch_arguments=(launch_arguments or {}).items(),
    )


def generate_launch_description():
    arm_type = LaunchConfiguration("arm_type")
    allow_execution = LaunchConfiguration("allow_trajectory_execution")
    is_rm65 = IfCondition(PythonExpression(["'", arm_type, "' == 'rm65'"]))
    is_rm75 = IfCondition(PythonExpression(["'", arm_type, "' == 'rm75'"]))

    return LaunchDescription([
        DeclareLaunchArgument(
            "arm_type",
            default_value="rm65",
            description="Robot model: rm65 or rm75",
        ),
        DeclareLaunchArgument(
            "allow_trajectory_execution",
            default_value="false",
            description="Keep false for status/RViz/planning-only validation",
        ),
        include("rm_driver", "rm_65_driver.launch.py", is_rm65),
        include("rm_description", "rm_65_display.launch.py", is_rm65),
        include("rm_control", "rm_65_control.launch.py", is_rm65),
        include(
            "rm_65_config",
            "real_moveit_demo.launch.py",
            is_rm65,
            {"allow_trajectory_execution": allow_execution},
        ),
        include("rm_driver", "rm_75_driver.launch.py", is_rm75),
        include("rm_description", "rm_75_display.launch.py", is_rm75),
        include("rm_control", "rm_75_control.launch.py", is_rm75),
        include(
            "rm_75_config",
            "real_moveit_demo.launch.py",
            is_rm75,
            {"allow_trajectory_execution": allow_execution},
        ),
    ])
