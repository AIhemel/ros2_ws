import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument,OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def noisy_controller(context,*arg,**kwargs):
    
    wheel_radius = float(LaunchConfiguration("wheel_radius").perform(context))
    robot_width = float(LaunchConfiguration("robot_width").perform(context))
    robot_width_error = float(LaunchConfiguration("robot_width_error").perform(context))
    wheel_radius_error = float(LaunchConfiguration("wheel_radius_error").perform(context))
    robot_length = float(LaunchConfiguration("robot_length").perform(context))

    noisy_controller_py = Node(
    
        package="robot_controller",
        executable="noisy_controller.py",
        parameters=[
           {"wheel_radius":wheel_radius+wheel_radius_error,
            "robot_width":robot_width+robot_width_error,
            "robot_length":robot_length

           }
        ],
    )
    return[
        noisy_controller_py
    ]

    
   


def generate_launch_description():
    # Declare launch arguments
    use_python_arg = DeclareLaunchArgument(
        "use_python",
        default_value="True",
        description="Set to True to use Python-based controller"
    )
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033",
        description="Wheel radius in meters"
    )
    robot_length_arg = DeclareLaunchArgument(
        "robot_length",
        default_value="0.38",
        description="Robot length (half of total length L)"
    )
    robot_width_arg = DeclareLaunchArgument(
        "robot_width",
        default_value="0.30",
        description="Robot width (half of total width W)"
    )
    wheel_radius_error_arg = DeclareLaunchArgument(
        "wheel_radius_error",
        default_value="0.005",
        description="Wheel radius error in meters"
    )
    robot_width_error_arg = DeclareLaunchArgument(
        "robot_width_error",
        default_value="0.02",
        description="Robot error in width"
    )

    # Launch configurations
    use_python = LaunchConfiguration("use_python")
    wheel_radius = LaunchConfiguration("wheel_radius")
    robot_length = LaunchConfiguration("robot_length")
    robot_width = LaunchConfiguration("robot_width")

    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    # Python Controller Node
    simple_controller_py = Node(
        package="robot_controller",
        executable="simple_controller.py",
        parameters=[{
            "wheel_radius": wheel_radius,
            "robot_length": robot_length,  # ✅ Fixed naming
            "robot_width": robot_width  # ✅ Fixed naming
        }],
        condition=IfCondition(use_python)  # ✅ Ensure `IfCondition` evaluates correctly
    )

    # Controller Spawner for Simple Velocity Controller
    simple_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["simple_velocity_controller",
                   "--controller-manager",
                   "/controller_manager"
                   ]
    )
    noisy_controller_launch=    OpaqueFunction(function=noisy_controller)

    return LaunchDescription([
        use_python_arg,
        wheel_radius_arg,
        robot_length_arg,  # ✅ Fixed naming
        robot_width_arg,  # ✅ Fixed naming
        robot_width_error_arg,
        wheel_radius_error_arg,
        joint_state_broadcaster_spawner,
        simple_controller,
        simple_controller_py,
        noisy_controller_launch, 
    ])