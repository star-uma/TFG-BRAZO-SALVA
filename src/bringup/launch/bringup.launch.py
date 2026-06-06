import os
import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch_param_builder import ParameterBuilder

def generate_launch_description():

    
    #===========================================================
    # CONFIG
    # ==========================================================

    urdf_path = os.path.join(
        get_package_share_directory("bringup"),
        "config",
        "manipulador.urdf.xacro",
    )
    robot_description_content = ParameterValue(
        Command(f'xacro "{urdf_path}"'),
        value_type=str
    )

    robot_description = {"robot_description": robot_description_content}


    #===========================================================
    # TF
    # ==========================================================

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

   
    #===========================================================
    # ROS2 CONTROL
    # ==========================================================

    ros2_controllers_path = os.path.join(
        get_package_share_directory("bringup"),
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
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster","-c","/controller_manager"],
    )

    brazo_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["brazo_controller", "-c", "/controller_manager"],
    )

   
    #====================================================================
    # JOYSTICK + TELEOP
    #====================================================================

    joystick_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[{
            "dev":               "/dev/input/js0",
            "deadzone":          0.1,
            "autorepeat_rate":   25.0,
            "coalesce_interval": 0.001,
        }],
        output="screen",
    )

    nodo_teleop = Node(
        package="teleop",
        executable="controlador",
        name="nodo_teleop",
        output="screen",
        parameters=[robot_description],
    )

    #===================================================================
    # LAUNCH DESCRIPTION
    #
    # Orden de arranque:
    #   1) TF + robot_state_publisher  (infraestructura)
    #   2) ros2_control_node           (hardware interface)
    #   3) controllers spawners        (joint_state_broadcaster + monomotor)
    #   4) joy + teleop 

    #====================================================================

    return LaunchDescription([
        

        # Infraestructura TF
        static_tf_node,
        robot_state_publisher,

        # Hardware
        ros2_control_node,
        joint_state_broadcaster_spawner,
        brazo_controller_spawner,
        
        # Teleop — espera 10 s a que servo_node esté listo
        TimerAction(
            period=10.0,
            actions=[
                joystick_node,
                nodo_teleop,
            ],
        ),
      
    ])