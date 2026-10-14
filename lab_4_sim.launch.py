# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, ThisLaunchFileDir

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():

    # Get URDF via xacro
    robot_description_content = ParameterValue(Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            "pupper_sim.xacro"
        ]
    ), value_type=str)
    
    robot_description = {"robot_description": robot_description_content}

    sim_parameters = "lab_4_sim.yaml"

    pkg_share = FindPackageShare("mujoco_ros2_control_demos")
    mujoco_plugins_file = PathJoinSubstitution([pkg_share, "config", "mujoco_ros2_control_plugins.yaml"])
    
    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare("ros2_control_demo_example_1"), "rviz", "rrbot.rviz"]
    # )

    control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        output="both",
        parameters=[
            
            {"use_sim_time": True},
            robot_description,
            mujoco_plugins_file
        ],
    )
    
    # control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[robot_description, robot_controllers],
    #     output="both",
    # )
    
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {"use_sim_time": True}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--param-file", sim_parameters ],
    )

    # imu_sensor_broadcaster_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["imu_sensor_broadcaster", "--controller-manager", "/controller_manager", "--controller-manager-timeout", "30"],
    # )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_command_controller", "--param-file", sim_parameters ]
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        )
    )

    nodes = [
        robot_state_pub_node,
        control_node,
        #robot_state_pub_node,
        joint_state_broadcaster_spawner,
        #imu_sensor_broadcaster_spawner,
        #delay_robot_controller_spawner_after_joint_state_broadcaster_spawner,
        robot_controller_spawner,
    ]

    return LaunchDescription(nodes)
