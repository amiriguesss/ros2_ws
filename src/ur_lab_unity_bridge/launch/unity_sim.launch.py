"""Unity simulation front end (replaces ``sim.launch.py`` / Gazebo).

Starts the ROS-TCP-Endpoint bridge Unity connects to, plus the unity_sim
node that emulates the Gazebo API (spawn/delete, model_states,
joint trajectory action, joint_states, gripper, grasp weld, 5 Hz wrist
camera, controller_manager stubs) with identical topic names and types, so
every existing ur_lab Python node runs unchanged.

Unity side: open ~/unity_ws/UR5eUnity, load Assets/Scenes/Lab.unity, press
Play with ROSConnection at 127.0.0.1:10000. The Unity wrist camera then owns
/wrist_camera/image_raw and the bridge camera yields automatically.
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    endpoint = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='unity_endpoint',
        parameters=[{'ROS_IP': '127.0.0.1', 'ROS_TCP_PORT': 10000}],
        output='screen',
    )
    sim = Node(
        package='ur_lab_unity_bridge',
        executable='unity_sim',
        name='unity_sim',
        output='screen',
    )
    return LaunchDescription([endpoint, sim])
