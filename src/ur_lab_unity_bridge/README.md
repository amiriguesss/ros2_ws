# ur_lab Unity front end (Gazebo replacement)

The existing `ur_lab` Python nodes are **untouched** — Unity replaces only the
Gazebo renderer/physics side, connected through the ROS TCP Connector bridge
with identical topic names and message types.

## ROS side

- `ros_tcp_endpoint` (cloned from Unity-Technologies/ROS-TCP-Endpoint,
  `main-ros2` branch): TCP bridge Unity connects to (`127.0.0.1:10000`).
- `ur_lab_unity_bridge/unity_sim`: emulates the Gazebo API the Python nodes
  use — `/spawn_entity`, `/delete_entity`, `/model_states`
  (+ `/gazebo/model_states`), `/joint_trajectory_controller/...` action,
  `/joint_states`, `/gripper_controller/commands`, `/ur_lab/grasp_weld`,
  `controller_manager/*`, and the 5 Hz `/wrist_camera/image_raw` camera
  (Unity-first: the bridge camera yields automatically when the Unity scene
  is in play mode). Plus `/unity/cube_poses` visual sync for Unity.

Launch it (terminal 1):

```bash
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 launch ur_lab_unity_bridge unity_sim.launch.py
```

Then run the unchanged demo (terminal 2):

```bash
export ROS_LOCALHOST_ONLY=1
source /opt/ros/humble/setup.zsh
source install/setup.zsh
ros2 run ur_lab scan_pick
```

> NOTE (WSL): this machine's virtual network blocks DDS multicast discovery,
> so **every** ROS command in every terminal needs `ROS_LOCALHOST_ONLY=1`
> (including terminal 1). Native Ubuntu with working multicast does not need it.

## Unity side (`~/unity_ws/UR5eUnity`)

- `Packages/manifest.json`: `com.unity.robotics.ros-tcp-connector#v0.7.0`,
  `com.unity.robotics.urdf-importer#v0.5.2`.
- `Assets/Urdf/ur5e_gripper.urdf`: same arm+Gripper+camera description Gazebo
  uses (resolved from `src/ur_lab/urdf/ur5e_gripper.urdf.xacro`), imported
  with the URDF Importer (17 articulation bodies, model name `ur`).
- `Assets/Scenes/Lab.unity` (built by `Assets/Editor/LabBuilder.cs`):
  table `0.8 x 1.2 x 0.6 m` at `(0.8, 0, 0.0)`, three `0.05 m` cubes
  (red/blue/yellow) on the scan line, high-quality lighting (soft shadowed
  key light, raised ambient, high shadow maps), and a 640x480 wrist camera
  publishing `/wrist_camera/image_raw` at exactly 5 Hz.
- `Assets/Scripts/`: `WristCameraPublisher.cs` (5 Hz `sensor_msgs/Image`),
  `ArmMirror.cs` (`/joint_states` visual tracking), `CubeMirror.cs`
  (cube visual sync).
