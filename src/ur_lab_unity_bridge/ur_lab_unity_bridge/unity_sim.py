#!/usr/bin/env python3
"""Unity simulation front end for ur_lab (Gazebo replacement).

Emulates the exact Gazebo API surface the unchanged ur_lab Python nodes use,
so ``scan_pick`` / ``pick_cube`` / ``move_robot`` run unmodified against Unity:

Topics (identical names + message types, as in Gazebo):
    /joint_states                        sensor_msgs/JointState
    /model_states, /gazebo/model_states  gazebo_msgs/ModelStates
    /gripper_controller/commands         std_msgs/Float64MultiArray (sub)
    /ur_lab/grasp_weld                   std_msgs/String (sub)
    /wrist_camera/image_raw              sensor_msgs/Image, rgb8 640x480, 5 Hz
    /unity/cube_poses                    std_msgs/String (extra visual-sync
                                         channel for the Unity CubeMirror only;
                                         NOT part of the Python API)

Action:
    /joint_trajectory_controller/follow_joint_trajectory
        control_msgs/FollowJointTrajectory (interpolated like ros2_control)

Services:
    /spawn_entity, /delete_entity        gazebo_msgs/srv/...
    /controller_manager/{list_controllers,load_controller,
        configure_controller,switch_controller}

Camera ownership: the Unity WristCameraPublisher owns /wrist_camera/image_raw
whenever the Unity scene is in play mode and connected through
ros_tcp_endpoint. This node checks for an existing external publisher at
startup and yields (Unity-first). Headless it publishes the synthetic 5 Hz
top-down view itself, generated from the same simulated state Unity mirrors,
so colour scanning behaves identically either way.

Physics: cubes rest on the Gazebo table top (z = 0.3 + 0.025 = 0.325);
the weld attaches a cube rigidly under the gripper (cube_z = tcp_z - 0.077,
same GRASP_TOOL_Z_OFFSET pick_cube.py uses); the lift moves vertically so a
rigid weld is exact.
"""

import json
import re
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile

from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.msg import ControllerState
from controller_manager_msgs.srv import (
    ConfigureController,
    ListControllers,
    LoadController,
    SwitchController,
)
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64MultiArray, Header, String

# FK helpers straight from the (unmodified) pick node: same arm, same math.
from ur_lab.pick_cube import FLIP, GRASP_TOOL_Z_OFFSET, HOME_Q, JOINTS, fk

ARM_JOINTS = list(JOINTS)
FINGER_JOINTS = ['finger_left_joint', 'finger_right_joint']
ALL_JOINTS = ARM_JOINTS + FINGER_JOINTS

TABLE_TOP_Z = 0.3
CUBE_REST_Z = TABLE_TOP_Z + 0.025

CAM_TOPIC = '/wrist_camera/image_raw'
CAM_W, CAM_H = 640, 480
CAM_HZ = 5.0  # identical to the Gazebo wrist camera rate

TABLE_RGB = (110, 75, 55)  # classifies as 'none' (brown table)
CUBE_RGB = {
    'red': (255, 0, 0),
    'blue': (0, 0, 255),
    'yellow': (255, 255, 0),
}
SNAP_RADIUS = 0.045  # TCP within this (x, y) distance => cube fills the view


def _color_of_sdf(xml, name):
    """Map the SDF material to a colour name, like the Gazebo visuals."""
    m = re.search(r'<diffuse>\s*([\d.]+)\s+([\d.]+)\s+([\d.]+)', xml or '')
    if not m:
        m = re.search(r'<ambient>\s*([\d.]+)\s+([\d.]+)\s+([\d.]+)', xml or '')
    if not m:
        return 'red' if 'red' in name else ('blue' if 'blue' in name else 'yellow')
    r, g, b = (float(m.group(1)) > 0.5, float(m.group(2)) > 0.5, float(m.group(3)) > 0.5)
    if r and not g and not b:
        return 'red'
    if b and not r and not g:
        return 'blue'
    if r and g and not b:
        return 'yellow'
    return 'red' if 'red' in name else ('blue' if 'blue' in name else 'yellow')


class UnitySim(Node):
    def __init__(self):
        super().__init__('unity_sim')
        self.declare_parameter('publish_camera', True)
        self._lock = threading.Lock()
        self._q = list(HOME_Q)
        self._fingers = [0.0, 0.0]
        self._cubes = {}   # name -> dict(x, y, z, color)
        self._welded = None
        cb = ReentrantCallbackGroup()

        self._js_pub = self.create_publisher(JointState, '/joint_states', 10)
        self._ms_pub = self.create_publisher(ModelStates, '/model_states', 10)
        self._ms2_pub = self.create_publisher(ModelStates, '/gazebo/model_states', 10)
        self._mirror_pub = self.create_publisher(String, '/unity/cube_poses', 10)

        self.create_subscription(Float64MultiArray, '/gripper_controller/commands',
                                 self._on_gripper, QoSProfile(depth=10), callback_group=cb)
        self.create_subscription(String, '/ur_lab/grasp_weld',
                                 self._on_weld, QoSProfile(depth=10), callback_group=cb)

        self.create_service(SpawnEntity, '/spawn_entity', self._on_spawn, callback_group=cb)
        self.create_service(DeleteEntity, '/delete_entity', self._on_delete, callback_group=cb)
        self.create_service(ListControllers, '/controller_manager/list_controllers',
                            self._on_list, callback_group=cb)
        self.create_service(LoadController, '/controller_manager/load_controller',
                            self._on_ok_load, callback_group=cb)
        self.create_service(ConfigureController, '/controller_manager/configure_controller',
                            self._on_ok_conf, callback_group=cb)
        self.create_service(SwitchController, '/controller_manager/switch_controller',
                            self._on_ok_switch, callback_group=cb)

        self._action = ActionServer(
            self, FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory',
            execute_callback=self._execute,
            goal_callback=lambda _: GoalResponse.ACCEPT,
            cancel_callback=lambda _: CancelResponse.ACCEPT,
            callback_group=cb)

        self.create_timer(0.02, self._pub_joints)    # 50 Hz like ros2_control
        self.create_timer(0.1, self._pub_models)     # 10 Hz like gazebo_ros_state
        self.create_timer(0.1, self._pub_mirror)

        # Camera ownership: Unity-first, bridge-fallback (see module docstring).
        self._cam_pub = None
        self._cam_timer = None
        want_cam = bool(self.get_parameter('publish_camera').value)
        if want_cam and not self._unity_camera_present():
            self._cam_pub = self.create_publisher(Image, CAM_TOPIC, 10)
            self._cam_timer = self.create_timer(1.0 / CAM_HZ, self._pub_camera)
            self.get_logger().info(f'Publishing {CAM_TOPIC} at {CAM_HZ} Hz (no Unity camera found)')
        elif want_cam:
            self.get_logger().info('Unity camera is live; bridge camera disabled (no double publish)')
        else:
            self.get_logger().info('Bridge camera disabled by parameter')

        self.get_logger().info('unity_sim ready (Gazebo API emulation for Unity)')

    # ---------- helpers ----------
    def _tcp(self):
        with self._lock:
            q = list(self._q)
        import numpy as np  # noqa: F401  (fk already pulls numpy)
        T = fk(q)
        p = FLIP @ T[:3, 3]
        return float(p[0]), float(p[1]), float(p[2])

    def _unity_camera_present(self, wait_s=3.0):
        """True if another node already publishes the camera (i.e. Unity)."""
        deadline = time.time() + wait_s
        while time.time() < deadline:
            try:
                infos = self.get_publishers_info_by_topic(CAM_TOPIC)
            except Exception:
                infos = []
            for info in infos:
                if info.node_name != self.get_name():
                    return True
            time.sleep(0.25)
        try:
            infos = self.get_publishers_info_by_topic(CAM_TOPIC)
        except Exception:
            return False
        return any(i.node_name != self.get_name() for i in infos)

    # ---------- subscriptions ----------
    def _on_gripper(self, msg):
        if len(msg.data) >= 2:
            with self._lock:
                self._fingers = [float(msg.data[0]), float(msg.data[1])]

    def _on_weld(self, msg):
        parts = (msg.data or '').split()
        if len(parts) != 2:
            return
        cmd, name = parts
        with self._lock:
            if cmd == 'attach':
                self._welded = name
                self.get_logger().info(f'{name} welded to gripper')
            elif cmd == 'detach' and self._welded == name:
                self._welded = None
                self.get_logger().info(f'{name} released')

    # ---------- services ----------
    def _on_spawn(self, req, resp):
        color = _color_of_sdf(req.xml, req.name)
        with self._lock:
            self._cubes[req.name] = {
                'x': float(req.initial_pose.position.x),
                'y': float(req.initial_pose.position.y),
                'z': CUBE_REST_Z,  # dropped cubes settle onto the table top
                'color': color,
            }
        resp.success = True
        resp.status_message = f'{req.name} spawned (unity_sim)'
        self.get_logger().info(f'{req.name} spawned at '
                               f"({req.initial_pose.position.x:.3f}, {req.initial_pose.position.y:.3f}) [{color}]")
        return resp

    def _on_delete(self, req, resp):
        with self._lock:
            resp.success = req.name in self._cubes
            self._cubes.pop(req.name, None)
            if self._welded == req.name:
                self._welded = None
        resp.status_message = 'ok'
        return resp

    def _on_list(self, req, resp):
        st = ControllerState()
        st.name = 'gripper_controller'
        st.state = 'active'
        st.type = 'position_controllers/JointGroupPositionController'
        st.claimed_interfaces = ['finger_left_joint/position', 'finger_right_joint/position']
        resp.controller = [st]
        return resp

    def _on_ok_load(self, req, resp):
        resp.ok = True
        return resp

    def _on_ok_conf(self, req, resp):
        resp.ok = True
        return resp

    def _on_ok_switch(self, req, resp):
        resp.ok = True
        return resp

    # ---------- action ----------
    def _execute(self, goal_handle):
        traj = goal_handle.request.trajectory
        names = list(traj.joint_names)
        with self._lock:
            cur = {j: self._q[ARM_JOINTS.index(j)] for j in ARM_JOINTS}
        prev_t = 0.0
        for pt in traj.points:
            t = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
            dur = max(0.1, t - prev_t)
            prev_t = t
            tgt = dict(cur)
            for n, p in zip(names, pt.positions):
                if n in tgt:
                    tgt[n] = float(p)
            steps = max(1, int(dur / 0.02))
            start = dict(cur)
            for s in range(1, steps + 1):
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    res = FollowJointTrajectory.Result()
                    res.error_code = res.SUCCESSFUL
                    return res
                a = s / steps
                with self._lock:
                    for j in ARM_JOINTS:
                        self._q[ARM_JOINTS.index(j)] = start[j] + (tgt[j] - start[j]) * a
                time.sleep(dur / steps)
            cur = tgt
            fb = FollowJointTrajectory.Feedback()
            goal_handle.publish_feedback(fb)
        goal_handle.succeed()
        res = FollowJointTrajectory.Result()
        res.error_code = res.SUCCESSFUL
        return res

    # ---------- publishers ----------
    def _pub_joints(self):
        with self._lock:
            q, f = list(self._q), list(self._fingers)
        msg = JointState()
        msg.header = Header()
        msg.header.frame_id = ''
        msg.name = ALL_JOINTS
        msg.position = [float(v) for v in (q + f)]
        msg.velocity = [0.0] * len(ALL_JOINTS)
        msg.effort = [0.0] * len(ALL_JOINTS)
        self._js_pub.publish(msg)

    def _pub_models(self):
        # Welded cube rides rigidly under the gripper (vertical lift => exact).
        with self._lock:
            if self._welded and self._welded in self._cubes:
                tx, ty, tz = self._tcp()
                c = self._cubes[self._welded]
                c['x'], c['y'], c['z'] = tx, ty, tz - GRASP_TOOL_Z_OFFSET
            names = list(self._cubes.keys())
            poses = []
            for n in names:
                c = self._cubes[n]
                p = Pose()
                p.position.x, p.position.y, p.position.z = c['x'], c['y'], c['z']
                p.orientation.w = 1.0
                poses.append(p)
        msg = ModelStates()
        msg.name = names
        msg.pose = poses
        msg.twist = []
        self._ms_pub.publish(msg)
        self._ms2_pub.publish(msg)

    def _pub_mirror(self):
        with self._lock:
            items = [{'name': n, 'color': c['color'],
                      'x': c['x'], 'y': c['y'], 'z': c['z']}
                     for n, c in self._cubes.items()]
        m = String()
        m.data = json.dumps(items)
        self._mirror_pub.publish(m)

    def _pub_camera(self):
        tx, ty, _ = self._tcp()
        with self._lock:
            best, best_d = None, float('inf')
            for c in self._cubes.values():
                d = abs(c['x'] - tx) + abs(c['y'] - ty)
                if d < best_d:
                    best, best_d = c, d
            rgb = CUBE_RGB.get(best['color'], TABLE_RGB) if (best and best_d < SNAP_RADIUS) else TABLE_RGB
        r, g, b = rgb
        row = bytes((r, g, b)) * CAM_W
        data = row * CAM_H
        msg = Image()
        msg.header = Header()
        msg.header.frame_id = 'wrist_camera_link'
        msg.height = CAM_H
        msg.width = CAM_W
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        msg.step = CAM_W * 3
        msg.data = data
        self._cam_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = UnitySim()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor(num_threads=4))
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
