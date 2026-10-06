"""MoveIt 2 skills with a Gazebo suction gripper and camera-fed object poses."""

import copy
import threading
import time

from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import Pose
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    AllowedCollisionEntry,
    AttachedCollisionObject,
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    MoveItErrorCodes,
    OrientationConstraint,
    PlanningScene,
    PlanningSceneComponents,
    PositionConstraint,
    RobotState,
)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from rclpy.action import ActionClient
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, Empty
from trajectory_msgs.msg import JointTrajectoryPoint

from .scene_state import build_world_state


class RobotSkills:
    SUCCESS = "SUCCESS"
    PLANNING_FAILED = "PLANNING_FAILED"
    EXECUTION_FAILED = "EXECUTION_FAILED"
    FAILURE = "FAILED"
    INVALID_OBJECT = "INVALID_OBJECT"
    INVALID_ZONE = "INVALID_ZONE"

    def __init__(self, node, scene, require_camera=False):
        self.node = node
        self.scene = scene
        self.require_camera = bool(require_camera)
        self.frame = scene["world_frame"]
        self.group = scene["planning_group"]
        self.eef = scene["eef_link"]
        self.orientation = scene.get("tool_orientation_xyzw", [1.0, 0.0, 0.0, 0.0])
        self.objects = copy.deepcopy(scene["objects"])
        self.zones = scene["zones"]
        self.table = scene["table"]
        self.temporary_slots = scene.get("temporary_slots", {})
        self.gripper = scene.get("gripper", {})
        self._held = None
        self._lock = threading.Lock()
        self._state_condition = threading.Condition()
        self._world_state = None
        self._gripper_detached = {}
        self.move_client = ActionClient(node, MoveGroup, scene.get("move_action", "/move_action"))
        self.execute_client = ActionClient(
            node, ExecuteTrajectory, scene.get("execute_action", "/execute_trajectory")
        )
        self.scene_client = node.create_client(
            ApplyPlanningScene, scene.get("apply_scene_service", "/apply_planning_scene")
        )
        self.get_scene_client = node.create_client(
            GetPlanningScene, scene.get("get_scene_service", "/get_planning_scene")
        )
        self.attach_publishers = {}
        self.detach_publishers = {}
        self._gripper_subscriptions = []
        self.gripper_client = None
        backend = self.gripper.get("backend", "gazebo_detachable")
        if backend == "gazebo_detachable":
            for object_id in self.objects:
                self.attach_publishers[object_id] = node.create_publisher(
                    Empty, "/gripper/attach/%s" % object_id, 10
                )
                self.detach_publishers[object_id] = node.create_publisher(
                    Empty, "/gripper/detach/%s" % object_id, 10
                )
                self._gripper_subscriptions.append(node.create_subscription(
                    Bool,
                    "/gripper/state/%s" % object_id,
                    lambda message, key=object_id: self._on_gripper_state(key, message),
                    10,
                ))
        elif backend == "follow_joint_trajectory":
            self.gripper_client = ActionClient(
                node,
                FollowJointTrajectory,
                self.gripper.get("action", "/gripper_controller/follow_joint_trajectory"),
            )

    @property
    def current_world_state(self):
        with self._state_condition:
            return copy.deepcopy(self._world_state)

    def update_world_state(self, state):
        with self._state_condition:
            self._world_state = copy.deepcopy(state)
            for object_id, item in state.get("objects", {}).items():
                if (
                    object_id in self.objects
                    and item.get("visible", False)
                    and item.get("position_xyz") is not None
                ):
                    self.objects[object_id]["position_xyz"] = [
                        float(value) for value in item["position_xyz"]
                    ]
            self._state_condition.notify_all()

    def initialize(self, timeout_sec=2.0):
        for client, label in (
            (self.move_client, "MoveGroup action"),
            (self.execute_client, "ExecuteTrajectory action"),
        ):
            if not client.wait_for_server(timeout_sec=timeout_sec):
                return False, "%s is unavailable." % label
        if not self.scene_client.wait_for_service(timeout_sec=timeout_sec):
            return False, "ApplyPlanningScene service is unavailable."
        if not self.get_scene_client.wait_for_service(timeout_sec=timeout_sec):
            return False, "GetPlanningScene service is unavailable."

        if self.require_camera:
            deadline = time.monotonic() + 30.0
            with self._state_condition:
                while time.monotonic() < deadline:
                    state = self._world_state
                    if state is not None and all(
                        state.get("objects", {}).get(object_id, {}).get("visible", False)
                        for object_id in self.objects
                    ):
                        break
                    self._state_condition.wait(timeout=0.2)
                else:
                    return False, "Camera has not detected every configured block."
        elif self._world_state is None:
            self.update_world_state(build_world_state(self.scene))

        if not self.sync_scene_from_camera(self.current_world_state):
            return False, "MoveIt rejected the camera/configured object scene."
        if self.table.get("pedestal_center_xyz") is not None:
            for link_name in ("base_link", "base_link_inertia"):
                if not self._set_collision_allowed("robot_pedestal", link_name, True):
                    return False, "Could not register the UR base-to-pedestal contact."

        if self.gripper.get("backend", "gazebo_detachable") == "gazebo_detachable":
            for object_id in self.objects:
                if not self._set_gazebo_attachment(object_id, attached=False, timeout=8.0):
                    return False, "Gazebo gripper did not release %s during initialization." % object_id
        elif self.gripper_client is not None:
            if not self.gripper_client.wait_for_server(timeout_sec=timeout_sec):
                return False, "Configured gripper action server is unavailable."
            if not self._trajectory_gripper(opening=True):
                return False, "Could not open the configured gripper."
        return True, "Ready."

    def sync_scene_from_camera(self, state):
        if not isinstance(state, dict):
            return False
        with self._lock:
            for object_id, item in state.get("objects", {}).items():
                if (
                    object_id in self.objects
                    and item.get("visible", False)
                    and item.get("position_xyz") is not None
                    and object_id != self._held
                ):
                    self.objects[object_id]["position_xyz"] = [
                        float(value) for value in item["position_xyz"]
                    ]
            return self._apply_world_scene()

    def pick(self, object_id):
        with self._lock:
            if object_id not in self.objects:
                return self.INVALID_OBJECT, "Object %r is not in the scene catalog." % object_id
            if self._held is not None:
                return self.FAILURE, "Already holding %s." % self._held
            if self.require_camera:
                state = self.current_world_state
                detected = (state or {}).get("objects", {}).get(object_id, {})
                if not detected.get("visible", False):
                    return self.FAILURE, "Camera cannot currently locate %s." % object_id
                if detected.get("position_xyz") is None:
                    return self.FAILURE, "Camera has no position for %s." % object_id
                self.objects[object_id]["position_xyz"] = [
                    float(value) for value in detected["position_xyz"]
                ]

            obj = self.objects[object_id]
            if not self._operate_gripper(object_id, opening=True):
                return self.FAILURE, "Could not open/release the gripper."
            tcp_pose = self._object_tcp_pose(obj)
            approach = self._offset_pose(
                tcp_pose, float(self.scene.get("approach_height_m", 0.10))
            )
            status, reason = self._move_pose(approach)
            if status != self.SUCCESS:
                return status, "Could not reach the approach pose: " + reason
            if not self._remove_world_object(object_id):
                return self.FAILURE, "Could not allow target contact in the MoveIt scene."
            status, reason = self._move_pose(tcp_pose)
            if status != self.SUCCESS:
                self._add_world_object(object_id, obj)
                return status, "Could not reach the grasp pose: " + reason
            if not self._operate_gripper(object_id, opening=False):
                self._add_world_object(object_id, obj)
                return self.FAILURE, "Gazebo gripper failed to attach the block."
            if not self._attach_object(object_id, obj):
                self._operate_gripper(object_id, opening=True)
                self._add_world_object(object_id, obj)
                return self.FAILURE, "Could not attach the block in the MoveIt scene."
            self._held = object_id
            if not self._set_collision_allowed(object_id, "work_table", True):
                return self.FAILURE, "Could not allow table contact during the initial lift."
            status, reason = self._move_pose(approach)
            if status != self.SUCCESS:
                return status, "Object is grasped, but retreat failed: " + reason
            if not self._set_collision_allowed(object_id, "work_table", False):
                return self.FAILURE, "Could not restore table collision checking after the lift."
            return self.SUCCESS, "Block attached by the Gazebo gripper."

    def place(self, object_id, zone_id):
        if zone_id not in self.zones:
            return self.INVALID_ZONE, "Zone %r is not in the scene catalog." % zone_id
        surface = self.zones[zone_id].get(
            "surface_xyz", self.zones[zone_id].get("center_xyz")
        )
        return self._place_at(object_id, surface, "zone %s" % zone_id)

    def place_temp(self, object_id, slot_id):
        if slot_id not in self.temporary_slots:
            return self.INVALID_ZONE, "Temporary slot %r is not configured." % slot_id
        surface = self.temporary_slots[slot_id]["surface_xyz"]
        return self._place_at(object_id, surface, "temporary slot %s" % slot_id)

    def _place_at(self, object_id, surface, label):
        with self._lock:
            if object_id not in self.objects:
                return self.INVALID_OBJECT, "Object %r is not in the scene catalog." % object_id
            if self._held != object_id:
                return self.FAILURE, "Robot is not holding %s." % object_id
            obj = self.objects[object_id]
            dimensions = obj["size_xyz"]
            target = Pose()
            target.position.x = float(surface[0])
            target.position.y = float(surface[1])
            target.position.z = (
                float(surface[2])
                + float(dimensions[2])
                + float(self.scene.get("place_clearance_m", 0.002))
            )
            target.orientation.x, target.orientation.y, target.orientation.z, target.orientation.w = (
                self.orientation
            )
            approach = self._offset_pose(
                target, float(self.scene.get("approach_height_m", 0.10))
            )
            status, reason = self._move_pose(approach)
            if status != self.SUCCESS:
                return status, "Could not reach the destination approach pose: " + reason
            status, reason = self._move_pose(target)
            if status != self.SUCCESS:
                return status, "Could not reach the destination pose: " + reason
            if not self._operate_gripper(object_id, opening=True):
                return self.FAILURE, "Gazebo gripper failed to release the block."

            state_before = self.current_world_state or {}
            sequence_before = int(state_before.get("sequence", 0))
            placed_obj = copy.deepcopy(obj)
            placed_obj["position_xyz"] = [
                target.position.x,
                target.position.y,
                float(surface[2]) + float(dimensions[2]) / 2.0,
            ]
            if not self._detach_and_add_world(object_id, placed_obj):
                return self.FAILURE, "Could not update the MoveIt scene after release."
            self.objects[object_id] = placed_obj
            self._held = None
            status, reason = self._move_pose(approach)
            if status != self.SUCCESS:
                return status, "Object was released, but retreat failed: " + reason
            if self.require_camera and not self._wait_for_object_at(
                object_id,
                placed_obj["position_xyz"],
                sequence_before,
                timeout_sec=8.0,
            ):
                return self.FAILURE, "Camera did not confirm the released block at %s." % label
            self._refresh_static_state(placed_obj)
            return self.SUCCESS, "Object placed at %s." % label

    def _refresh_static_state(self, placed_obj):
        if self.require_camera:
            return
        positions = {
            object_id: item["position_xyz"] for object_id, item in self.objects.items()
        }
        self.update_world_state(build_world_state(self.scene, positions=positions))

    def _wait_for_object_at(self, object_id, expected_xyz, sequence_before, timeout_sec):
        deadline = time.monotonic() + timeout_sec
        with self._state_condition:
            while time.monotonic() < deadline:
                state = self._world_state or {}
                item = state.get("objects", {}).get(object_id, {})
                sequence = int(state.get("sequence", 0))
                if sequence > sequence_before and item.get("visible", False):
                    actual = item.get("position_xyz")
                    if actual is not None:
                        distance = sum(
                            (float(actual[i]) - float(expected_xyz[i])) ** 2
                            for i in range(2)
                        ) ** 0.5
                        if distance <= 0.08:
                            return True
                self._state_condition.wait(timeout=0.2)
        return False

    def home(self):
        with self._lock:
            if self._held is not None:
                return self.FAILURE, "Refusing home() while an object is held."
            positions = self.scene.get("home_joint_positions", {})
            if not positions:
                return self.FAILURE, "home_joint_positions is missing from scene config."
            constraints = Constraints()
            for name, value in positions.items():
                joint = JointConstraint()
                joint.joint_name = name
                joint.position = float(value)
                tolerance = float(self.scene.get("home_joint_tolerance_rad", 0.04))
                joint.tolerance_above = tolerance
                joint.tolerance_below = tolerance
                joint.weight = 1.0
                constraints.joint_constraints.append(joint)
            return self._move_constraints(constraints)

    def _move_pose(self, pose):
        constraints = Constraints()
        position = PositionConstraint()
        position.header.frame_id = self.frame
        position.link_name = self.eef
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.005]
        position.constraint_region.primitives.append(sphere)
        position.constraint_region.primitive_poses.append(copy.deepcopy(pose))
        position.weight = 1.0
        orientation = OrientationConstraint()
        orientation.header.frame_id = self.frame
        orientation.link_name = self.eef
        orientation.orientation = copy.deepcopy(pose.orientation)
        tolerance = float(self.scene.get("orientation_tolerance_rad", 0.12))
        orientation.absolute_x_axis_tolerance = tolerance
        orientation.absolute_y_axis_tolerance = tolerance
        orientation.absolute_z_axis_tolerance = tolerance
        orientation.weight = 1.0
        constraints.position_constraints.append(position)
        constraints.orientation_constraints.append(orientation)
        return self._move_constraints(constraints)

    def _move_constraints(self, constraints):
        request = MotionPlanRequest()
        request.group_name = self.group
        request.pipeline_id = self.scene.get("planning_pipeline", "")
        request.planner_id = self.scene.get("planner_id", "")
        request.num_planning_attempts = int(self.scene.get("planning_attempts", 8))
        request.allowed_planning_time = float(self.scene.get("planning_time_sec", 5.0))
        request.max_velocity_scaling_factor = float(self.scene.get("velocity_scaling", 0.08))
        request.max_acceleration_scaling_factor = float(
            self.scene.get("acceleration_scaling", 0.08)
        )
        request.start_state = RobotState()
        request.start_state.is_diff = True
        request.goal_constraints = [constraints]
        goal = MoveGroup.Goal()
        goal.request = request
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        planned = self._send_action(self.move_client, goal)
        if planned is None or planned.status != GoalStatus.STATUS_SUCCEEDED:
            return self.PLANNING_FAILED, "MoveGroup action failed while planning."
        result = planned.result
        if result.error_code.val != MoveItErrorCodes.SUCCESS:
            return self.PLANNING_FAILED, "MoveIt planning error %d." % result.error_code.val
        execute_goal = ExecuteTrajectory.Goal()
        execute_goal.trajectory = result.planned_trajectory
        executed = self._send_action(self.execute_client, execute_goal, timeout=180.0)
        if executed is None or executed.status != GoalStatus.STATUS_SUCCEEDED:
            return self.EXECUTION_FAILED, "ExecuteTrajectory action failed."
        if executed.result.error_code.val != MoveItErrorCodes.SUCCESS:
            return self.EXECUTION_FAILED, "MoveIt execution error %d." % executed.result.error_code.val
        return self.SUCCESS, "MoveIt planned and executed the motion."

    def _send_action(self, client, goal, timeout=45.0):
        try:
            handle = self._await_future(client.send_goal_async(goal), timeout)
            if handle is None or not handle.accepted:
                return None
            return self._await_future(handle.get_result_async(), timeout)
        except Exception as error:
            self.node.get_logger().error("MoveIt action failed or timed out: %s" % error)
            return None

    def _operate_gripper(self, object_id, opening):
        backend = self.gripper.get("backend", "gazebo_detachable")
        if backend == "gazebo_detachable":
            return self._set_gazebo_attachment(object_id, attached=not opening)
        if backend == "follow_joint_trajectory":
            return self._trajectory_gripper(opening)
        self.node.get_logger().error("Unsupported gripper backend: %s" % backend)
        return False

    def _set_gazebo_attachment(self, object_id, attached, timeout=6.0):
        detached_target = not attached
        with self._state_condition:
            if self._gripper_detached.get(object_id) == detached_target:
                return True
        publishers = self.attach_publishers if attached else self.detach_publishers
        publisher = publishers.get(object_id)
        if publisher is None:
            return False
        deadline = time.monotonic() + timeout
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.05)
        if publisher.get_subscription_count() == 0:
            self.node.get_logger().error('No Gazebo bridge is listening for %s.' % object_id)
            return False

        # Gazebo DetachableJoint accepts Empty command topics, but its output topic
        # does not provide an initial state or a reliable command acknowledgement.
        # Send a few idempotent commands after the bridge subscriber is ready.
        message = Empty()
        for _ in range(3):
            publisher.publish(message)
            time.sleep(0.10)
        with self._state_condition:
            self._gripper_detached[object_id] = detached_target
            self._state_condition.notify_all()
        return True

    def _on_gripper_state(self, object_id, message):
        with self._state_condition:
            # DetachableJoint output reports whether the object is detached.
            self._gripper_detached[object_id] = bool(message.data)
            self._state_condition.notify_all()

    def _trajectory_gripper(self, opening):
        joints = self.gripper.get("joint_names", [])
        values = self.gripper.get("open_positions" if opening else "closed_positions", [])
        if self.gripper_client is None or not joints or len(joints) != len(values):
            self.node.get_logger().error("Configure matching gripper joints and position arrays.")
            return False
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in values]
        point.time_from_start.sec = int(self.gripper.get("motion_time_sec", 1))
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(joints)
        goal.trajectory.points = [point]
        try:
            handle = self._await_future(self.gripper_client.send_goal_async(goal), 10.0)
            if handle is None or not handle.accepted:
                return False
            result = self._await_future(handle.get_result_async(), 15.0)
            return (
                result is not None
                and result.status == GoalStatus.STATUS_SUCCEEDED
                and result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL
            )
        except Exception as error:
            self.node.get_logger().error("Gripper action failed: %s" % error)
            return False

    def _object_tcp_pose(self, obj):
        pose = Pose()
        position = obj["position_xyz"]
        clearance = float(self.scene.get("grasp_clearance_m", 0.002))
        pose.position.x = float(position[0])
        pose.position.y = float(position[1])
        pose.position.z = (
            float(position[2]) + float(obj["size_xyz"][2]) / 2.0 + clearance
        )
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = (
            self.orientation
        )
        return pose

    @staticmethod
    def _offset_pose(pose, dz):
        result = copy.deepcopy(pose)
        result.position.z += dz
        return result

    def _collision_object(self, object_id, obj):
        collision = CollisionObject()
        collision.header.frame_id = self.frame
        collision.id = object_id
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(value) for value in obj["size_xyz"]]
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = [
            float(value) for value in obj["position_xyz"]
        ]
        pose.orientation.w = 1.0
        collision.primitives = [primitive]
        collision.primitive_poses = [pose]
        collision.operation = CollisionObject.ADD
        return collision

    def _apply_world_scene(self):
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(self._collision_object(
            "work_table",
            {"size_xyz": self.table["size_xyz"], "position_xyz": self.table["center_xyz"]},
        ))
        if self.table.get("pedestal_center_xyz") is not None:
            scene.world.collision_objects.append(self._collision_object(
                "robot_pedestal",
                {
                    "size_xyz": self.table["pedestal_size_xyz"],
                    "position_xyz": self.table["pedestal_center_xyz"],
                },
            ))
        for object_id, obj in self.objects.items():
            if obj.get("position_xyz") is not None:
                scene.world.collision_objects.append(self._collision_object(object_id, obj))
        return self._apply_scene(scene)

    def _remove_world_object(self, object_id):
        scene = PlanningScene()
        scene.is_diff = True
        collision = CollisionObject()
        collision.header.frame_id = self.frame
        collision.id = object_id
        collision.operation = CollisionObject.REMOVE
        scene.world.collision_objects.append(collision)
        return self._apply_scene(scene)

    def _add_world_object(self, object_id, obj):
        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(self._collision_object(object_id, obj))
        return self._apply_scene(scene)

    def _attach_object(self, object_id, obj):
        attached = AttachedCollisionObject()
        attached.link_name = self.eef
        attached.touch_links = list(self.gripper.get("touch_links", [self.eef]))
        attached.object.id = object_id
        attached.object.header.frame_id = self.eef
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [float(value) for value in obj["size_xyz"]]
        pose = Pose()
        pose.position.z = (
            float(obj["size_xyz"][2]) / 2.0
            + float(self.scene.get("grasp_clearance_m", 0.002))
        )
        pose.orientation.w = 1.0
        attached.object.primitives = [primitive]
        attached.object.primitive_poses = [pose]
        attached.object.operation = CollisionObject.ADD
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        return self._apply_scene(scene)

    def _set_collision_allowed(self, first, second, allowed):
        request = GetPlanningScene.Request()
        request.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
        try:
            response = self._await_future(self.get_scene_client.call_async(request), 10.0)
        except Exception as error:
            self.node.get_logger().error("GetPlanningScene failed: %s" % error)
            return False
        if response is None:
            return False

        # Keep UR's SRDF self-collision exclusions; add or update only this pair.
        matrix = copy.deepcopy(response.scene.allowed_collision_matrix)
        names = list(matrix.entry_names)
        entries = list(matrix.entry_values)
        for row in entries:
            while len(row.enabled) < len(names):
                row.enabled.append(False)
        for name in (first, second):
            if name not in names:
                names.append(name)
                for row in entries:
                    row.enabled.append(False)
                new_row = AllowedCollisionEntry()
                new_row.enabled = [False] * len(names)
                entries.append(new_row)
        first_index = names.index(first)
        second_index = names.index(second)
        entries[first_index].enabled[second_index] = bool(allowed)
        entries[second_index].enabled[first_index] = bool(allowed)
        matrix.entry_names = names
        matrix.entry_values = entries

        scene = PlanningScene()
        scene.is_diff = True
        scene.allowed_collision_matrix = matrix
        return self._apply_scene(scene)

    def _detach_and_add_world(self, object_id, obj):
        attached = AttachedCollisionObject()
        attached.link_name = self.eef
        attached.object.id = object_id
        attached.object.operation = CollisionObject.REMOVE
        scene = PlanningScene()
        scene.is_diff = True
        scene.robot_state.is_diff = True
        scene.robot_state.attached_collision_objects.append(attached)
        scene.world.collision_objects.append(self._collision_object(object_id, obj))
        return self._apply_scene(scene)

    @staticmethod
    def _await_future(future, timeout):
        completed = threading.Event()
        future.add_done_callback(lambda _: completed.set())
        if not completed.wait(timeout):
            return None
        return future.result()

    def _apply_scene(self, scene):
        try:
            request = ApplyPlanningScene.Request()
            request.scene = scene
            response = self._await_future(self.scene_client.call_async(request), 10.0)
            return response is not None and response.success
        except Exception as error:
            self.node.get_logger().error("ApplyPlanningScene failed: %s" % error)
            return False
