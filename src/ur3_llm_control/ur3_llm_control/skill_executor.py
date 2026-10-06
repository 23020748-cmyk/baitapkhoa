"""Revalidates a plan at the robot boundary and executes MoveIt skills."""

import json
import queue
import threading
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .robot_skills import RobotSkills
from .scene_state import build_world_state
from .task_validator import PlanValidationError, TaskValidator


class SkillExecutor(Node):
    def __init__(self):
        super().__init__("skill_executor")
        share = get_package_share_directory("ur3_llm_control")
        self.declare_parameter("scene_config", share + "/config/scene_bai2.yaml")
        self.declare_parameter("require_camera", False)
        with open(self.get_parameter("scene_config").value, "r", encoding="utf-8") as stream:
            self.scene = yaml.safe_load(stream)
        self.require_camera = bool(self.get_parameter("require_camera").value)
        self.validator = TaskValidator(
            self.scene["objects"],
            self.scene["zones"],
            self.scene.get("temporary_slots", {}),
        )
        self._state_lock = threading.Lock()
        self._world_state = None if self.require_camera else build_world_state(self.scene)
        self._state_received = 0.0
        self.robot = RobotSkills(self, self.scene, require_camera=self.require_camera)
        if self._world_state is not None:
            self.robot.update_world_state(self._world_state)

        self._initialized = threading.Event()
        self._initializing = False
        self._plans = queue.Queue()
        self.create_subscription(String, "/validated_plan", self._on_plan, 10)
        self.create_subscription(String, "/world_state", self._on_world_state, 10)
        self.ready_pub = self.create_publisher(Bool, "/skill_executor/ready", 10)
        self.result_pub = self.create_publisher(String, "/task_result", 10)
        self.create_timer(0.5, self._initialize)
        threading.Thread(target=self._execute_queue, daemon=True).start()

    def _on_world_state(self, message):
        try:
            state = json.loads(message.data)
            if not isinstance(state, dict) or not isinstance(state.get("objects"), dict):
                raise ValueError("missing objects")
            with self._state_lock:
                self._world_state = state
                self._state_received = time.monotonic()
            self.robot.update_world_state(state)
        except (ValueError, TypeError) as error:
            self.get_logger().warning("Ignoring malformed /world_state: %s" % error)

    def _initialize(self):
        if self._initialized.is_set():
            self.ready_pub.publish(Bool(data=True))
            return
        if self._initializing:
            return
        self._initializing = True
        threading.Thread(target=self._initialize_worker, daemon=True).start()

    def _initialize_worker(self):
        ready, reason = self.robot.initialize()
        self._initializing = False
        if ready:
            self._initialized.set()
            self.get_logger().info("MoveIt scene and physical Gazebo gripper are ready.")
        else:
            self.get_logger().warning("Waiting for MoveIt and Gazebo gripper: %s" % reason)

    def _on_plan(self, message):
        self._plans.put(message.data)

    def _execute_queue(self):
        while rclpy.ok():
            try:
                encoded = self._plans.get(timeout=0.2)
            except queue.Empty:
                continue
            self._execute_plan(encoded)

    def _execute_plan(self, encoded):
        if not self._initialized.wait(timeout=120.0):
            self.get_logger().error("MoveIt is not ready; rejected plan.")
            return
        task_id = None
        command = ""
        try:
            envelope = json.loads(encoded)
            task_id = envelope["task_id"]
            command = envelope.get("command", "")
            incoming = self.validator.validate({"plan": envelope["plan"]})
            state = self._current_world_state()
            plan = self.validator.resolve_occupied_zones(incoming, state)
            plan = self.validator.validate({"plan": plan}, world_state=state)
            if not self.robot.sync_scene_from_camera(state):
                raise RuntimeError("Could not synchronize MoveIt collision objects with camera.")
        except (ValueError, KeyError, PlanValidationError, TypeError, RuntimeError) as error:
            self.get_logger().error("Plan rejected at executor boundary: %s" % error)
            if task_id:
                self._publish_result(task_id, False, str(error))
            return

        print("\nEXECUTION:", flush=True)
        success = True
        reason = ""
        for step in plan:
            status, reason = self._execute_step(step)
            line = "%s ........ %s" % (self._format_step(step), status)
            if status != RobotSkills.SUCCESS:
                line += " -- " + reason
            print(line, flush=True)
            if status != RobotSkills.SUCCESS:
                success = False
                break
        self._publish_result(task_id, success, reason)

    def _current_world_state(self):
        if not self.require_camera:
            return self._world_state or build_world_state(self.scene)
        age_limit = 2.0
        with self._state_lock:
            state = self._world_state
            age = time.monotonic() - self._state_received
        if state is None or age > age_limit:
            raise PlanValidationError("No fresh camera state at the skill executor.")
        missing = [
            object_id
            for object_id, data in state.get("objects", {}).items()
            if not data.get("visible", False) or data.get("position_xyz") is None
        ]
        if missing:
            raise PlanValidationError(
                "Camera cannot locate before execution: %s" % ", ".join(missing)
            )
        return state

    def _execute_step(self, step):
        skill = step["skill"]
        if skill == "pick":
            return self.robot.pick(step["object"])
        if skill == "place":
            return self.robot.place(step["object"], step["zone"])
        if skill == "place_temp":
            return self.robot.place_temp(step["object"], step["slot"])
        return self.robot.home()

    def _format_step(self, step):
        skill = step["skill"]
        if skill == "pick":
            label = self.scene["objects"][step["object"]].get("label_en", step["object"])
            return "pick(%s)" % label
        if skill == "place":
            label = self.scene["objects"][step["object"]].get("label_en", step["object"])
            zone = self.scene["zones"][step["zone"]].get("label_en", step["zone"])
            return "place(%s, %s)" % (label, zone)
        if skill == "place_temp":
            label = self.scene["objects"][step["object"]].get("label_en", step["object"])
            return "place_temp(%s, %s)" % (label, step["slot"])
        return "home()"

    def _publish_result(self, task_id, success, reason):
        message = String()
        message.data = json.dumps(
            {
                "task_id": task_id,
                "success": success,
                "reason": reason,
            },
            ensure_ascii=False,
        )
        self.result_pub.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = SkillExecutor()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
