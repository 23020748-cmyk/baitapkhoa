"""LLM-only skill planner with camera state, validation and task-result reporting."""

import copy
import json
import queue
import threading
import time
import urllib.error
import urllib.request
import uuid

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .scene_state import build_world_state
from .task_validator import PlanValidationError, TaskValidator


def read_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


class LlmPlanner(Node):
    def __init__(self):
        super().__init__("llm_task_planner")
        share = get_package_share_directory("ur3_llm_control")
        self.declare_parameter("scene_config", share + "/config/scene_bai2.yaml")
        self.declare_parameter("student_config", share + "/config/student_config.yaml")
        self.declare_parameter("require_camera", False)
        self.declare_parameter("world_state_timeout_sec", 10.0)
        self.declare_parameter("max_world_state_age_sec", 2.0)
        self.declare_parameter("ollama_url", "http://localhost:11434/api/chat")
        self.declare_parameter("model", "qwen2.5:0.5b")
        self.declare_parameter("request_timeout_sec", 300.0)
        self.declare_parameter("task_result_timeout_sec", 1800.0)
        self.scene = read_yaml(self.get_parameter("scene_config").value)
        self.student = read_yaml(self.get_parameter("student_config").value)
        self.objects = self.scene["objects"]
        self.zones = self.scene["zones"]
        self.require_camera = bool(self.get_parameter("require_camera").value)
        self.validator = TaskValidator(
            self.objects, self.zones, self.scene.get("temporary_slots", {})
        )
        self._commands = queue.Queue()
        self._ready = threading.Event()
        self._state_condition = threading.Condition()
        self._world_state = None if self.require_camera else build_world_state(self.scene)
        self._state_received_monotonic = 0.0
        self._condition = threading.Condition()
        self._results = {}
        self.create_subscription(String, "/task_command", self._on_command, 10)
        self.create_subscription(Bool, "/skill_executor/ready", self._on_ready, 10)
        self.create_subscription(String, "/task_result", self._on_result, 10)
        self.create_subscription(String, "/world_state", self._on_world_state, 10)
        self.plan_pub = self.create_publisher(String, "/validated_plan", 10)
        threading.Thread(target=self._process_commands, daemon=True).start()
        self.get_logger().info(
            "Listening on /task_command; LLM outputs whitelisted skill JSON only."
        )

    def _on_command(self, message):
        command = message.data.strip()
        if command:
            self._commands.put(command)
        else:
            self.get_logger().warning("Ignoring an empty task command.")

    def _on_ready(self, message):
        if message.data:
            self._ready.set()

    def _on_world_state(self, message):
        try:
            state = json.loads(message.data)
            if not isinstance(state, dict) or not isinstance(state.get("objects"), dict):
                raise ValueError("missing objects")
            with self._state_condition:
                self._world_state = state
                self._state_received_monotonic = time.monotonic()
                self._state_condition.notify_all()
        except (ValueError, TypeError) as error:
            self.get_logger().warning("Ignoring malformed /world_state: %s" % error)

    def _on_result(self, message):
        try:
            result = json.loads(message.data)
            with self._condition:
                self._results[result["task_id"]] = result
                self._condition.notify_all()
        except (ValueError, KeyError, TypeError):
            self.get_logger().warning("Ignoring malformed /task_result message.")

    def _process_commands(self):
        while rclpy.ok():
            try:
                command = self._commands.get(timeout=0.2)
            except queue.Empty:
                continue
            self._run_command(command)

    def _run_command(self, command):
        print("\nUSER COMMAND:\n" + command, flush=True)
        try:
            world_state = self._wait_for_world_state()
            llm_plan = self.validator.validate(self._ask_model(command, world_state))
            plan = self.validator.resolve_occupied_zones(llm_plan, world_state)
        except (PlanValidationError, ValueError, KeyError, OSError, TimeoutError) as error:
            print("\nLLM PLAN REJECTED: %s\nTASK FAILED" % error, flush=True)
            self.get_logger().error("Planner rejected task: %s" % error)
            return

        print("\nLLM PLAN:", flush=True)
        for number, step in enumerate(llm_plan, 1):
            print("%d. %s" % (number, self._format_step(step)), flush=True)
        if plan != llm_plan:
            print("\nCAMERA PRECHECK: occupied zone detected; adding a temporary relocation.", flush=True)
            print("VALIDATED EXECUTION PLAN:", flush=True)
            for number, step in enumerate(plan, 1):
                print("%d. %s" % (number, self._format_step(step)), flush=True)

        if not self._ready.wait(timeout=120.0):
            print("\nTASK FAILED: skill executor is not ready.", flush=True)
            self.get_logger().error("Timed out waiting for skill executor.")
            return

        task_id = str(uuid.uuid4())
        payload = String()
        payload.data = json.dumps(
            {"task_id": task_id, "command": command, "plan": plan},
            ensure_ascii=False,
            separators=(",", ":"),
        )
        self.plan_pub.publish(payload)
        result_timeout = float(self.get_parameter("task_result_timeout_sec").value)
        with self._condition:
            self._condition.wait_for(
                lambda: task_id in self._results or not rclpy.ok(),
                timeout=result_timeout,
            )
            result = self._results.pop(task_id, None)
        if result is None:
            print(
                "\nTASK FAILED: timed out waiting for the skill executor after %.0f seconds."
                % result_timeout,
                flush=True,
            )
        elif result.get("success"):
            print("\nTASK SUCCESS", flush=True)
        else:
            print("\nTASK FAILED: %s" % result.get("reason", "skill execution failed"), flush=True)

    def _wait_for_world_state(self):
        timeout = float(self.get_parameter("world_state_timeout_sec").value)
        age_limit = float(self.get_parameter("max_world_state_age_sec").value)
        deadline = time.monotonic() + timeout
        with self._state_condition:
            while rclpy.ok():
                if self._world_state is not None:
                    age = time.monotonic() - self._state_received_monotonic
                    if not self.require_camera or age <= age_limit:
                        state = copy.deepcopy(self._world_state)
                        missing = [
                            object_id
                            for object_id, item in state.get("objects", {}).items()
                            if not item.get("visible", False) or item.get("position_xyz") is None
                        ]
                        if self.require_camera and missing:
                            remaining = deadline - time.monotonic()
                            if remaining <= 0.0:
                                raise TimeoutError(
                                    "Camera could not locate: %s" % ", ".join(missing)
                                )
                            self._state_condition.wait(timeout=min(0.2, remaining))
                            continue
                        return state
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    raise TimeoutError("No fresh camera world state was received.")
                self._state_condition.wait(timeout=min(0.2, remaining))
        raise TimeoutError("ROS is shutting down.")

    def _format_step(self, step):
        skill = step["skill"]
        if skill == "pick":
            label = self.objects[step["object"]].get("label_en", step["object"])
            return "pick(%s)" % label
        if skill == "place":
            label = self.objects[step["object"]].get("label_en", step["object"])
            zone = self.zones[step["zone"]].get("label_en", step["zone"])
            return "place(%s, %s)" % (label, zone)
        if skill == "place_temp":
            label = self.objects[step["object"]].get("label_en", step["object"])
            return "place_temp(%s, %s)" % (label, step["slot"])
        return "home()"

    def _ask_model(self, command, world_state):
        student_id = str(self.student["student_id"])
        last_two = int(student_id[-2:])
        p_value = last_two % 6
        mapping = self.student["assignments"][str(p_value)]
        object_catalog = {
            key: item.get("label_vi", key) for key, item in self.objects.items()
        }
        zone_catalog = {
            key: item.get("label_vi", key) for key, item in self.zones.items()
        }
        occupancy = {
            zone: value.get("occupied_by")
            for zone, value in world_state.get("zones", {}).items()
        }
        free_slots = [
            slot
            for slot, value in world_state.get("temporary_slots", {}).items()
            if value.get("occupied_by") is None
        ]
        object_locations = {
            object_id: {
                "visible": item.get("visible", False),
                "zone": item.get("zone_id"),
                "temporary_slot": item.get("temporary_slot"),
            }
            for object_id, item in world_state.get("objects", {}).items()
        }
        system = (
            "Translate the user's natural-language request into JSON skill steps for a UR3/UR3e. "
            "Never output joint values, trajectories, or low-level robot commands. "
            "Allowed skills are pick(object), place(object, zone), "
            "place_temp(object, slot), and home(). Use only exact catalog IDs. "
            "Every place must have a preceding pick of the same object. Keep each pick/place pair "
            "together, and end every plan with home(). For a normal request to put one object in "
            "a zone, output only pick(requested_object), place(requested_object, requested_zone), "
            "home(). Do not add steps to move an object already occupying that zone: the Python "
            "validator checks occupancy and inserts a safe temporary relocation automatically. "
            "Use place(object, zone) with zone_a, zone_b, or zone_c only. Use "
            "place_temp(object, slot) with temp_1 or temp_2 only; never put a temporary slot ID "
            "in the zone field. For a request to arrange all objects, use the personal student-ID "
            "zone-to-object assignment and emit one pick/place pair for each object that must move. "
            "If an object is not visible or the request is unclear, return an empty plan. "
            "Return JSON only. Example: for 'Put the red cube in Zone B', return "
            "{\"plan\":[{\"skill\":\"pick\",\"object\":\"red_cube\"},"
            "{\"skill\":\"place\",\"object\":\"red_cube\",\"zone\":\"zone_b\"},"
            "{\"skill\":\"home\"}]}. For 'Put the purple cube in temp_1', return "
            "{\"plan\":[{\"skill\":\"pick\",\"object\":\"purple_cube\"},"
            "{\"skill\":\"place_temp\",\"object\":\"purple_cube\",\"slot\":\"temp_1\"},"
            "{\"skill\":\"home\"}]}."
            "\nObjects: %s" % json.dumps(object_catalog, ensure_ascii=False)
            + "\nZones: %s" % json.dumps(zone_catalog, ensure_ascii=False)
            + "\nCamera object locations: %s" % json.dumps(object_locations, ensure_ascii=False)
            + "\nZone occupancy: %s" % json.dumps(occupancy, ensure_ascii=False)
            + "\nFree temporary slots: %s" % json.dumps(free_slots, ensure_ascii=False)
            + "\nStudent: %s, ID %s, P=%d mod 6=%d."
            % (self.student["student_name"], student_id, last_two, p_value)
            + "\nPersonal assignment zone -> object: %s"
            % json.dumps(mapping, ensure_ascii=False)
        )
        body = json.dumps({
            "model": self.get_parameter("model").value,
            "stream": False,
            "format": "json",
            "options": {"num_predict": 1024, "num_ctx": 4096, "temperature": 0.1},
            "messages": [
                {"role": "system", "content": system},
                {"role": "user", "content": command},
            ],
        }).encode("utf-8")
        request = urllib.request.Request(
            self.get_parameter("ollama_url").value,
            data=body,
            headers={"Content-Type": "application/json"},
            method="POST",
        )
        try:
            with urllib.request.urlopen(
                request, timeout=float(self.get_parameter("request_timeout_sec").value)
            ) as response:
                reply = json.loads(response.read().decode("utf-8"))
        except urllib.error.URLError as error:
            raise OSError("Could not connect to Ollama: %s" % error) from error
        content = reply.get("message", {}).get("content", "")
        try:
            plan = json.loads(content)
        except (TypeError, ValueError) as error:
            raise ValueError("LLM did not return valid JSON: %s" % content[:500]) from error
        self.get_logger().info(
            "LLM response JSON: %s" % json.dumps(plan, ensure_ascii=False)
        )
        return plan


def main(args=None):
    rclpy.init(args=args)
    node = LlmPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
