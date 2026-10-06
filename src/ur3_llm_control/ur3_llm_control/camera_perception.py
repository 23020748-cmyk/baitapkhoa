"""Gazebo RGB camera node that detects colored blocks and zone occupancy."""

import json
import time

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from .scene_state import build_world_state
from .vision_utils import detect_colored_objects


class CameraPerception(Node):
    def __init__(self):
        super().__init__("camera_perception")
        share = get_package_share_directory("ur3_llm_control")
        self.declare_parameter("scene_config", share + "/config/scene_bai3.yaml")
        self.declare_parameter("image_topic", "/camera/image")
        self.declare_parameter("world_state_topic", "/world_state")
        self.declare_parameter("minimum_area_px", 35.0)
        with open(self.get_parameter("scene_config").value, "r", encoding="utf-8") as stream:
            self.scene = yaml.safe_load(stream)
        self.bridge = CvBridge()
        self.sequence = 0
        self.last_positions = {}
        self.last_detections = {}
        self.publisher = self.create_publisher(
            String, self.get_parameter("world_state_topic").value, 10
        )
        self.subscription = self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self._on_image,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "Detecting blocks from %s; object coordinates come from the RGB image."
            % self.get_parameter("image_topic").value
        )

    def _on_image(self, message):
        try:
            image = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            camera = dict(self.scene["camera"])
            camera["image_size"] = [int(message.width), int(message.height)]
            camera["table_surface_z"] = (
                float(self.scene["table"]["center_xyz"][2])
                + float(self.scene["table"]["size_xyz"][2]) / 2.0
            )
            camera["detection_plane_z"] = camera["table_surface_z"] + 0.75 * max(
                float(obj["size_xyz"][2]) for obj in self.scene["objects"].values()
            )
            color_ranges = {
                object_id: obj["hsv_ranges"]
                for object_id, obj in self.scene["objects"].items()
            }
            detections = detect_colored_objects(
                image,
                color_ranges,
                camera,
                self.scene["table"],
                minimum_area_px=float(self.get_parameter("minimum_area_px").value),
            )
        except Exception as error:
            self.get_logger().warning("Could not process camera frame: %s" % error)
            return

        visible = set(detections)
        for object_id, detection in detections.items():
            object_height = float(self.scene["objects"][object_id]["size_xyz"][2])
            surface_z = camera["table_surface_z"]
            self.last_positions[object_id] = [
                float(detection["position_xy"][0]),
                float(detection["position_xy"][1]),
                surface_z + object_height / 2.0,
            ]
            self.last_detections[object_id] = detection

        state = build_world_state(
            self.scene,
            positions=self.last_positions,
            visible_ids=visible,
            stamp_unix=time.time(),
        )
        for object_id, detection in detections.items():
            state["objects"][object_id]["confidence"] = min(
                1.0, float(detection["area_px"]) / 220.0
            )
            state["objects"][object_id]["pixel_xy"] = [
                float(v) for v in detection["pixel_xy"]
            ]
        self.sequence += 1
        state["sequence"] = self.sequence
        output = String()
        output.data = json.dumps(state, separators=(",", ":"), ensure_ascii=False)
        self.publisher.publish(output)

    def destroy_node(self):
        self.subscription = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
