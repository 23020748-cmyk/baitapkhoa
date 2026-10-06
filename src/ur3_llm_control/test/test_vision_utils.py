import unittest

import cv2
import numpy as np

from ur3_llm_control.vision_utils import detect_colored_objects, pixel_to_world


class VisionUtilsTests(unittest.TestCase):
    def setUp(self):
        self.camera = {
            "position_xyz": [0.55, 0.0, 1.1],
            "table_surface_z": 0.32,
            "image_size": [640, 480],
            "horizontal_fov_rad": 1.0471975512,
            "yaw_rad": 0.0,
        }
        self.table = {
            "center_xyz": [0.55, 0.0, 0.30],
            "size_xyz": [0.8, 0.9, 0.04],
        }

    def test_pixel_projection_maps_center_and_axes(self):
        center = pixel_to_world(320, 240, self.camera)
        right = pixel_to_world(420, 240, self.camera)
        up = pixel_to_world(320, 140, self.camera)
        self.assertAlmostEqual(center[0], 0.55, places=6)
        self.assertAlmostEqual(center[1], 0.0, places=6)
        self.assertGreater(right[0], center[0])
        self.assertGreater(up[1], center[1])

    def test_top_down_camera_projection_uses_rotated_image_axes(self):
        camera = dict(self.camera)
        camera["image_right_axis_world"] = [0.0, -1.0]
        camera["image_down_axis_world"] = [-1.0, 0.0]
        center = pixel_to_world(320, 240, camera)
        right = pixel_to_world(420, 240, camera)
        down = pixel_to_world(320, 340, camera)
        self.assertGreater(center[0], down[0])
        self.assertGreater(center[1], right[1])

    def test_detects_red_yellow_and_blue_cubes_in_rgb_frame(self):
        image = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.rectangle(image, (270, 210), (310, 250), (0, 0, 255), -1)
        cv2.rectangle(image, (330, 210), (370, 250), (0, 255, 255), -1)
        cv2.rectangle(image, (390, 210), (430, 250), (255, 0, 0), -1)
        color_ranges = {
            "red_cube": [[0, 120, 70, 10, 255, 255], [170, 120, 70, 179, 255, 255]],
            "yellow_cube": [[18, 100, 90, 38, 255, 255]],
            "blue_cube": [[95, 100, 70, 135, 255, 255]],
        }
        detections = detect_colored_objects(
            image, color_ranges, self.camera, self.table, minimum_area_px=35
        )
        self.assertEqual(set(detections), set(color_ranges))
        self.assertGreater(detections["blue_cube"]["position_xy"][0],
                           detections["yellow_cube"]["position_xy"][0])


if __name__ == "__main__":
    unittest.main()
