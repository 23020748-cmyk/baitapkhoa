"""Color segmentation and overhead-camera pinhole projection utilities."""

import math

import cv2
import numpy as np


def detect_colored_objects(image_bgr, color_ranges, camera, table, minimum_area_px=35):
    """Return color-centroid detections mapped to XY positions on the table plane."""
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)
    detections = {}
    for object_id, ranges in color_ranges.items():
        mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for values in ranges:
            low = np.array(values[:3], dtype=np.uint8)
            high = np.array(values[3:], dtype=np.uint8)
            mask = cv2.bitwise_or(mask, cv2.inRange(hsv, low, high))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            continue
        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        if area < float(minimum_area_px):
            continue
        moments = cv2.moments(contour)
        if moments["m00"] <= 0.0:
            continue
        pixel = (
            float(moments["m10"] / moments["m00"]),
            float(moments["m01"] / moments["m00"]),
        )
        position_xy = pixel_to_world(pixel[0], pixel[1], camera)
        half_x = float(table["size_xyz"][0]) / 2.0
        half_y = float(table["size_xyz"][1]) / 2.0
        center_x, center_y = (float(v) for v in table["center_xyz"][:2])
        if (
            center_x - half_x <= position_xy[0] <= center_x + half_x
            and center_y - half_y <= position_xy[1] <= center_y + half_y
        ):
            detections[object_id] = {
                "pixel_xy": pixel,
                "position_xy": position_xy,
                "area_px": area,
            }
    return detections


def pixel_to_world(pixel_x, pixel_y, camera):
    """Intersect a calibrated camera ray with the detected object's horizontal plane."""
    width, height = (int(v) for v in camera["image_size"])
    x_cam, y_cam, z_cam = (float(v) for v in camera["position_xyz"])
    plane_z = float(camera.get("detection_plane_z", camera["table_surface_z"]))
    focal_x = width / (2.0 * math.tan(float(camera["horizontal_fov_rad"]) / 2.0))
    focal_y = focal_x
    pixel_right = (float(pixel_x) - width / 2.0) / focal_x
    pixel_down = (float(pixel_y) - height / 2.0) / focal_y

    if "rotation_rpy_rad" in camera:
        roll, pitch, yaw = (float(v) for v in camera["rotation_rpy_rad"])
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        rotation = np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ], dtype=float)
        forward = rotation @ np.array([1.0, 0.0, 0.0])
        image_right = rotation @ np.array([0.0, -1.0, 0.0])
        image_down = rotation @ np.array([0.0, 0.0, -1.0])
        ray = forward + pixel_right * image_right + pixel_down * image_down
        if ray[2] >= -1e-6:
            return [float("nan"), float("nan")]
        distance = (plane_z - z_cam) / ray[2]
        if distance <= 0.0:
            return [float("nan"), float("nan")]
        world = np.array([x_cam, y_cam, z_cam]) + distance * ray
        return [float(world[0]), float(world[1])]

    # Compatibility for a camera pointing straight down.
    scale = z_cam - plane_z
    offset_right = pixel_right * scale
    offset_down = pixel_down * scale
    right_axis = camera.get("image_right_axis_world")
    down_axis = camera.get("image_down_axis_world")
    if right_axis is not None and down_axis is not None:
        world_x = x_cam + offset_right * float(right_axis[0]) + offset_down * float(down_axis[0])
        world_y = y_cam + offset_right * float(right_axis[1]) + offset_down * float(down_axis[1])
    else:
        local_y = -offset_down
        yaw = float(camera.get("yaw_rad", 0.0))
        world_x = x_cam + math.cos(yaw) * offset_right - math.sin(yaw) * local_y
        world_y = y_cam + math.sin(yaw) * offset_right + math.cos(yaw) * local_y
    return [world_x, world_y]
