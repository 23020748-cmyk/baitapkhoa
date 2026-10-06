"""Helpers for turning camera or configured scene data into a world-state snapshot."""

import math
import time


def zone_for_position(position_xyz, zones):
    """Return the zone containing an object's XY center, or None."""
    x, y = float(position_xyz[0]), float(position_xyz[1])
    for zone_id, zone in zones.items():
        center = zone.get("center_xyz", zone.get("surface_xyz"))
        size = zone.get("size_xy", [0.14, 0.14])
        if (
            abs(x - float(center[0])) <= float(size[0]) / 2.0
            and abs(y - float(center[1])) <= float(size[1]) / 2.0
        ):
            return zone_id
    return None


def slot_for_position(position_xyz, slots):
    """Return the nearest temporary slot within its configured occupancy radius."""
    x, y = float(position_xyz[0]), float(position_xyz[1])
    for slot_id, slot in slots.items():
        center = slot["position_xyz"]
        radius = float(slot.get("occupancy_radius_m", 0.055))
        if math.hypot(x - float(center[0]), y - float(center[1])) <= radius:
            return slot_id
    return None


def build_world_state(scene, positions=None, visible_ids=None, stamp_unix=None):
    """Build a serializable state from detected poses; config poses are for Bài 2 only."""
    uses_configured_poses = positions is None
    positions = positions or {}
    if visible_ids is None:
        visible_ids = set(positions)
        if uses_configured_poses:
            visible_ids.update(
                object_id
                for object_id, item in scene['objects'].items()
                if item.get('position_xyz') is not None
            )
    else:
        visible_ids = set(visible_ids)

    objects = {}
    zones = {
        zone_id: {"occupied": False, "occupied_by": None}
        for zone_id in scene["zones"]
    }
    slots_config = scene.get("temporary_slots", {})
    slots = {
        slot_id: {
            "occupied": False,
            "occupied_by": None,
            "position_xyz": [float(v) for v in slot["position_xyz"]],
        }
        for slot_id, slot in slots_config.items()
    }

    for object_id, obj in scene["objects"].items():
        position = positions.get(object_id)
        if position is None and object_id not in positions:
            position = obj.get("position_xyz")
        position = [float(v) for v in position] if position is not None else None
        visible = object_id in visible_ids and position is not None
        zone_id = zone_for_position(position, scene["zones"]) if visible else None
        temporary_slot = slot_for_position(position, slots_config) if visible else None
        objects[object_id] = {
            "position_xyz": position,
            "visible": visible,
            "confidence": 1.0 if visible else 0.0,
            "zone_id": zone_id,
            "temporary_slot": temporary_slot,
        }
        if visible and zone_id is not None:
            if zones[zone_id]["occupied_by"] is None:
                zones[zone_id]["occupied_by"] = object_id
            else:
                zones[zone_id]["occupied_by"] = "AMBIGUOUS"
            zones[zone_id]["occupied"] = True
        if visible and temporary_slot is not None:
            if slots[temporary_slot]["occupied_by"] is None:
                slots[temporary_slot]["occupied_by"] = object_id
            else:
                slots[temporary_slot]["occupied_by"] = "AMBIGUOUS"
            slots[temporary_slot]["occupied"] = True

    return {
        "stamp_unix": float(stamp_unix if stamp_unix is not None else time.time()),
        "world_frame": scene.get("world_frame", "base_link"),
        "objects": objects,
        "zones": zones,
        "temporary_slots": slots,
    }
