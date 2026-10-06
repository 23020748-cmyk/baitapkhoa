"""Whitelist validation and occupancy-aware expansion of LLM skill plans."""


class PlanValidationError(ValueError):
    """Raised when a plan contains an unsupported or unsafe instruction."""


class TaskValidator:
    SKILLS = {"pick", "place", "place_temp", "home"}
    MAX_STEPS = 48

    def __init__(self, objects, zones, temporary_slots=None):
        self.objects = set(objects)
        self.zones = set(zones)
        self.temporary_slot_order = list(temporary_slots or ())
        self.temporary_slots = set(self.temporary_slot_order)

    def validate(self, payload, world_state=None):
        if not isinstance(payload, dict) or set(payload) != {"plan"}:
            raise PlanValidationError("Expected a JSON object with exactly one 'plan' field.")
        steps = payload["plan"]
        if not isinstance(steps, list) or not steps or len(steps) > self.MAX_STEPS:
            raise PlanValidationError(f"'plan' must contain 1 to {self.MAX_STEPS} steps.")

        held = None
        picked = {}
        placed = {}
        validated = []
        for index, step in enumerate(steps):
            if not isinstance(step, dict) or not isinstance(step.get("skill"), str):
                raise PlanValidationError(f"Step {index + 1} must contain a skill string.")
            skill = step["skill"].strip().lower()
            if skill not in self.SKILLS:
                raise PlanValidationError(
                    f"Unsupported skill at step {index + 1}: {step['skill']!r}."
                )
            if skill == "pick":
                if set(step) != {"skill", "object"}:
                    raise PlanValidationError(f"pick at step {index + 1} requires only 'object'.")
                obj = self._check_object(step["object"], index)
                if held is not None:
                    raise PlanValidationError("Place the held object before picking another.")
                held = obj
                picked[obj] = picked.get(obj, 0) + 1
                validated.append({"skill": skill, "object": obj})
            elif skill in {"place", "place_temp"}:
                expected = {"skill", "object", "zone"} if skill == "place" else {
                    "skill", "object", "slot"
                }
                if set(step) != expected:
                    args = "object and zone" if skill == "place" else "object and slot"
                    raise PlanValidationError(
                        f"{skill} at step {index + 1} requires {args}."
                    )
                obj = self._check_object(step["object"], index)
                target_key = "zone" if skill == "place" else "slot"
                target = step[target_key]
                allowed = self.zones if skill == "place" else self.temporary_slots
                if not isinstance(target, str) or target not in allowed:
                    label = "zone" if skill == "place" else "temporary slot"
                    raise PlanValidationError(
                        f"Invalid {label} at step {index + 1}: {target!r}."
                    )
                if held != obj:
                    raise PlanValidationError(f"Cannot place {obj!r}; it is not currently held.")
                held = None
                placed[obj] = placed.get(obj, 0) + 1
                validated.append({"skill": skill, "object": obj, target_key: target})
            else:
                if set(step) != {"skill"}:
                    raise PlanValidationError(f"home at step {index + 1} takes no parameters.")
                if index != len(steps) - 1:
                    raise PlanValidationError("home() must be the final step.")
                if held is not None:
                    raise PlanValidationError("The plan cannot finish with an object still held.")
                validated.append({"skill": skill})

        if validated[-1]["skill"] != "home":
            raise PlanValidationError("Every task plan must finish with home().")
        if picked != placed:
            raise PlanValidationError("Every picked object must have one matching place step.")
        if world_state is not None:
            self._validate_world_state(validated, world_state)
        return validated

    def resolve_occupied_zones(self, plan, world_state):
        """Insert camera-driven relocations before placing into an occupied zone."""
        if not isinstance(world_state, dict):
            raise PlanValidationError("A current world state is required before execution.")
        objects_state = world_state.get("objects", {})
        zones_state = world_state.get("zones", {})
        slots_state = world_state.get("temporary_slots", {})
        zone_occupancy = {
            zone: data.get("occupied_by") for zone, data in zones_state.items()
        }
        slot_occupancy = {
            slot: data.get("occupied_by") for slot, data in slots_state.items()
        }
        locations = {}
        for object_id, data in objects_state.items():
            locations[object_id] = (data.get("zone_id"), data.get("temporary_slot"))

        result = []
        index = 0
        while index < len(plan):
            step = plan[index]
            if step["skill"] == "home":
                result.append(step)
                index += 1
                continue
            if step["skill"] != "pick" or index + 1 >= len(plan):
                raise PlanValidationError("Each pick must be followed immediately by a place.")
            drop = plan[index + 1]
            obj = step["object"]
            self._require_visible(obj, objects_state)
            self._clear_location(obj, locations, zone_occupancy, slot_occupancy)

            if drop["skill"] == "place":
                target_zone = drop["zone"]
                blocker = zone_occupancy.get(target_zone)
                if blocker not in (None, "AMBIGUOUS", obj):
                    slot = self._first_free_slot(slot_occupancy)
                    if slot is None:
                        raise PlanValidationError(
                            f"{target_zone} is occupied by {blocker}, and no temporary slot is free."
                        )
                    self._require_visible(blocker, objects_state)
                    result.extend([
                        {"skill": "pick", "object": blocker},
                        {"skill": "place_temp", "object": blocker, "slot": slot},
                    ])
                    self._clear_location(blocker, locations, zone_occupancy, slot_occupancy)
                    locations[blocker] = (None, slot)
                    slot_occupancy[slot] = blocker
                elif blocker == "AMBIGUOUS":
                    raise PlanValidationError(
                        f"Camera found more than one object in {target_zone}; refusing to plan."
                    )
                result.extend([step, drop])
                zone_occupancy[target_zone] = obj
                locations[obj] = (target_zone, None)
            else:
                slot = drop["slot"]
                blocker = slot_occupancy.get(slot)
                if blocker not in (None, "AMBIGUOUS", obj):
                    raise PlanValidationError(
                        f"Temporary slot {slot} is occupied by {blocker}."
                    )
                if blocker == "AMBIGUOUS":
                    raise PlanValidationError(
                        f"Camera found multiple objects in temporary slot {slot}."
                    )
                result.extend([step, drop])
                slot_occupancy[slot] = obj
                locations[obj] = (None, slot)
            index += 2

        return self.validate({"plan": result}, world_state=world_state)

    def _validate_world_state(self, steps, world_state):
        objects_state = world_state.get("objects", {})
        zones_state = world_state.get("zones", {})
        slots_state = world_state.get("temporary_slots", {})
        zone_occupancy = {
            zone: data.get("occupied_by") for zone, data in zones_state.items()
        }
        slot_occupancy = {
            slot: data.get("occupied_by") for slot, data in slots_state.items()
        }
        locations = {
            object_id: (data.get("zone_id"), data.get("temporary_slot"))
            for object_id, data in objects_state.items()
        }
        held = None
        for index, step in enumerate(steps):
            skill = step["skill"]
            if skill == "pick":
                obj = step["object"]
                self._require_visible(obj, objects_state, index)
                if held is not None:
                    raise PlanValidationError("Place the held object before picking another.")
                self._clear_location(obj, locations, zone_occupancy, slot_occupancy)
                held = obj
            elif skill == "place":
                zone = step["zone"]
                occupant = zone_occupancy.get(zone)
                if occupant not in (None, step["object"]):
                    raise PlanValidationError(
                        f"{zone} is occupied by {occupant}; relocate it before placing."
                    )
                zone_occupancy[zone] = step["object"]
                locations[step["object"]] = (zone, None)
                held = None
            elif skill == "place_temp":
                slot = step["slot"]
                occupant = slot_occupancy.get(slot)
                if occupant not in (None, step["object"]):
                    raise PlanValidationError(
                        f"Temporary slot {slot} is occupied by {occupant}."
                    )
                slot_occupancy[slot] = step["object"]
                locations[step["object"]] = (None, slot)
                held = None

    def _require_visible(self, object_id, objects_state, index=None):
        data = objects_state.get(object_id)
        suffix = f" at step {index + 1}" if index is not None else ""
        if (
            data is None
            or not data.get("visible", False)
            or data.get("position_xyz") is None
        ):
            raise PlanValidationError(
                f"Current scene state cannot locate {object_id!r}{suffix}."
            )

    @staticmethod
    def _clear_location(object_id, locations, zone_occupancy, slot_occupancy):
        zone, slot = locations.get(object_id, (None, None))
        if zone is not None and zone_occupancy.get(zone) == object_id:
            zone_occupancy[zone] = None
        if slot is not None and slot_occupancy.get(slot) == object_id:
            slot_occupancy[slot] = None

    def _first_free_slot(self, occupancy):
        for slot in self.temporary_slot_order:
            if occupancy.get(slot) is None:
                return slot
        return None

    def _check_object(self, obj, index):
        if not isinstance(obj, str) or obj not in self.objects:
            raise PlanValidationError(f"Invalid object at step {index + 1}: {obj!r}.")
        return obj
