import unittest

from ur3_llm_control.task_validator import PlanValidationError, TaskValidator


OBJECTS = {"red_cube", "yellow_cube", "blue_cube", "green_cube", "purple_cube"}
ZONES = {"zone_a", "zone_b", "zone_c"}
SLOTS = ["temp_1", "temp_2"]


def state_with_blue_in_b():
    objects = {
        object_id: {
            "position_xyz": [0.7, 0.3, 0.34],
            "visible": True,
            "zone_id": None,
            "temporary_slot": None,
        }
        for object_id in OBJECTS
    }
    objects["red_cube"].update(position_xyz=[0.5, -0.22, 0.34], zone_id="zone_a")
    objects["blue_cube"].update(position_xyz=[0.5, 0.0, 0.34], zone_id="zone_b")
    objects["yellow_cube"].update(position_xyz=[0.5, 0.22, 0.34], zone_id="zone_c")
    return {
        "objects": objects,
        "zones": {
            "zone_a": {"occupied": True, "occupied_by": "red_cube"},
            "zone_b": {"occupied": True, "occupied_by": "blue_cube"},
            "zone_c": {"occupied": True, "occupied_by": "yellow_cube"},
        },
        "temporary_slots": {
            "temp_1": {"occupied": False, "occupied_by": None},
            "temp_2": {"occupied": False, "occupied_by": None},
        },
    }


class TaskValidatorTests(unittest.TestCase):
    def setUp(self):
        self.validator = TaskValidator(OBJECTS, ZONES, SLOTS)

    def test_accepts_basic_assignment_plan(self):
        result = self.validator.validate({"plan": [
            {"skill": "pick", "object": "red_cube"},
            {"skill": "place", "object": "red_cube", "zone": "zone_b"},
            {"skill": "home"},
        ]})
        self.assertEqual(len(result), 3)

    def test_rejects_unknown_skill_object_and_zone(self):
        with self.assertRaises(PlanValidationError):
            self.validator.validate({"plan": [
                {"skill": "joint_trajectory", "positions": [0.0]},
                {"skill": "home"},
            ]})
        with self.assertRaises(PlanValidationError):
            self.validator.validate({"plan": [
                {"skill": "pick", "object": "orange_cube"},
                {"skill": "place", "object": "orange_cube", "zone": "zone_a"},
                {"skill": "home"},
            ]})
        with self.assertRaises(PlanValidationError):
            self.validator.validate({"plan": [
                {"skill": "pick", "object": "red_cube"},
                {"skill": "place", "object": "red_cube", "zone": "zone_d"},
                {"skill": "home"},
            ]})

    def test_automatically_moves_zone_blocker_to_camera_verified_slot(self):
        state = state_with_blue_in_b()
        raw = self.validator.validate({"plan": [
            {"skill": "pick", "object": "red_cube"},
            {"skill": "place", "object": "red_cube", "zone": "zone_b"},
            {"skill": "home"},
        ]})
        resolved = self.validator.resolve_occupied_zones(raw, state)
        self.assertEqual(resolved, [
            {"skill": "pick", "object": "blue_cube"},
            {"skill": "place_temp", "object": "blue_cube", "slot": "temp_1"},
            {"skill": "pick", "object": "red_cube"},
            {"skill": "place", "object": "red_cube", "zone": "zone_b"},
            {"skill": "home"},
        ])
        self.assertEqual(
            self.validator.validate({"plan": resolved}, world_state=state),
            resolved,
        )

    def test_rejects_if_no_temporary_slot_can_be_used(self):
        state = state_with_blue_in_b()
        state["temporary_slots"]["temp_1"]["occupied_by"] = "green_cube"
        state["temporary_slots"]["temp_1"]["occupied"] = True
        state["temporary_slots"]["temp_2"]["occupied_by"] = "purple_cube"
        state["temporary_slots"]["temp_2"]["occupied"] = True
        with self.assertRaises(PlanValidationError):
            plan = self.validator.validate({"plan": [
                {"skill": "pick", "object": "red_cube"},
                {"skill": "place", "object": "red_cube", "zone": "zone_b"},
                {"skill": "home"},
            ]})
            self.validator.resolve_occupied_zones(plan, state)

    def test_refuses_execution_when_camera_cannot_find_requested_object(self):
        state = state_with_blue_in_b()
        state["objects"]["red_cube"]["visible"] = False
        with self.assertRaises(PlanValidationError):
            self.validator.validate({"plan": [
                {"skill": "pick", "object": "red_cube"},
                {"skill": "place", "object": "red_cube", "zone": "zone_a"},
                {"skill": "home"},
            ]}, world_state=state)


if __name__ == "__main__":
    unittest.main()
