import unittest

from ur3_llm_control.scene_state import build_world_state


class SceneStateTests(unittest.TestCase):
    def test_configured_scene_poses_are_valid_for_static_assignment(self):
        scene = {
            "world_frame": "base_link",
            "zones": {
                "zone_a": {
                    "center_xyz": [0.32, -0.16, 0.07],
                    "size_xy": [0.14, 0.14],
                }
            },
            "objects": {
                "red_cube": {"position_xyz": [0.32, -0.16, 0.09]},
                "blue_cube": {"position_xyz": [0.44, 0.0, 0.09]},
            },
            "temporary_slots": {},
        }
        state = build_world_state(scene)
        self.assertTrue(state["objects"]["red_cube"]["visible"])
        self.assertEqual(state["objects"]["red_cube"]["zone_id"], "zone_a")
        self.assertEqual(state["zones"]["zone_a"]["occupied_by"], "red_cube")
        self.assertTrue(state["objects"]["blue_cube"]["visible"])


if __name__ == "__main__":
    unittest.main()
