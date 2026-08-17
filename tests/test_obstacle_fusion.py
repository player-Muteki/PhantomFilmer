import unittest

from control.obstacle_fusion import (
    InfraredArraySnapshot,
    InfraredObservation,
    ObstacleFusionEngine,
    VisualObstacleAdvisor,
    VisualObstacleCandidate,
    VisualObstacleRisk,
    choose_bypass_direction,
    classify_infrared_snapshot,
    normalized_obstacle_config,
    score_distance,
)
from drone.front_tof import FrontToFSnapshot


class InfraredFusionContractTests(unittest.TestCase):
    def test_classify_infrared_snapshot_marks_blocked_and_safe_cases(self) -> None:
        blocked = classify_infrared_snapshot(
            FrontToFSnapshot(
                distance_cm=40.0,
                status="valid",
                timestamp=1.0,
                age_seconds=0.1,
                sequence=7,
                consecutive_blocked=2,
            ),
            caution_distance_cm=100.0,
            blocked_distance_cm=60.0,
            max_age_seconds=0.8,
        )
        self.assertEqual(blocked.state, "VALID")
        self.assertFalse(blocked.is_safe_to_advance)

        safe = classify_infrared_snapshot(
            FrontToFSnapshot(
                distance_cm=150.0,
                status="valid",
                timestamp=1.0,
                age_seconds=0.1,
                sequence=8,
                consecutive_blocked=0,
            ),
            caution_distance_cm=100.0,
            blocked_distance_cm=60.0,
            max_age_seconds=0.8,
        )
        self.assertEqual(safe.state, "VALID")
        self.assertTrue(safe.is_safe_to_advance)

        stale = classify_infrared_snapshot(
            FrontToFSnapshot(
                distance_cm=None,
                status="stale",
                timestamp=0.0,
                age_seconds=1.1,
                sequence=9,
                consecutive_blocked=0,
            ),
            caution_distance_cm=100.0,
            blocked_distance_cm=60.0,
            max_age_seconds=0.8,
        )
        self.assertEqual(stale.state, "STALE")
        self.assertFalse(stale.is_safe_to_advance)

    def test_visual_advisor_detects_center_object_and_target_only(self) -> None:
        advisor = VisualObstacleAdvisor({})
        frame = type("Frame", (), {"shape": (480, 640, 3)})()

        central = advisor.evaluate(
            frame,
            {
                "visual_objects": [
                    {
                        "bbox_xyxy": (250, 180, 390, 420),
                        "confidence": 0.88,
                        "class_name": "chair",
                        "display_label": "障碍物候选：椅子",
                    }
                ]
            },
        )
        self.assertEqual(central.state, "CENTER_OBJECT")
        self.assertEqual(central.primary_candidate.class_name, "chair")

        target_only = advisor.evaluate(
            frame,
            {"visual_objects": [], "found": True, "bbox": (200, 100, 150, 250)},
        )
        self.assertEqual(target_only.state, "TARGET_PERSON_ONLY")

    def test_fusion_priority_keeps_ir_authority(self) -> None:
        engine = ObstacleFusionEngine({})
        clear_visual = VisualObstacleRisk(
            state="CLEAR",
            candidates=(),
            primary_candidate=None,
            confidence=0.0,
            reason="clear",
        )
        caution_visual = VisualObstacleRisk(
            state="CENTER_OBJECT",
            candidates=(
                VisualObstacleCandidate(
                    "障碍物候选：椅子",
                    "chair",
                    (100, 100, 120, 130),
                    0.9,
                    "center",
                    0.2,
                    0.4,
                    True,
                ),
            ),
            primary_candidate=None,
            confidence=0.8,
            reason="center object",
        )

        safe_ir = InfraredObservation("VALID", 120.0, "valid", 0.1, 5, True, True)
        blocked_ir = InfraredObservation("VALID", 40.0, "valid", 0.1, 6, True, False)
        unknown_ir = InfraredObservation("STALE", None, "stale", 1.2, 7, False, False)

        self.assertEqual(engine.fuse(safe_ir, caution_visual).state, "VISUAL_CAUTION")
        self.assertEqual(engine.fuse(blocked_ir, clear_visual).state, "IR_BLOCKED")
        self.assertEqual(engine.fuse(unknown_ir, clear_visual).state, "IR_UNKNOWN")

    def test_normalized_obstacle_config_keeps_legacy_fields(self) -> None:
        cfg = normalized_obstacle_config(
            {"obstacle": {"front_tof_blocked_distance_cm": 45}}
        )
        self.assertEqual(cfg["front_tof_blocked_distance_cm"], 45)
        self.assertEqual(cfg["front_tof_caution_distance_cm"], 100.0)
        self.assertTrue(cfg["front_tof_enabled"] in {True, False})


    def test_direction_scoring_prefers_more_infrared_clearance(self) -> None:
        clear_visual = VisualObstacleRisk(
            state="CLEAR",
            candidates=(),
            primary_candidate=None,
            confidence=0.0,
            reason="clear",
        )
        infrared = InfraredArraySnapshot(
            front_left_cm=150.0,
            front_center_cm=40.0,
            front_right_cm=50.0,
            left_status="valid",
            center_status="valid",
            right_status="valid",
            sequence=1,
            age_seconds=0.1,
        )

        self.assertGreater(score_distance(150.0), score_distance(50.0))
        self.assertEqual(choose_bypass_direction(infrared, clear_visual), "left")

    def test_out_of_range_is_fresh_clearance_not_unknown(self) -> None:
        observation = classify_infrared_snapshot(
            FrontToFSnapshot(
                distance_cm=None,
                status="out_of_range",
                timestamp=1.0,
                age_seconds=0.1,
                sequence=10,
                consecutive_blocked=0,
            ),
            caution_distance_cm=100.0,
            blocked_distance_cm=60.0,
            max_age_seconds=0.8,
        )
        clear_visual = VisualObstacleRisk(
            state="CLEAR",
            candidates=(),
            primary_candidate=None,
            confidence=0.0,
            reason="clear",
        )

        self.assertEqual(observation.state, "OUT_OF_RANGE")
        self.assertTrue(observation.is_safe_to_advance)
        self.assertEqual(ObstacleFusionEngine({}).fuse(observation, clear_visual).state, "CLEAR")

    def test_visual_approach_requires_sustained_growth(self) -> None:
        advisor = VisualObstacleAdvisor({"visual_assist_approach_frames": 3})
        frame = type("Frame", (), {"shape": (480, 640, 3)})()

        def risk_for_box(box):
            return advisor.evaluate(
                frame,
                {
                    "visual_objects": [
                        {
                            "bbox_xyxy": box,
                            "confidence": 0.9,
                            "class_name": "chair",
                        }
                    ]
                },
            )

        first = risk_for_box((260, 180, 380, 360))
        second = risk_for_box((250, 160, 390, 380))
        third = risk_for_box((235, 130, 405, 410))

        self.assertEqual(first.state, "CENTER_OBJECT")
        self.assertEqual(second.state, "CENTER_OBJECT")
        self.assertEqual(third.state, "APPROACHING_OBJECT")
        self.assertGreater(third.primary_candidate.approach_score, 0.0)

    def test_static_large_visual_box_never_becomes_approaching(self) -> None:
        advisor = VisualObstacleAdvisor({"visual_assist_approach_frames": 3})
        frame = type("Frame", (), {"shape": (480, 640, 3)})()
        target = {
            "visual_objects": [
                {
                    "bbox_xyxy": (100, 20, 540, 460),
                    "confidence": 0.9,
                    "class_name": "chair",
                }
            ]
        }

        states = [advisor.evaluate(frame, target).state for _ in range(4)]

        self.assertEqual(states, ["CENTER_OBJECT"] * 4)

    def test_visual_assist_can_be_disabled(self) -> None:
        advisor = VisualObstacleAdvisor({"visual_assist_enabled": False})
        frame = type("Frame", (), {"shape": (480, 640, 3)})()
        risk = advisor.evaluate(
            frame,
            {
                "visual_objects": [
                    {
                        "bbox_xyxy": (250, 180, 390, 420),
                        "confidence": 0.9,
                        "class_name": "chair",
                    }
                ]
            },
        )

        self.assertEqual(risk.state, "CLEAR")

