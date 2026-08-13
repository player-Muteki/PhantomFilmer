"""Deterministic tests for target-memory based occlusion recovery."""

import unittest

import numpy as np

from control.features.occlusion import OcclusionRecoveryConfig, OcclusionRecoveryFeature
from control.follow_control import RCCommand
from control.kernel.arbitration import ArbitrationEngine
from control.kernel.features import ArbitrationContext, FeatureProposal
from control.kernel.phases import KernelPhase
from vision.obstacle_detect import ObstacleResult


def context(found: bool, *, bbox=(280, 160, 80, 180), now=0.0, yaw_deg=None):
    target = {
        "found": found,
        "is_predicted": False,
        "ambiguous": False,
        "center": (bbox[0] + bbox[2] // 2, bbox[1] + bbox[3] // 2) if found else None,
        "bbox": bbox if found else None,
        "area": bbox[2] * bbox[3] if found else 0,
    }
    return ArbitrationContext(
        phase=KernelPhase.FOLLOW,
        target_result=target,
        frame=np.zeros((480, 640, 3), dtype=np.uint8),
        frame_width=640,
        frame_height=480,
        yaw_deg=yaw_deg,
        now=now,
    )


def recovery_config(**overrides):
    return {
        "occlusion_recovery": {
            "enabled": True,
            "initial_lock_frames": 2,
            "initial_scan_yaw_speed": 9,
            "initial_scan_degrees": 360.0,
            "initial_scan_fallback_seconds": 36.0,
            "initial_acquire_timeout_seconds": 20.0,
            "occlusion_check_seconds": 0.8,
            "occlusion_max_age_seconds": 1.5,
            "occluder_min_area_ratio": 0.02,
            "occlusion_overlap_ratio": 0.25,
            "occluder_iou_threshold": 0.15,
            "occlusion_confirm_frames": 2,
            "lateral_speed": 10,
            "lateral_pulse_seconds": 0.4,
            "settle_seconds": 0.3,
            "max_lateral_pulses": 2,
            "local_scan_degrees": 20.0,
            "local_scan_yaw_speed": 12,
            "local_scan_hold_seconds": 0.3,
            "local_scan_fallback_seconds": 2.0,
            "local_scan_return_tolerance_degrees": 3.0,
            "reacquire_frames": 2,
            "min_free_space_score": 0.35,
            **overrides,
        }
    }


def blocked(bbox=(270, 150, 120, 200), *, left=0.2, right=0.9):
    return ObstacleResult(
        found=True,
        state="BLOCKED",
        bbox=bbox,
        center=(bbox[0] + bbox[2] // 2, bbox[1] + bbox[3] // 2),
        side="center",
        free_space={
            "far_left": left,
            "left": left,
            "center": 0.0,
            "right": right,
            "far_right": right,
        },
        consecutive_found_frames=3,
    )


def caution(
    bbox=(270, 150, 120, 200),
    *,
    area_ratio=0.05,
    left=0.2,
    right=0.9,
):
    result = blocked(bbox=bbox, left=left, right=right)
    result.state = "CAUTION"
    result.area_ratio = area_ratio
    return result


class OcclusionRecoveryFeatureTestCase(unittest.TestCase):
    def test_production_defaults_request_single_one_meter_peek(self):
        config = OcclusionRecoveryConfig.from_config({"occlusion_recovery": {}})

        self.assertEqual(config.initial_scan_yaw_speed, 20)
        self.assertEqual(config.initial_scan_fallback_seconds, 18.0)
        self.assertEqual(config.lateral_speed, 25)
        self.assertEqual(config.lateral_pulse_seconds, 4.0)
        self.assertEqual(config.max_lateral_pulses, 1)
        self.assertEqual(config.local_scan_degrees, 30.0)
        self.assertEqual(config.local_scan_yaw_speed, 20)
        self.assertEqual(config.local_scan_fallback_seconds, 1.5)

    def test_never_locked_target_allows_yaw_only_even_with_obstacle(self):
        feature = OcclusionRecoveryFeature(config=recovery_config())

        first = feature.propose(context(False, now=0.0), blocked(), 0.0)
        second = feature.propose(context(False, now=1.1), blocked(), 1.1)

        self.assertEqual(first.command.as_tuple(), (0, 0, 0, 9))
        self.assertEqual(second.command.as_tuple(), (0, 0, 0, 9))
        self.assertFalse(feature.ever_target_locked)
        self.assertEqual(feature.state, "INITIAL_ACQUIRE")

    def test_initial_acquisition_accumulates_one_turn_across_yaw_wrap(self):
        feature = OcclusionRecoveryFeature(
            config=recovery_config(
                initial_scan_degrees=40.0,
                initial_acquire_timeout_seconds=10.0,
            )
        )

        first = feature.propose(
            context(False, now=0.0, yaw_deg=170), None, 0.0
        )
        second = feature.propose(
            context(False, now=0.1, yaw_deg=-170), None, 0.1
        )
        complete = feature.propose(
            context(False, now=0.2, yaw_deg=-150), None, 0.2
        )

        self.assertEqual(first.command.yaw, 9)
        self.assertEqual(second.command.yaw, 9)
        self.assertEqual(complete.state, "INITIAL_SCAN_COMPLETE")
        self.assertTrue(complete.requires_landing)
        self.assertEqual(complete.command.as_tuple(), (0, 0, 0, 0))

    def test_initial_acquisition_timeout_requests_deferred_landing(self):
        feature = OcclusionRecoveryFeature(
            config=recovery_config(initial_acquire_timeout_seconds=2.0)
        )
        feature.propose(context(False, now=0.0), blocked(), 0.0)

        timed_out = feature.propose(context(False, now=2.1), blocked(), 2.1)

        self.assertEqual(timed_out.command.as_tuple(), (0, 0, 0, 0))
        self.assertTrue(timed_out.requires_landing)
        self.assertEqual(timed_out.landing_kind, "target_lost")

        outcome = object.__new__(ArbitrationEngine)._finish(timed_out)
        self.assertFalse(outcome.requires_landing)
        self.assertTrue(outcome.lost_land)

    def test_only_persistent_overlapping_blocker_starts_lateral_peek(self):
        feature = OcclusionRecoveryFeature(config=recovery_config())
        self.assertIsNotNone(feature.propose(context(True, now=0.0), None, 0.0))
        self.assertIsNone(feature.propose(context(True, now=0.1), None, 0.1))

        confirming = feature.propose(context(False, now=0.2), blocked(), 0.2)
        bypass = feature.propose(context(False, now=0.3), blocked(), 0.3)

        self.assertEqual(confirming.command.as_tuple(), (0, 0, 0, 0))
        self.assertEqual(confirming.state, "OCCLUSION_CHECK")
        self.assertEqual(bypass.state, "OCCLUSION_BYPASS")
        self.assertEqual(bypass.command.as_tuple(), (10, 0, 0, 0))
        self.assertIn("overlap=", bypass.reason)

    def test_target_linked_caution_board_starts_lateral_peek(self):
        feature = OcclusionRecoveryFeature(config=recovery_config())
        feature.propose(context(True, now=0.0), None, 0.0)
        feature.propose(context(True, now=0.1), None, 0.1)

        confirming = feature.propose(context(False, now=0.2), caution(), 0.2)
        bypass = feature.propose(context(False, now=0.3), caution(), 0.3)

        self.assertEqual(confirming.state, "OCCLUSION_CHECK")
        self.assertEqual(bypass.state, "OCCLUSION_BYPASS")
        self.assertEqual(bypass.command.as_tuple(), (10, 0, 0, 0))

    def test_small_caution_noise_does_not_start_lateral_peek(self):
        feature = OcclusionRecoveryFeature(config=recovery_config())
        feature.propose(context(True, now=0.0), None, 0.0)
        feature.propose(context(True, now=0.1), None, 0.1)
        noise = caution(area_ratio=0.01)

        first = feature.propose(context(False, now=0.2), noise, 0.2)
        second = feature.propose(context(False, now=0.3), noise, 0.3)

        self.assertEqual(first.command.as_tuple(), (0, 0, 0, 0))
        self.assertEqual(second.command.as_tuple(), (0, 0, 0, 0))
        self.assertEqual(feature.state, "OCCLUSION_CHECK")

    def test_unrelated_obstacle_never_starts_active_bypass(self):
        feature = OcclusionRecoveryFeature(config=recovery_config())
        feature.propose(context(True, now=0.0), None, 0.0)
        feature.propose(context(True, now=0.1), None, 0.1)
        unrelated = blocked(bbox=(30, 30, 60, 80))

        hold = feature.propose(context(False, now=0.2), unrelated, 0.2)
        fallback = feature.propose(context(False, now=1.1), unrelated, 1.1)

        self.assertEqual(hold.command.as_tuple(), (0, 0, 0, 0))
        self.assertIsNone(fallback)
        self.assertEqual(feature.state, "FALLBACK_SEARCH")

    def test_reacquire_requires_consecutive_identity_frames(self):
        feature = OcclusionRecoveryFeature(config=recovery_config())
        feature.propose(context(True, now=0.0), None, 0.0)
        feature.propose(context(True, now=0.1), None, 0.1)
        feature.propose(context(False, now=0.2), blocked(), 0.2)
        feature.propose(context(False, now=0.3), blocked(), 0.3)

        verify = feature.propose(context(True, now=0.4), None, 0.4)
        released = feature.propose(context(True, now=0.5), None, 0.5)

        self.assertEqual(verify.state, "REACQUIRE_VERIFY")
        self.assertEqual(verify.command.as_tuple(), (0, 0, 0, 0))
        self.assertIsNone(released)
        self.assertEqual(feature.state, "TRACKING")

    def test_single_meter_limit_prevents_second_peek_after_failed_verify(self):
        feature = OcclusionRecoveryFeature(
            config=recovery_config(max_lateral_pulses=1)
        )
        feature.propose(context(True, now=0.0), None, 0.0)
        feature.propose(context(True, now=0.1), None, 0.1)
        feature.propose(context(False, now=0.2), blocked(), 0.2)
        first_peek = feature.propose(context(False, now=0.3), blocked(), 0.3)

        verify = feature.propose(context(True, now=0.4), None, 0.4)
        failed_verify = feature.propose(context(False, now=0.5), blocked(), 0.5)

        self.assertNotEqual(first_peek.command.left_right, 0)
        self.assertEqual(verify.state, "REACQUIRE_VERIFY")
        self.assertIsNone(failed_verify)
        self.assertEqual(feature.state, "FALLBACK_SEARCH")

    def test_lateral_peek_then_scans_opposite_and_returns_before_next_peek(self):
        feature = OcclusionRecoveryFeature(config=recovery_config())
        feature.propose(context(True, now=0.0, yaw_deg=0), None, 0.0)
        feature.propose(context(True, now=0.1, yaw_deg=0), None, 0.1)
        feature.propose(context(False, now=0.2, yaw_deg=0), blocked(), 0.2)
        feature.propose(context(False, now=0.3, yaw_deg=0), blocked(), 0.3)

        settle = feature.propose(
            context(False, now=0.8, yaw_deg=0), blocked(), 0.8
        )
        scan_out = feature.propose(
            context(False, now=1.4, yaw_deg=0), blocked(), 1.4
        )
        scan_edge = feature.propose(
            context(False, now=1.5, yaw_deg=-20), blocked(), 1.5
        )
        scan_return = feature.propose(
            context(False, now=1.9, yaw_deg=-20), blocked(), 1.9
        )
        next_peek = feature.propose(
            context(False, now=2.0, yaw_deg=0), blocked(), 2.0
        )

        self.assertEqual(settle.state, "OCCLUSION_SETTLE")
        self.assertEqual(scan_out.command.as_tuple(), (0, 0, 0, -12))
        self.assertEqual(scan_edge.state, "OCCLUSION_LOCAL_SCAN_HOLD")
        self.assertEqual(scan_edge.command.as_tuple(), (0, 0, 0, 0))
        self.assertEqual(scan_return.command.as_tuple(), (0, 0, 0, 12))
        self.assertEqual(next_peek.command.as_tuple(), (10, 0, 0, 0))

    def test_no_free_side_refuses_bypass(self):
        feature = OcclusionRecoveryFeature(config=recovery_config())
        feature.propose(context(True, now=0.0), None, 0.0)
        feature.propose(context(True, now=0.1), None, 0.1)
        no_route = blocked(left=0.1, right=0.1)

        feature.propose(context(False, now=0.2), no_route, 0.2)
        proposal = feature.propose(context(False, now=0.3), no_route, 0.3)

        self.assertEqual(proposal.state, "OCCLUSION_NO_SAFE_ROUTE")
        self.assertEqual(proposal.command.as_tuple(), (0, 0, 0, 0))

        waiting = feature.propose(context(False, now=2.0), no_route, 2.0)
        landing = feature.propose(context(False, now=3.4), no_route, 3.4)

        self.assertEqual(waiting.state, "OCCLUSION_NO_SAFE_ROUTE")
        self.assertEqual(waiting.command.as_tuple(), (0, 0, 0, 0))
        self.assertEqual(landing.state, "OCCLUSION_NO_SAFE_ROUTE_LANDING")
        self.assertTrue(landing.requires_landing)
        self.assertEqual(landing.landing_kind, "target_lost")

    def test_safe_side_appearing_after_hold_starts_bypass(self):
        feature = OcclusionRecoveryFeature(config=recovery_config())
        feature.propose(context(True, now=0.0), None, 0.0)
        feature.propose(context(True, now=0.1), None, 0.1)
        no_route = blocked(left=0.1, right=0.1)

        feature.propose(context(False, now=0.2), no_route, 0.2)
        feature.propose(context(False, now=0.3), no_route, 0.3)
        bypass = feature.propose(context(False, now=0.5), blocked(), 0.5)

        self.assertEqual(bypass.state, "OCCLUSION_BYPASS")
        self.assertEqual(bypass.command.as_tuple(), (10, 0, 0, 0))

    def test_unassociated_obstacle_vetoes_search_translation_but_keeps_yaw(self):
        search = FeatureProposal(
            RCCommand(left_right=8, forward_backward=-12, up_down=10, yaw=15),
            state="SEARCH",
            reason="bounded search",
            feature="search",
        )

        gated = ArbitrationEngine._gate_lost_search(search, blocked())

        self.assertEqual(gated.command.as_tuple(), (0, 0, 0, 15))
        self.assertIn("obstacle veto", gated.reason)


if __name__ == "__main__":
    unittest.main()
