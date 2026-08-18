"""Feature-SDK adapters that wrap existing controllers into the kernel protocol.

Each adapter owns one autonomous capability (follow / obstacle / search / safety)
and proposes a bounded FeatureProposal per tick. Adapters never read scheduling
state — the kernel decides who runs via the arbitration table.
"""

from control.features.follow import FollowFeature
from control.features.manual import ManualFeature
from control.features.obstacle import ObstacleFeature
from control.features.safety import SafetyFeature
from control.features.search import SearchFeature


def build_features(
    *,
    follow_controller,
    safety_manager,
    target_search=None,
    search_enabled=False,
    motion_arbiter=None,
    manual_controller=None,
    mode_label="FOLLOW",
):
    """Assemble the feature registry for one session from existing controllers.

    Follow and safety are always present; search is registered only when ReID
    search is enabled; obstacle is registered only when a motion arbiter exists.
    """
    features = {
        "follow": FollowFeature(
            follow_controller=follow_controller,
            safety_manager=safety_manager,
            target_search=target_search,
            search_enabled=search_enabled,
        ),
        "safety": SafetyFeature(
            safety_manager=safety_manager,
            follow_controller=follow_controller,
        ),
    }
    if search_enabled and target_search is not None:
        features["search"] = SearchFeature(
            target_search=target_search,
            safety_manager=safety_manager,
            follow_controller=follow_controller,
        )
    if motion_arbiter is not None:
        features["obstacle"] = ObstacleFeature(
            arbiter=motion_arbiter,
            mode_label=mode_label,
        )
    if manual_controller is not None and manual_controller.config.enabled:
        features["manual"] = ManualFeature(manual_controller)
    return features


__all__ = [
    "FollowFeature",
    "ManualFeature",
    "ObstacleFeature",
    "SafetyFeature",
    "SearchFeature",
    "build_features",
]
