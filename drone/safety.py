"""Safety protection logic for the low-altitude drone umbrella prototype."""

from dataclasses import dataclass
from time import monotonic
from typing import Optional, Tuple


@dataclass
class SafetyConfig:
    """Safety thresholds loaded from config.yaml."""

    min_battery_takeoff: int
    low_battery_land: int
    max_height_cm: int
    min_height_cm: int
    max_rc_speed: int
    target_lost_hover_seconds: int
    target_lost_land_seconds: int

    @classmethod
    def from_dict(cls, data: dict) -> "SafetyConfig":
        """Build safety settings from a configuration dictionary."""
        return cls(
            min_battery_takeoff=int(data.get("min_battery_takeoff", 30)),
            low_battery_land=int(data.get("low_battery_land", 20)),
            max_height_cm=int(data.get("max_height_cm", 150)),
            min_height_cm=int(data.get("min_height_cm", 60)),
            max_rc_speed=int(data.get("max_rc_speed", 25)),
            target_lost_hover_seconds=int(
                data.get("lost_target_hover_seconds", data.get("target_lost_hover_seconds", 3))
            ),
            target_lost_land_seconds=int(
                data.get("lost_target_land_seconds", data.get("target_lost_land_seconds", 8))
            ),
        )


class SafetyManager:
    """Central safety manager for battery, height, RC speed, and target loss."""

    def __init__(self, config: SafetyConfig) -> None:
        self.config = config
        self._target_lost_since: Optional[float] = None

    @classmethod
    def from_dict(cls, data: dict) -> "SafetyManager":
        """Create a safety manager directly from config.yaml data."""
        return cls(SafetyConfig.from_dict(data))

    def can_takeoff(self, battery: int) -> bool:
        """Return True only when battery is high enough for takeoff."""
        # 起飞前电量不足时禁止起飞，避免刚起飞就触发低电量降落。
        return battery >= self.config.min_battery_takeoff

    def should_land(self, battery: int) -> bool:
        """Return True when battery is low enough to recommend landing."""
        # 飞行中低于保护电量时建议立即降落。
        return battery < self.config.low_battery_land

    def limit_rc_command(
        self,
        left_right: int,
        forward_backward: int,
        up_down: int,
        yaw: int,
    ) -> Tuple[int, int, int, int]:
        """Clamp all RC command channels to the configured safe speed range."""
        # Tello RC 指令每个通道都限制在 -max_rc_speed 到 +max_rc_speed。
        return (
            self._limit_speed(left_right),
            self._limit_speed(forward_backward),
            self._limit_speed(up_down),
            self._limit_speed(yaw),
        )

    def check_height(self, height: int) -> bool:
        """Return True when height is inside the configured safe range."""
        # 缩比原型限制在低空范围内，过低或过高都视为不安全。
        return self.config.min_height_cm <= height <= self.config.max_height_cm

    def update_target_lost(self, found: bool) -> str:
        """Update target-lost timer and return keep, hover, or land."""
        now = monotonic()
        if found:
            # 目标重新出现后清零丢失计时，继续正常跟随。
            self._target_lost_since = None
            return "keep"

        if self._target_lost_since is None:
            # 第一次发现目标丢失，只开始计时，不立刻降落。
            self._target_lost_since = now
            return "keep"

        lost_seconds = now - self._target_lost_since
        if lost_seconds >= self.config.target_lost_land_seconds:
            return "land"
        if lost_seconds >= self.config.target_lost_hover_seconds:
            return "hover"
        return "keep"

    def _limit_speed(self, value: int) -> int:
        """Clamp one RC speed value."""
        limit = self.config.max_rc_speed
        return max(-limit, min(limit, int(value)))
