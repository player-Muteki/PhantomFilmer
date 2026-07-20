"""Predefined low-speed maneuver used before the normal follow task."""

from dataclasses import dataclass
from time import monotonic, sleep
from typing import Callable, Optional, Sequence

from control.follow_control import RCCommand


@dataclass(frozen=True)
class FixedDemoStep:
    """One timed RC segment followed by a zero-output settling period."""

    name: str
    command: RCCommand
    duration_seconds: float
    settle_seconds: float


@dataclass(frozen=True)
class FixedDemoProgress:
    """Progress information exposed to the session UI and abort handler."""

    step_index: int
    step_count: int
    step: FixedDemoStep
    elapsed_seconds: float
    settling: bool


FIXED_DEMO_STEPS = (
    FixedDemoStep("向前飞行", RCCommand(forward_backward=24), 2.0, 0.5),
    FixedDemoStep("向左平移", RCCommand(left_right=-16), 3.0, 0.5),
    FixedDemoStep("再次向前", RCCommand(forward_backward=12), 1.0, 0.5),
    FixedDemoStep("向右平移", RCCommand(left_right=16), 3.0, 1.0),
)


class FixedDemoManeuver:
    """Execute the fixed route while continuously refreshing RC output."""

    def __init__(
        self,
        steps: Sequence[FixedDemoStep] = FIXED_DEMO_STEPS,
        control_interval: float = 0.05,
        clock: Callable[[], float] = monotonic,
        sleep_fn: Callable[[float], None] = sleep,
    ) -> None:
        self.steps = tuple(steps)
        self.control_interval = max(0.02, min(0.2, float(control_interval)))
        self._clock = clock
        self._sleep = sleep_fn

    def run(
        self,
        send_command: Callable[[RCCommand], None],
        should_abort: Callable[[], bool],
        on_progress: Optional[Callable[[FixedDemoProgress], bool]] = None,
    ) -> bool:
        """Run all segments, returning False when the caller requests an abort."""
        zero = RCCommand()
        try:
            for index, step in enumerate(self.steps, start=1):
                print(
                    f"固定演示 {index}/{len(self.steps)}：{step.name} "
                    f"{step.duration_seconds:.1f} 秒。"
                )
                if not self._run_period(
                    duration=step.duration_seconds,
                    command=step.command,
                    step=step,
                    step_index=index,
                    settling=False,
                    send_command=send_command,
                    should_abort=should_abort,
                    on_progress=on_progress,
                ):
                    return False

                send_command(zero)
                if not self._run_period(
                    duration=step.settle_seconds,
                    command=zero,
                    step=step,
                    step_index=index,
                    settling=True,
                    send_command=send_command,
                    should_abort=should_abort,
                    on_progress=on_progress,
                ):
                    return False
            return True
        finally:
            send_command(zero)

    def _run_period(
        self,
        duration: float,
        command: RCCommand,
        step: FixedDemoStep,
        step_index: int,
        settling: bool,
        send_command: Callable[[RCCommand], None],
        should_abort: Callable[[], bool],
        on_progress: Optional[Callable[[FixedDemoProgress], bool]],
    ) -> bool:
        """Refresh one command until its deadline or an abort request."""
        started_at = self._clock()
        while True:
            if should_abort():
                return False

            elapsed = self._clock() - started_at
            if elapsed >= duration:
                return True

            send_command(command)
            progress = FixedDemoProgress(
                step_index=step_index,
                step_count=len(self.steps),
                step=step,
                elapsed_seconds=elapsed,
                settling=settling,
            )
            if on_progress is not None and not on_progress(progress):
                return False

            remaining = duration - (self._clock() - started_at)
            if remaining > 0:
                self._sleep(min(self.control_interval, remaining))
