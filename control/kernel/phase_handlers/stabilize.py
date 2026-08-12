"""STABILIZING handler: post-takeoff stabilization window.

Keeps ReID detection active (when applicable) for a fixed duration with no RC
emission, then advances to height verification.
"""

from typing import Any, Optional

from control.kernel.phases import KernelPhase


class StabilizeHandler:
    """Owns the takeoff-stabilization phase (S2.2)."""

    phase = KernelPhase.STABILIZING

    def run(self, session: Any, ctx: Any) -> Optional[KernelPhase]:
        if not session._wait_for_takeoff_stabilization(2.0):
            return None
        return KernelPhase.HEIGHT_VERIFY
