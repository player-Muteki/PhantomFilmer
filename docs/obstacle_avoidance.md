# Obstacle Avoidance

PhantomFilmer uses a deterministic local planner for online obstacle avoidance. No remote LLM call is allowed inside the 20 Hz control loop.

## Control Path

`MotionArbiter.decide(desired_command, frame, context)` is the single seam for autonomous motion:

1. Read the cached top/front ToF distance (camera pixels are not inspected for obstacles).
2. Build a distance-only obstacle observation.
3. Select a bounded lateral/yaw action from `CLEAR`, `BRAKING`, `SIDE_STEP_OUT`, `POST_BYPASS_LEFT_TURN`, and `FAILSAFE`. Avoidance never emits forward motion. The separate centre-loss recovery may advance under live ToF protection before avoidance starts.
4. Apply safety limits and send RC through the existing drone adapter.

`follow`, `follow-dry-run`, `console`, and `fixed-demo` all use this path. The fixed-demo route is no longer a bypass: each preset RC segment is treated as a desired command and may be interrupted by obstacle avoidance. While an obstacle is visible, the fixed-demo route timer pauses so the remaining route time is not silently consumed during detour or scan behavior.

## Distance Perception

The RoboMaster TT expansion command `EXT tof?` reports millimetres. The adapter converts valid readings to centimetres; the SDK out-of-range value `8192` is treated as a healthy sensor with no in-range obstacle.

- `distance <= 60 cm`: `BLOCKED`;
- `distance > 60 cm` or out of range: do not increase risk;
- stale/error sample: fail-safe zero command;
- no camera contour, area, risk-zone, target masking, or free-space-sector calculation.

The sensor has no left/right free-space information. After temporal confirmation, the planner moves right for an estimated 100 cm (RC 20 for about 5 seconds), then turns left at yaw RC 12 until telemetry accumulates 90 degrees. It never advances during avoidance. Lateral distance is dead reckoning—not a measured absolute coordinate.

If the last three trustworthy ReID boxes did not touch the left/right 5% margins and the target suddenly disappears, a separate recovery action advances at RC 25 while ToF remains out of range. The first valid ToF return at or below 120 cm stops forward motion immediately and starts the same one-metre-right / 90-degree-left avoidance. ToF detects an object, not a person.

## Safety and Failure Mode

- A blocked candidate brakes before it is confirmed, then only detours after temporal confirmation.
- If the estimated one-metre lateral move does not finish within `bypass_max_sidestep_seconds`, `timeout_action: land` requests landing.
- While the target is lost and front distance is blocked, avoidance takes priority while ReID search is paused; once distance clears, target-loss handling resumes.
- Any detector/planner exception in `MotionArbiter` degrades to a zero RC command.
- JSONL logging is asynchronous and bounded; log failure or queue overflow must never delay control.
