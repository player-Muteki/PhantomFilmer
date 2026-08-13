# Obstacle Avoidance

PhantomFilmer uses a deterministic local planner for online obstacle avoidance. No remote LLM call is allowed inside the 20 Hz control loop.

## Control Path

`MotionArbiter.decide(desired_command, frame, context)` is the single seam for autonomous motion:

1. Read the cached top/front ToF distance (camera pixels are not inspected for obstacles).
2. Build a distance-only obstacle observation.
3. Select a bounded local action from `CLEAR`, `BRAKING`, `SIDE_STEP_OUT`, `FORWARD_120CM`, `SIDE_STEP_RETURN`, and `FAILSAFE`.
4. Apply safety limits and send RC through the existing drone adapter.

`follow`, `follow-dry-run`, `console`, and `fixed-demo` all use this path. The fixed-demo route is no longer a bypass: each preset RC segment is treated as a desired command and may be interrupted by obstacle avoidance. While an obstacle is visible, the fixed-demo route timer pauses so the remaining route time is not silently consumed during detour or scan behavior.

## Distance Perception

The RoboMaster TT expansion command `EXT tof?` reports millimetres. The adapter converts valid readings to centimetres; the SDK out-of-range value `8192` is treated as a healthy sensor with no in-range obstacle.

- `distance <= 60 cm`: `BLOCKED`;
- `distance > 60 cm` or out of range: do not increase risk;
- stale/error sample: fail-safe zero command;
- no camera contour, area, risk-zone, target masking, or free-space-sector calculation.

The sensor has no left/right free-space information. After temporal confirmation, the planner moves in the configured fixed direction until front distance exceeds 70 cm, advances approximately 1.2 m, then reverses the lateral command for the same active duration. The return is dead reckoning—not a measured return to an absolute coordinate.

## Safety and Failure Mode

- A blocked candidate brakes before it is confirmed, then only detours after temporal confirmation.
- If the lateral search does not obtain more than 70 cm clearance within `bypass_max_sidestep_seconds`, `timeout_action: land` requests landing.
- While the target is lost and front distance is blocked, avoidance takes priority while ReID search is paused; once distance clears, target-loss handling resumes.
- Any detector/planner exception in `MotionArbiter` degrades to a zero RC command.
- JSONL logging is asynchronous and bounded; log failure or queue overflow must never delay control.
