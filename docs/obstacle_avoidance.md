# Obstacle Avoidance

PhantomFilmer uses a deterministic local planner for online obstacle avoidance. No remote LLM call is allowed inside the 20 Hz control loop.

## Control Path

`MotionArbiter.decide(desired_command, frame, context)` is the single seam for autonomous motion:

1. Read one camera frame.
2. Build target and obstacle observations.
3. Select a bounded local action from `CLEAR`, `CAUTION`, `BRAKING`, `DETOUR_LEFT`, `DETOUR_RIGHT`, `SCAN`, `RECOVERING`, and `FAILSAFE`.
4. Apply safety limits and send RC through the existing drone adapter.

`follow`, `follow-dry-run`, `console`, and `fixed-demo` all use this path. The fixed-demo route is no longer a bypass: each preset RC segment is treated as a desired command and may be interrupted by obstacle avoidance. While an obstacle is visible, the fixed-demo route timer pauses so the remaining route time is not silently consumed during detour or scan behavior.

## Perception

The detector keeps legacy fields such as `found`, `state`, `center`, `bbox`, `area_ratio`, `side`, and `risk_zone`, and adds:

- multiple `ObstacleCandidate` objects per frame;
- normalized coordinates suitable for LLM-friendly logs;
- five horizontal free-space sector scores;
- consecutive found/clear frame counters;
- motion score and estimated time-to-collision when growth data is available.

The implementation is intentionally conservative: no metric depth is claimed, no global map is built, and a blocked obstacle never allows positive forward motion.

## Safety and Failure Mode

- A blocked candidate brakes before it is confirmed, then only detours after temporal confirmation.
- When both lateral sectors are unavailable, the planner scans at low yaw speed without moving forward.
- If no safe local route is found within `max_avoidance_seconds`, `timeout_action: land` requests landing.
- While the target is lost and a blocking obstacle is visible, avoidance takes priority: the planner actively detours (even though the desired command is all-zero) while ReID search is paused; once the obstacle clears, target-loss handling resumes and search re-triggers. Forward motion is still forced to zero throughout.
- Any detector/planner exception in `MotionArbiter` degrades to a zero RC command.
- JSONL logging is asynchronous and bounded; log failure or queue overflow must never delay control.
