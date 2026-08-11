"""Project entry point for the PhantomFilmer prototype.

Argument parsing and mode dispatch live here; configuration loading, dependency
wiring, and per-mode runners live in the ``app`` package.  The re-exports below
keep the historical ``main`` module API stable for tests and scripts.
"""

import argparse

from app.builder import (
    build_fake_swarm_manager,
    build_obstacle_modules,
    build_safety_manager,
    build_swarm_manager,
    build_system,
    create_drone_adapter,
)
from app.config import (
    CONFIG_PATH,
    FOLLOW_MODES,
    _load_config_without_yaml,
    configured_obstacle_enabled,
    load_config,
    load_runtime_config,
    prompt_obstacle_enabled,
    read_control_interval,
    selected_detector_type,
)
from app.modes import (
    confirm_real_swarm_action,
    print_swarm_batch,
    run_basic_flight_test,
    run_camera,
    run_camera_debug,
    run_console,
    run_fixed_demo,
    run_follow,
    run_follow_dry_run,
    run_follow_test,
    run_safety_test,
    run_status,
    run_swarm_basic_test,
    run_swarm_connect_test,
    run_swarm_hover_test,
    run_swarm_rc_test,
    run_swarm_sim,
    run_swarm_status,
)


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="PhantomFilmer prototype controller")
    parser.add_argument(
        "--mode",
        choices=(
            "demo",
            "status",
            "safety-test",
            "follow-test",
            "follow-dry-run",
            "fixed-demo",
            "basic-flight-test",
            "camera-debug",
            "camera",
            "follow",
            "console",
            "swarm-sim",
            "swarm-status",
            "swarm-connect-test",
            "swarm-basic-test",
            "swarm-hover-test",
            "swarm-rc-test",
        ),
        default="demo",
        help="运行模式：fixed-demo 执行固定航线后进入目标跟随；其余模式保持原有用途。",
    )
    parser.add_argument(
        "--fake",
        action="store_true",
        help="使用模拟无人机，不连接 RoboMaster TT / Tello 真机。",
    )
    return parser.parse_args()


def main() -> int:
    """Run the selected project mode."""
    args = parse_args()
    obstacle_enabled = None
    if args.mode in FOLLOW_MODES:
        config = load_config()
        obstacle_enabled = prompt_obstacle_enabled(
            configured_obstacle_enabled(config)
        )
        if obstacle_enabled is None:
            return 0

    if args.mode == "status":
        return run_status(use_fake=args.fake)
    if args.mode == "safety-test":
        return run_safety_test()
    if args.mode == "follow-test":
        return run_follow_test()
    if args.mode == "follow-dry-run":
        return run_follow_dry_run(
            use_fake=args.fake,
            obstacle_enabled=obstacle_enabled,
        )
    if args.mode == "fixed-demo":
        return run_fixed_demo(
            use_fake=args.fake,
            obstacle_enabled=obstacle_enabled,
        )
    if args.mode == "basic-flight-test":
        return run_basic_flight_test()
    if args.mode == "camera-debug":
        return run_camera_debug(use_fake=args.fake)
    if args.mode == "camera":
        return run_camera(use_fake=args.fake)
    if args.mode == "follow":
        return run_follow(
            use_fake=args.fake,
            obstacle_enabled=obstacle_enabled,
        )
    if args.mode == "console":
        return run_console(
            use_fake=args.fake,
            obstacle_enabled=obstacle_enabled,
        )
    if args.mode == "swarm-sim":
        return run_swarm_sim()
    if args.mode == "swarm-status":
        return run_swarm_status(use_fake=args.fake)
    if args.mode == "swarm-connect-test":
        return run_swarm_connect_test(use_fake=args.fake)
    if args.mode == "swarm-basic-test":
        return run_swarm_basic_test(use_fake=args.fake)
    if args.mode == "swarm-hover-test":
        return run_swarm_hover_test(use_fake=args.fake)
    if args.mode == "swarm-rc-test":
        return run_swarm_rc_test(use_fake=args.fake)

    controller = build_system(use_fake=args.fake)
    controller.describe()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
