"""Project entry point for the PhantomFilmer prototype.

Argument parsing and mode dispatch live here; configuration loading, dependency
wiring, and per-mode runners live in the ``app`` package.  The re-exports below
keep the historical ``main`` module API stable for tests and scripts.
"""

import argparse

from app.builder import (
    build_obstacle_modules,
    build_safety_manager,
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
    run_basic_flight_test,
    run_camera,
    run_camera_debug,
    run_console,
    run_fixed_demo,
    run_follow,
    run_follow_dry_run,
    run_follow_test,
    run_reid_demo,
    run_reid_enroll,
    run_reid_recovery_test,
    run_safety_test,
    run_status,
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
            "reid-enroll",
            "reid-demo",
            "reid-recovery-test",
            "console",
        ),
        default="demo",
        help="运行模式：fixed-demo 执行固定航线后进入目标跟随；其余模式保持原有用途。",
    )
    parser.add_argument(
        "--fake",
        action="store_true",
        help="使用模拟无人机，不连接 RoboMaster TT / Tello 真机。",
    )
    parser.add_argument(
        "--reference-image",
        action="append",
        default=None,
        help="reid-enroll/reid-demo 目标人物照片路径，可重复传入多张。",
    )
    parser.add_argument(
        "--reference-dir",
        default=None,
        help="reid-enroll 参考照片目录，自动读取其中支持的图片。",
    )
    parser.add_argument(
        "--profile",
        default=None,
        help="reid-enroll 创建或 reid-demo 加载的本地人物档案名。",
    )
    parser.add_argument(
        "--overwrite-profile",
        action="store_true",
        help="reid-enroll 显式覆盖同名本地人物档案。",
    )
    parser.add_argument(
        "--capture-reference",
        action="store_true",
        help="reid-enroll/reid-demo 使用电脑摄像头现场拍摄参考照片。",
    )
    parser.add_argument(
        "--reference-camera",
        type=int,
        default=0,
        help="电脑摄像头索引，默认为 0。",
    )
    parser.add_argument(
        "--reference-count",
        type=int,
        default=3,
        help="现场连拍参考照片数，默认为 3。",
    )
    parser.add_argument(
        "--lock-frames",
        type=int,
        default=None,
        help="起飞前需要连续识别成功的帧数。",
    )
    parser.add_argument(
        "--trace",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="开启/关闭无人机行为接口实时跟踪；未指定时运行时询问（回车默认开启）。",
    )
    parser.add_argument(
        "--trace-file",
        default=None,
        help="行为跟踪输出文件路径；默认 logs/trace/<UTC时间戳>.jsonl。",
    )
    return parser.parse_args()


def main() -> int:
    """Run the selected project mode."""
    args = parse_args()
    trace_enabled = getattr(args, "trace", False)
    if trace_enabled is None:
        # 命令行未显式指定 --trace/--no-trace：运行时询问，回车默认开启。
        from app.trace import prompt_trace_enabled

        trace_enabled = prompt_trace_enabled(default_enabled=True)
        if trace_enabled is None:
            return 0
    if trace_enabled:
        from app.trace import enable_trace

        trace_logger = enable_trace(getattr(args, "trace_file", None))
        print(f"行为跟踪已开启：{trace_logger.path}")

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
    if args.mode == "reid-enroll":
        return run_reid_enroll(
            profile_name=args.profile,
            reference_images=args.reference_image,
            reference_directory=args.reference_dir,
            capture_reference=args.capture_reference,
            reference_camera=args.reference_camera,
            reference_count=args.reference_count,
            overwrite_profile=args.overwrite_profile,
        )
    if args.mode == "reid-demo":
        return run_reid_demo(
            use_fake=args.fake,
            obstacle_enabled=obstacle_enabled,
            reference_images=args.reference_image,
            profile_name=args.profile,
            capture_reference=args.capture_reference,
            reference_camera=args.reference_camera,
            reference_count=args.reference_count,
            lock_frames=args.lock_frames,
        )
    if args.mode == "reid-recovery-test":
        if args.fake:
            print("reid-recovery-test 是真机专项模式，不接受 --fake；已取消。")
            return 1
        return run_reid_recovery_test(
            profile_name=args.profile,
            reference_images=args.reference_image,
            capture_reference=args.capture_reference,
            reference_camera=args.reference_camera,
            reference_count=args.reference_count,
        )
    if args.mode == "console":
        return run_console(
            use_fake=args.fake,
            obstacle_enabled=obstacle_enabled,
        )

    controller = build_system(use_fake=args.fake)
    controller.describe()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
