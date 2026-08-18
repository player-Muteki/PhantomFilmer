"""Per-mode CLI runners for PhantomFilmer."""

from pathlib import Path
from time import monotonic, sleep
from typing import Optional, Sequence

from app.builder import (
    build_obstacle_modules,
    build_safety_manager,
    build_system,
    create_drone_adapter,
)
from app.config import load_config, load_runtime_config, read_control_interval
from control.features import build_features
from control.fixed_demo import FixedDemoManeuver
from control.follow_control import FollowController
from control.follow_session import FollowSession
from control.kernel.arbitration import ArbitrationEngine
from control.kernel.features import ArbitrationContext
from control.kernel.phases import KernelPhase
from drone.front_tof import FrontToFMonitor
from drone.safety import SafetyManager
from drone.tello_adapter import TelloDroneAdapter
from vision.camera import CameraStream
from vision.detector_factory import create_detector
from vision.reid_enrollment import (
    build_reid_runtime_config,
    collect_reference_images,
    validate_reference_directory,
)
from vision.reid_profiles import save_reid_profile
from vision.target_detect import TargetDetector


def run_status(use_fake: bool = False) -> int:
    """Connect to the drone and print battery and height status."""
    drone = create_drone_adapter(use_fake)
    try:
        print("正在连接模拟无人机..." if use_fake else "正在连接 RoboMaster TT / Tello...")
        drone.connect()
        battery = drone.get_battery()
        height = drone.get_height()
        print("无人机状态：")
        print(f"- 电量：{battery}%")
        print(f"- TOF 离地高度：{height} cm")
        estimated_height_reader = getattr(drone, "get_estimated_height", None)
        if callable(estimated_height_reader):
            print(f"- 飞控原始 h（仅诊断）：{estimated_height_reader()} cm")
        return 0
    except RuntimeError as exc:
        print(str(exc))
        print("请先连接 RoboMaster TT / Tello 的 Wi-Fi。")
        return 1
    finally:
        drone.stop()


def run_web(obstacle_enabled: Optional[bool] = None) -> int:
    """Run the real-aircraft-only local browser control surface."""
    try:
        import uvicorn
    except ModuleNotFoundError:
        print("缺少 WebUI 依赖：请先安装 requirements.txt。")
        return 1

    from web.server import create_app

    config = load_config()
    web_config = config.get("web", {})
    if not isinstance(web_config, dict):
        web_config = {}
    host = str(web_config.get("host", "0.0.0.0"))
    port = int(web_config.get("port", 8080))
    app = create_app(obstacle_enabled=obstacle_enabled)
    print(f"WebUI 启动中: http://localhost:{port}")
    print("WebUI 仅允许连接真实 RoboMaster TT / Tello，未验证前不会开启图传。")
    uvicorn.run(app, host=host, port=port)
    return 0


def run_connection_test(use_fake: bool = False) -> int:
    """Verify SDK command/response communication without camera or flight."""
    drone = create_drone_adapter(use_fake)
    try:
        print("正在运行无人机连接测试（不会起飞、不会开启摄像头）...")
        drone.connect()
        battery = getattr(drone, "last_connection_battery", None)
        if battery is None:
            battery = drone.get_battery()
        print(f"连接测试成功：battery={battery}%")
        return 0
    except RuntimeError as exc:
        print(f"连接测试失败：{exc}")
        return 1
    finally:
        drone.stop()


def run_console(
    use_fake: bool = False,
    obstacle_enabled: Optional[bool] = None,
) -> int:
    """Run the interactive rule-based Console scheduler."""
    controller = build_system(
        use_fake=use_fake,
        obstacle_enabled=obstacle_enabled,
    )
    try:
        print("正在连接模拟无人机..." if use_fake else "正在连接 RoboMaster TT / Tello...")
        controller.tools.connect()
        return controller.run()
    except RuntimeError as exc:
        print(str(exc))
        if not use_fake:
            print("请先连接 RoboMaster TT / Tello 的 Wi-Fi。")
        return 1
    finally:
        if controller.tools.connected:
            controller.tools.close()


def run_camera(use_fake: bool = False) -> int:
    """Connect to the drone, show camera stream, and detect red targets."""
    try:
        import cv2
    except ModuleNotFoundError:
        print("缺少 opencv-contrib-python 依赖：请先安装 requirements.txt。")
        return 1

    config = load_config()
    drone = create_drone_adapter(use_fake, config=config)
    camera = None
    detector = create_detector(config)

    try:
        print("正在连接模拟无人机..." if use_fake else "正在连接 RoboMaster TT / Tello...")
        drone.connect()
        camera = CameraStream(
            drone=drone,
            width=int(config.get("camera_width", 640)),
            height=int(config.get("camera_height", 480)),
        )
        camera.start()
        print("视频流已开启。按 q 退出画面。")

        while True:
            frame = camera.read_frame()
            if frame is None:
                print("未读取到画面，请检查无人机视频流。")
                continue

            result = detector.detect(frame)
            debug_frame = detector.draw_debug(frame, result)
            cv2.imshow("PhantomFilmer Camera", debug_frame)
            last_mask = getattr(detector, "last_mask", None)
            if last_mask is not None:
                cv2.imshow("PhantomFilmer Red Mask", last_mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("已收到退出指令，正在关闭视频流。")
                break

        return 0
    except RuntimeError as exc:
        print(str(exc))
        print("请先连接 RoboMaster TT / Tello 的 Wi-Fi。")
        return 1
    except KeyboardInterrupt:
        print("已手动中断，正在关闭视频流。")
        return 0
    finally:
        if camera is not None:
            try:
                camera.stop()
            except RuntimeError as exc:
                print(str(exc))
        drone.stop()
        cv2.destroyAllWindows()


def run_camera_debug(use_fake: bool = False) -> int:
    """Show BGR, channel-swapped, and red-mask windows for camera diagnosis."""
    try:
        import cv2
    except ModuleNotFoundError:
        print("缺少 opencv-contrib-python 依赖：请先安装 requirements.txt。")
        return 1

    config = load_config()
    drone = create_drone_adapter(use_fake, config=config)
    camera = None
    detector = TargetDetector.from_config(config)

    try:
        print("正在连接模拟无人机..." if use_fake else "正在连接 RoboMaster TT / Tello...")
        drone.connect()
        camera = CameraStream(
            drone=drone,
            width=int(config.get("camera_width", 640)),
            height=int(config.get("camera_height", 480)),
        )
        camera.start()
        print("camera-debug 已启动：不允许起飞，不发送 move_rc。按 q 退出。")

        while True:
            frame = camera.read_frame()
            if frame is None:
                print("未读取到画面，请检查无人机视频流。")
                continue

            result = detector.detect(frame)
            debug_frame = detector.draw_debug(frame, result)
            swapped_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mask = detector.last_mask if detector.last_mask is not None else detector.create_red_mask(frame)

            cv2.imshow("camera-debug original BGR", debug_frame)
            cv2.imshow("camera-debug BGR/RGB swapped", swapped_frame)
            cv2.imshow("camera-debug red mask", mask)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("已收到退出指令，正在关闭视频流。")
                break

        return 0
    except RuntimeError as exc:
        print(str(exc))
        if not use_fake:
            print("请先连接 RoboMaster TT / Tello 的 Wi-Fi。")
        return 1
    except KeyboardInterrupt:
        print("已手动中断，正在关闭视频流。")
        return 0
    finally:
        if camera is not None:
            try:
                camera.stop()
            except RuntimeError as exc:
                print(str(exc))
        drone.stop()
        cv2.destroyAllWindows()


def run_basic_flight_test() -> int:
    """Run a real-drone takeoff, 5-second hover, and landing test."""
    config = load_config()
    safety = SafetyManager.from_dict(config)
    drone = TelloDroneAdapter()
    airborne = False

    try:
        print("正在连接 RoboMaster TT / Tello...")
        drone.connect()
        battery = drone.get_battery()
        height = drone.get_height()
        print("基础飞行测试状态：")
        print(f"- 当前电量：{battery}%")
        print(f"- 当前 TOF 离地高度：{height} cm")

        if not safety.can_takeoff(battery):
            print("电量低于安全起飞阈值，禁止起飞。")
            return 1

        print("即将执行基础起飞降落测试：起飞后只悬停 5 秒，然后自动降落。")
        answer = input("确认周围安全后输入 yes 起飞，其他输入取消：").strip()
        if answer != "yes":
            print("已取消基础飞行测试：未收到 yes 确认。")
            return 0

        drone.takeoff()
        airborne = True
        print("已起飞，保持悬停 5 秒。")
        sleep(5)
        print("悬停结束，准备自动降落。")
        drone.land()
        airborne = False
        return 0
    except KeyboardInterrupt:
        print("收到 Ctrl+C，中断测试，准备降落。")
        return 1
    except RuntimeError as exc:
        print(str(exc))
        print("请先连接 RoboMaster TT / Tello 的 Wi-Fi。")
        return 1
    finally:
        if airborne:
            try:
                drone.land()
            except RuntimeError as exc:
                print(str(exc))
        drone.stop()


def run_follow(
    use_fake: bool = False,
    obstacle_enabled: Optional[bool] = None,
) -> int:
    """Connect to the drone and run the shared visual follow session."""

    config = load_runtime_config(obstacle_enabled)
    safety = SafetyManager.from_dict(config)
    drone = create_drone_adapter(use_fake, config=config)
    detector = create_detector(config)
    controller = FollowController.from_config(safety_manager=safety, config=config)
    _, _, motion_arbiter = build_obstacle_modules(config, safety)
    try:
        print("正在连接模拟无人机..." if use_fake else "正在连接 RoboMaster TT / Tello...")
        drone.connect()

        battery = drone.get_battery()
        print(f"当前电量：{battery}%")
        if not safety.can_takeoff(battery):
            print("电量低于安全起飞阈值，禁止起飞。")
            return 1

        print("跟随模式需要起飞。请确认无人机周围安全、已安装保护罩、人员远离。")
        if use_fake:
            answer = input("输入 YES 确认模拟起飞，其他输入取消：").strip()
            if answer != "YES":
                print("已取消跟随模式：未收到用户确认。")
                return 0

        session = FollowSession(
            drone=drone,
            safety_manager=safety,
            detector=detector,
            follow_controller=controller,
            config=config,
            mode_label="FAKE" if use_fake else "REAL",
            window_name="PhantomFilmer Follow",
            state_label="FOLLOW",
            allow_pause=False,
            motion_arbiter=motion_arbiter,
        )
        session.run()
        return 0
    except RuntimeError as exc:
        print(str(exc))
        print("请先连接 RoboMaster TT / Tello 的 Wi-Fi。")
        return 1
    except KeyboardInterrupt:
        print("已手动中断，准备降落并退出。")
        return 0
    finally:
        drone.stop()


def run_reid_enroll(
    profile_name: Optional[str],
    reference_images: Optional[Sequence[str]] = None,
    reference_directory: Optional[str] = None,
    capture_reference: bool = False,
    reference_camera: int = 0,
    reference_count: int = 3,
    overwrite_profile: bool = False,
) -> int:
    """Create a persistent local ReID profile without connecting a drone."""
    if not str(profile_name or "").strip():
        print("reid-enroll 必须使用 --profile 指定人物档案名。")
        return 1
    try:
        if reference_directory:
            if reference_images or capture_reference:
                raise RuntimeError(
                    "--reference-dir 不能与 --reference-image 或 --capture-reference 同时使用。"
                )
            selected_images = validate_reference_directory(reference_directory)
        else:
            selected_images = collect_reference_images(
                provided_values=reference_images,
                capture_from_camera=capture_reference,
                camera_index=reference_camera,
                image_count=reference_count,
            )

        config = build_reid_runtime_config(load_config(), selected_images)
        detector = create_detector(config)
        print("正在加载 ReID 模型并从参考照片提取人物特征...")
        prepare_detector = getattr(detector, "prepare", None)
        if not callable(prepare_detector):
            raise RuntimeError("当前检测器不支持人物档案注册。")
        prepare_detector()
        reference_feature = getattr(detector, "reference_feature", None)
        if reference_feature is None:
            raise RuntimeError("当前检测器未生成可保存的人物特征。")
        manifest = save_reid_profile(
            str(profile_name),
            reference_feature,
            config,
            selected_images,
            overwrite=overwrite_profile,
        )
    except (RuntimeError, ValueError) as exc:
        print(str(exc))
        return 1

    print(f"人物档案注册成功：{manifest['profile_name']}")
    print(f"- 参考照片：{manifest['photo_count']} 张")
    print(f"- 特征维度：{manifest['embedding_dimension']}")
    print("- 档案仅保存在本地 data/reid_profiles/，不会连接无人机。")
    return 0


def run_reid_demo(
    use_fake: bool = False,
    obstacle_enabled: Optional[bool] = None,
    reference_images: Optional[Sequence[str]] = None,
    profile_name: Optional[str] = None,
    capture_reference: bool = False,
    reference_camera: int = 0,
    reference_count: int = 3,
    lock_frames: Optional[int] = None,
) -> int:
    """Load a ReID identity, take off on terminal confirmation, then follow/search."""
    selected_profile = str(profile_name or "").strip()
    selected_images: Sequence[Path] = []
    if selected_profile:
        if reference_images or capture_reference:
            print("--profile 不能与 --reference-image 或 --capture-reference 同时使用。")
            return 1
    else:
        try:
            selected_images = collect_reference_images(
                provided_values=reference_images,
                capture_from_camera=capture_reference,
                camera_index=reference_camera,
                image_count=reference_count,
            )
        except RuntimeError as exc:
            print(str(exc))
            return 1

    base_config = load_runtime_config(obstacle_enabled)
    config = build_reid_runtime_config(
        base_config,
        selected_images,
        profile_name=selected_profile or None,
    )
    safety = SafetyManager.from_dict(config)
    drone = create_drone_adapter(use_fake, config=config)
    try:
        detector = create_detector(config)
    except (RuntimeError, ValueError) as exc:
        print(str(exc))
        return 1
    controller = FollowController.from_config(safety_manager=safety, config=config)
    # 保留 --lock-frames 参数以兼容旧命令，但直接起飞流程不再用识别帧数授权起飞。
    del lock_frames

    try:
        if selected_profile:
            print("正在连接无人机前加载本地人物档案和实时 ReID 模型...")
        else:
            print("正在连接无人机前加载 ReID 模型并检查参考照片...")
        prepare_detector = getattr(detector, "prepare", None)
        if callable(prepare_detector):
            prepare_detector()
        print(
            "正在连接模拟无人机..."
            if use_fake
            else "正在连接 RoboMaster TT / Tello..."
        )
        drone.connect()
        battery = drone.get_battery()
        print(f"当前电量：{battery}%")
        if not safety.can_takeoff(battery):
            print("电量低于安全起飞阈值，禁止起飞。")
            return 1

        if selected_profile:
            print(f"已加载本地人物档案：{selected_profile}")
        else:
            print("已录入参考照片：")
            for path in selected_images:
                print(f"- {path}")
        print(
            "无需先识别到目标。确认后无人机将直接起飞到 "
            f"{int(config.get('base_hover_height_cm', 150))} cm，"
            "再进入 ReID 跟随；未发现目标时自动执行丢失搜索。"
        )
        try:
            answer = input(
                "确认周围及后方、上下方净空后，输入 y 起飞："
            ).strip().lower()
        except (EOFError, KeyboardInterrupt):
            answer = ""
        if answer not in {"y", "yes", "是"}:
            print("已取消 ReID 演示：未收到起飞确认。")
            return 0

        authorize_takeoff = getattr(drone, "authorize_next_takeoff", None)
        if callable(authorize_takeoff):
            authorize_takeoff()

        _, _, motion_arbiter = build_obstacle_modules(config, safety)

        session = FollowSession(
            drone=drone,
            safety_manager=safety,
            detector=detector,
            follow_controller=controller,
            config=config,
            mode_label="REID-DEMO FAKE" if use_fake else "REID-DEMO REAL",
            window_name="PhantomFilmer ReID Demo",
            state_label="REID",
            allow_pause=False,
            motion_arbiter=motion_arbiter,
            initial_target_lock_frames=0,
            enable_target_search=True,
        )
        session.run()
        return 0
    except RuntimeError as exc:
        print(str(exc))
        if not use_fake:
            print("请检查无人机 Wi-Fi、ReID 依赖、模型权重和参考照片。")
        return 1
    except KeyboardInterrupt:
        print("已手动中断 ReID 演示。")
        return 0
    finally:
        drone.stop()


def run_fixed_demo(
    use_fake: bool = False,
    obstacle_enabled: Optional[bool] = None,
) -> int:
    """Run the fixed low-speed route, then hand control to normal following."""
    config = load_runtime_config(obstacle_enabled)
    safety = SafetyManager.from_dict(config)
    drone = create_drone_adapter(use_fake, config=config)
    detector = create_detector(config)
    controller = FollowController.from_config(safety_manager=safety, config=config)
    _, _, motion_arbiter = build_obstacle_modules(config, safety)

    try:
        print("正在连接模拟无人机..." if use_fake else "正在连接 RoboMaster TT / Tello...")
        drone.connect()
        battery = drone.get_battery()
        print(f"当前电量：{battery}%")
        if not safety.can_takeoff(battery):
            print("电量低于安全起飞阈值，禁止起飞。")
            return 1

        print("固定演示需要起飞。请确认航线净空、已安装保护罩、人员远离。")
        print("航线：左移 3 秒 → 前进 2 秒 → 右移 3 秒 → 跟随。")
        if use_fake:
            answer = input("输入 YES 确认模拟起飞，其他输入取消：").strip()
            if answer != "YES":
                print("已取消固定演示：未收到用户确认。")
                return 0

        session = FollowSession(
            drone=drone,
            safety_manager=safety,
            detector=detector,
            follow_controller=controller,
            config=config,
            mode_label="FIXED-DEMO FAKE" if use_fake else "FIXED-DEMO REAL",
            window_name="PhantomFilmer Fixed Demo",
            state_label="FOLLOW",
            allow_pause=False,
            pre_follow_maneuver=FixedDemoManeuver(
                control_interval=read_control_interval(config)
            ),
            motion_arbiter=motion_arbiter,
        )
        session.run()
        return 0
    except RuntimeError as exc:
        print(str(exc))
        if not use_fake:
            print("请先连接 RoboMaster TT / Tello 的 Wi-Fi。")
        return 1
    except KeyboardInterrupt:
        print("已手动中断，准备降落并退出。")
        return 0
    finally:
        drone.stop()


def run_follow_dry_run(
    use_fake: bool = False,
    obstacle_enabled: Optional[bool] = None,
) -> int:
    """Preview follow-control commands without takeoff or RC output."""
    try:
        import cv2
    except ModuleNotFoundError:
        print("缺少 opencv-contrib-python 依赖：请先安装 requirements.txt。")
        return 1

    config = load_runtime_config(obstacle_enabled)
    safety = SafetyManager.from_dict(config)
    drone = create_drone_adapter(use_fake, config=config)
    camera = None
    detector = create_detector(config)
    controller = FollowController.from_config(safety_manager=safety, config=config)
    _, _, motion_arbiter = build_obstacle_modules(config, safety)
    front_tof_monitor = None
    obstacle_config = config.get("obstacle", {})
    if (
        motion_arbiter is not None
        and isinstance(obstacle_config, dict)
        and bool(obstacle_config.get("front_tof_enabled", False))
    ):
        front_tof_monitor = FrontToFMonitor.from_config(drone, config)
        motion_arbiter.set_front_tof_provider(front_tof_monitor.snapshot)
    control_interval = read_control_interval(config)
    # 与生产路径共用同一仲裁引擎与 feature 注册表，保证 dry-run 决策逐帧一致。
    engine = ArbitrationEngine(
        features=build_features(
            follow_controller=controller,
            safety_manager=safety,
            motion_arbiter=motion_arbiter,
            mode_label="DRY_RUN",
        ),
        follow_controller=controller,
        mode_label="DRY_RUN",
    )

    try:
        print("正在连接模拟无人机..." if use_fake else "正在连接 RoboMaster TT / Tello...")
        drone.connect()
        camera = CameraStream(
            drone=drone,
            width=int(config.get("camera_width", 640)),
            height=int(config.get("camera_height", 480)),
        )
        camera.start()
        if front_tof_monitor is not None:
            front_tof_monitor.prepare()
            front_tof_monitor.start()
        print("follow-dry-run 已启动：只计算控制量，不起飞，不发送 move_rc。按 q 退出。")

        while True:
            frame = camera.read_frame()
            if frame is None:
                command = controller.hover()
                print(f"未读取到画面，理论控制量：{command.as_tuple()}")
                continue

            frame_height, frame_width = frame.shape[:2]
            target_result = detector.detect(frame)
            # 配方表（1-6）逐帧仲裁：目标丢失+障碍 → 避障接管（暂停找人）、
            # 目标存在 → follow 期望指令经避障仲裁、目标丢失+无障碍 → 丢失悬停。
            outcome = engine.arbitrate(
                ArbitrationContext(
                    phase=KernelPhase.FOLLOW,
                    target_result=target_result,
                    frame=frame,
                    frame_width=frame_width,
                    frame_height=frame_height,
                    mode="DRY_RUN",
                    height_cm=None,
                    paused=False,
                    emergency=False,
                    stop_requested=False,
                    now=monotonic(),
                )
            )
            command = outcome.command
            obstacle_result = outcome.obstacle_observation
            avoidance_decision = outcome.avoidance_decision
            debug = controller.last_debug
            left_right, forward_backward, up_down, yaw = command.as_tuple()
            print(
                "理论控制量："
                f"target_center_x={debug.target_center_x}, "
                f"target_center_y={debug.target_center_y}, "
                f"frame_center_x={debug.frame_center_x}, "
                f"frame_center_y={debug.frame_center_y}, "
                f"horizontal_error={debug.horizontal_error}, "
                f"horizontal_error_ratio={debug.horizontal_error_ratio:.3f}, "
                f"vertical_error={debug.vertical_error}, "
                f"vertical_error_ratio={debug.vertical_error_ratio:.3f}, "
                f"target_area={debug.target_area:.1f}, "
                f"area_ratio={debug.area_ratio:.4f}, "
                f"target_state={debug.target_state}, "
                f"obstacle_state={(avoidance_decision.state if avoidance_decision else 'DISABLED')}, "
                f"yaw={yaw}, "
                f"left_right={left_right}, "
                f"forward_back={forward_backward}, "
                f"up_down={up_down}"
            )

            debug_frame = detector.draw_debug(frame, target_result)
            if motion_arbiter is not None:
                debug_frame = motion_arbiter.detector.draw_debug(debug_frame, obstacle_result)
            cv2.rectangle(debug_frame, (12, 84), (620, 238), (0, 0, 0), -1)
            obstacle_line = "obstacle=DISABLED"
            if obstacle_result is not None and avoidance_decision is not None:
                obstacle_line = (
                    f"obstacle={avoidance_decision.state} side={obstacle_result.side} "
                    f"area={obstacle_result.area_ratio:.3f} "
                    f"front_tof={obstacle_result.front_distance_cm}cm "
                    f"{avoidance_decision.reason}"
                )
            dry_run_lines = (
                f"dry-run rc: lr={left_right} fb={forward_backward} ud={up_down} yaw={yaw}",
                f"x_err={debug.horizontal_error} x_ratio={debug.horizontal_error_ratio:.2f}",
                f"y_err={debug.vertical_error} y_ratio={debug.vertical_error_ratio:.2f} area_ratio={debug.area_ratio:.3f}",
                obstacle_line,
                f"target={debug.target_state} no takeoff, no move_rc, q to quit",
            )
            for index, line in enumerate(dry_run_lines):
                cv2.putText(
                    debug_frame,
                    line,
                    (20, 106 + index * 26),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (255, 255, 255),
                    2,
                )
            cv2.imshow("PhantomFilmer Follow Dry Run", debug_frame)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                print("已收到退出指令，正在关闭视频流。")
                break
            sleep(control_interval)

        return 0
    except RuntimeError as exc:
        print(str(exc))
        if not use_fake:
            print("请先连接 RoboMaster TT / Tello 的 Wi-Fi。")
        return 1
    except KeyboardInterrupt:
        print("已手动中断，正在关闭视频流。")
        return 0
    finally:
        if front_tof_monitor is not None:
            front_tof_monitor.stop()
        if camera is not None:
            try:
                camera.stop()
            except RuntimeError as exc:
                print(str(exc))
        if motion_arbiter is not None:
            motion_arbiter.close()
        drone.stop()
        cv2.destroyAllWindows()


def run_follow_test() -> int:
    """Verify FollowController direction logic without drone or camera."""
    safety = build_safety_manager()
    controller = FollowController.from_config(safety_manager=safety, config=load_config())
    frame_width = 640
    frame_height = 480

    cases = (
        ("目标在画面左侧", {"found": True, "center": (120, 240), "area": 6000, "bbox": (90, 210, 60, 60)}),
        ("目标在画面右侧", {"found": True, "center": (520, 240), "area": 6000, "bbox": (490, 210, 60, 60)}),
        ("目标在画面中心", {"found": True, "center": (320, 240), "area": 6000, "bbox": (290, 210, 60, 60)}),
        ("目标在画面上方", {"found": True, "center": (320, 100), "area": 6000, "bbox": (290, 70, 60, 60)}),
        ("目标在画面下方", {"found": True, "center": (320, 380), "area": 6000, "bbox": (290, 350, 60, 60)}),
        ("目标面积太小", {"found": True, "center": (320, 240), "area": 1000, "bbox": (305, 225, 30, 30)}),
        ("目标面积太大", {"found": True, "center": (320, 240), "area": 40000, "bbox": (220, 140, 200, 200)}),
        ("目标丢失", {"found": False, "center": None, "area": 0, "bbox": None}),
    )

    print("跟随控制方向逻辑测试：")
    print(f"frame_width={frame_width}, frame_height={frame_height}")
    for name, target_result in cases:
        command = controller.compute_command(target_result, frame_width, frame_height)
        left_right, forward_backward, up_down, yaw = command.as_tuple()
        print(
            f"- {name}："
            f"left_right={left_right}, "
            f"forward_backward={forward_backward}, "
            f"up_down={up_down}, "
            f"yaw={yaw}"
        )

    return 0


def run_safety_test() -> int:
    """Run simple safety checks without connecting to a real drone."""
    safety = build_safety_manager()

    print("安全保护模块测试：")

    # 电量检查：低于起飞阈值禁止起飞，低于降落阈值建议降落。
    for battery in (10, 20, 29, 30, 80):
        print(
            f"- 电量 {battery}%："
            f"允许起飞={safety.can_takeoff(battery)}，"
            f"建议降落={safety.should_land(battery)}"
        )

    # 速度限制：输入超过阈值时自动裁剪到安全范围。
    raw_command = (-100, -30, 0, 88)
    limited_command = safety.limit_rc_command(*raw_command)
    print(f"- RC 原始指令：{raw_command}")
    print(f"- RC 限制后：{limited_command}")

    # 高度检查：只允许配置范围内的低空高度。
    for height in (30, 60, 120, 150, 200):
        print(f"- 高度 {height} cm：安全={safety.check_height(height)}")

    # 目标丢失逻辑：用较短等待演示 keep -> hover -> land 的状态变化。
    print("- 目标丢失逻辑：")
    print(f"  目标可见：{safety.update_target_lost(True)}")
    print(f"  刚丢失：{safety.update_target_lost(False)}")
    safety._target_lost_since -= safety.config.target_lost_hover_seconds + 0.1
    print(f"  超过悬停时间：{safety.update_target_lost(False)}")
    safety._target_lost_since -= (
        safety.config.target_lost_land_seconds - safety.config.target_lost_hover_seconds + 0.1
    )
    print(f"  超过降落时间：{safety.update_target_lost(False)}")
    print(f"  目标恢复：{safety.update_target_lost(True)}")

    return 0
