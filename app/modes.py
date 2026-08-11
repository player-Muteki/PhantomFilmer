"""Per-mode CLI runners for PhantomFilmer."""

from pathlib import Path
from time import sleep
from typing import Optional

from app.builder import (
    build_obstacle_modules,
    build_safety_manager,
    build_swarm_manager,
    build_system,
    create_drone_adapter,
)
from app.config import load_config, load_runtime_config, read_control_interval
from control.fixed_demo import FixedDemoManeuver
from control.follow_control import FollowController
from control.follow_session import FollowSession
from control.motion_arbiter import MotionContext
from drone.safety import SafetyManager
from drone.tello_adapter import TelloDroneAdapter
from swarm.formation_sim import FormationSimulator
from swarm.swarm_manager import SwarmBatchResult
from vision.camera import CameraStream
from vision.detector_factory import create_detector
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
        print(f"- 高度：{height} cm")
        return 0
    except RuntimeError as exc:
        print(str(exc))
        print("请先连接 RoboMaster TT / Tello 的 Wi-Fi。")
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
    obstacle_detector, _obstacle_planner = build_obstacle_modules(config, SafetyManager.from_dict(config))

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
            if obstacle_detector is not None:
                obstacle_result = obstacle_detector.detect(frame, result)
                debug_frame = obstacle_detector.draw_debug(debug_frame, obstacle_result)
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
        print(f"- 当前高度：{height} cm")

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
    obstacle_detector, obstacle_planner, motion_arbiter = build_obstacle_modules(config, safety)

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
            obstacle_detector=obstacle_detector,
            obstacle_planner=obstacle_planner,
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
    obstacle_detector, obstacle_planner, motion_arbiter = build_obstacle_modules(config, safety)

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
            obstacle_detector=obstacle_detector,
            obstacle_planner=obstacle_planner,
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
    obstacle_detector, obstacle_planner, motion_arbiter = build_obstacle_modules(config, safety)
    control_interval = read_control_interval(config)

    try:
        print("正在连接模拟无人机..." if use_fake else "正在连接 RoboMaster TT / Tello...")
        drone.connect()
        camera = CameraStream(
            drone=drone,
            width=int(config.get("camera_width", 640)),
            height=int(config.get("camera_height", 480)),
        )
        camera.start()
        print("follow-dry-run 已启动：只计算控制量，不起飞，不发送 move_rc。按 q 退出。")

        while True:
            frame = camera.read_frame()
            if frame is None:
                command = controller.hover()
                print(f"未读取到画面，理论控制量：{command.as_tuple()}")
                continue

            frame_height, frame_width = frame.shape[:2]
            target_result = detector.detect(frame)
            command = controller.compute_command(target_result, frame_width, frame_height)
            obstacle_result = None
            avoidance_decision = None
            if motion_arbiter is not None:
                avoidance_decision = motion_arbiter.decide(
                    desired_command=command,
                    frame=frame,
                    context=MotionContext(mode="DRY_RUN", target_result=target_result),
                )
                obstacle_result = avoidance_decision.observation
                command = avoidance_decision.command
            elif obstacle_detector is not None and obstacle_planner is not None and target_result.get("found"):
                obstacle_result = obstacle_detector.detect(frame, target_result)
                avoidance_decision = obstacle_planner.plan(command, obstacle_result)
                command = avoidance_decision.command
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
            if obstacle_detector is not None:
                debug_frame = obstacle_detector.draw_debug(debug_frame, obstacle_result)
            cv2.rectangle(debug_frame, (12, 84), (620, 238), (0, 0, 0), -1)
            obstacle_line = "obstacle=DISABLED"
            if obstacle_result is not None and avoidance_decision is not None:
                obstacle_line = (
                    f"obstacle={avoidance_decision.state} side={obstacle_result.side} "
                    f"area={obstacle_result.area_ratio:.3f} {avoidance_decision.reason}"
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


def run_swarm_sim() -> int:
    """Run a four-drone virtual-structure formation simulation."""
    target = (0.0, 0.0, 0.0)
    d = 1.2
    h = 1.5
    simulator = FormationSimulator(target=target, d=d, h=h)
    points = simulator.compute_umbrella_formation()
    output_path = Path(__file__).resolve().parents[1] / "docs" / "swarm_formation.png"
    simulator.save_2d_plot(output_path)

    print("四机协同编队仿真：虚拟结构法")
    print(f"- 行人目标中心 target=(x={target[0]:.2f}, y={target[1]:.2f}, z={target[2]:.2f})")
    print(f"- 水平偏移 d={d:.2f} m，飞行高度 h={h:.2f} m")
    for name, point in points.items():
        print(f"- {name}: x={point.x:.2f}, y={point.y:.2f}, z={point.z:.2f}")
    print(f"- 二维可视化已保存：{output_path}")
    return 0


def print_swarm_batch(result: SwarmBatchResult) -> None:
    """Print compact per-node swarm results for manual checks."""
    print(f"{result.action}: success={result.success}, elapsed_ms={result.elapsed_ms:.1f}")
    for drone_id, item in result.results.items():
        status = item.status
        print(
            f"- {drone_id}: "
            f"success={item.success}, "
            f"connected={status.connected}, "
            f"airborne={status.airborne}, "
            f"battery={status.battery}, "
            f"height={status.height}, "
            f"command={item.command}, "
            f"error={item.error}"
        )


def run_swarm_status(use_fake: bool = False) -> int:
    """Read swarm status without takeoff or video."""
    manager = build_swarm_manager(load_config(), use_fake=use_fake)
    print("Swarm 状态读取：不会起飞，不打开视频流。")
    print_swarm_batch(manager.connect_all())
    print_swarm_batch(manager.status_all())
    return 0


def run_swarm_connect_test(use_fake: bool = False) -> int:
    """Connect swarm nodes and send zero RC only."""
    if not use_fake and not confirm_real_swarm_action("连接四机并发送零 RC/急停清理"):
        return 0
    manager = build_swarm_manager(load_config(), use_fake=use_fake)
    print_swarm_batch(manager.connect_all())
    print_swarm_batch(manager.zero_rc_all())
    print_swarm_batch(manager.emergency_stop_all())
    return 0


def run_swarm_basic_test(use_fake: bool = False) -> int:
    """Run swarm connect, takeoff, zero RC, and landing sequence."""
    if not use_fake and not confirm_real_swarm_action("四机顺序起飞、清零、顺序降落"):
        return 0
    manager = build_swarm_manager(load_config(), use_fake=use_fake)
    print_swarm_batch(manager.connect_all())
    print_swarm_batch(manager.takeoff_sequence())
    print_swarm_batch(manager.zero_rc_all())
    print_swarm_batch(manager.land_sequence())
    print_swarm_batch(manager.emergency_stop_all())
    return 0


def run_swarm_hover_test(use_fake: bool = False) -> int:
    """Run sequential takeoff, short synchronized hover, and landing."""
    config = load_config()
    if not use_fake and not confirm_real_swarm_action("四机顺序起飞、同步悬停、顺序降落"):
        return 0
    manager = build_swarm_manager(config, use_fake=use_fake)
    swarm_config = config.get("swarm", {})
    if not isinstance(swarm_config, dict):
        swarm_config = {}
    hover_seconds = float(swarm_config.get("hover_test_seconds", 10))
    print_swarm_batch(manager.connect_all())
    takeoff = manager.takeoff_sequence()
    print_swarm_batch(takeoff)
    if takeoff.success:
        print(f"同步悬停 {hover_seconds:.1f} 秒。")
        sleep(max(0.0, hover_seconds))
        print_swarm_batch(manager.zero_rc_all())
    print_swarm_batch(manager.land_sequence())
    print_swarm_batch(manager.emergency_stop_all())
    return 0


def run_swarm_rc_test(use_fake: bool = False) -> int:
    """Run one low-speed, short RC move and immediately zero all nodes."""
    config = load_config()
    if not use_fake and not confirm_real_swarm_action("四机顺序起飞、低速短时移动、立即清零并降落"):
        return 0
    swarm_config = config.get("swarm", {})
    if not isinstance(swarm_config, dict):
        swarm_config = {}
    move_seconds = float(swarm_config.get("rc_test_seconds", 0.5))
    command = (
        int(swarm_config.get("rc_test_left_right", 0)),
        int(swarm_config.get("rc_test_forward_backward", 8)),
        int(swarm_config.get("rc_test_up_down", 0)),
        int(swarm_config.get("rc_test_yaw", 0)),
    )

    manager = build_swarm_manager(config, use_fake=use_fake)
    print_swarm_batch(manager.connect_all())
    takeoff = manager.takeoff_sequence()
    print_swarm_batch(takeoff)
    if takeoff.success:
        print(f"低速短时 RC 指令 {command}，持续 {move_seconds:.1f} 秒后立即清零。")
        result = manager.send_rc_all(command, duration_s=move_seconds)
        print_swarm_batch(result)
        print_swarm_batch(manager.zero_rc_all())
    print_swarm_batch(manager.land_sequence())
    print_swarm_batch(manager.emergency_stop_all())
    return 0


def confirm_real_swarm_action(action_label: str) -> bool:
    """Require explicit confirmation before any real swarm action with risk."""
    print(f"即将执行真机 Swarm 操作：{action_label}。")
    print("请确认空域安全、桨叶/保护罩状态正确、四机 IP 与编号已经核对。")
    answer = input("输入 YES 继续，其他输入取消：").strip()
    if answer != "YES":
        print("已取消真机 Swarm 操作：未收到 YES 确认。")
        return False
    return True
