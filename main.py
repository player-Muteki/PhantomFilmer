"""Project entry point for the DroneUmbrella prototype."""

import argparse
from pathlib import Path
from time import sleep
from typing import Optional

from agent.agent_controller import AgentController
from agent.command_parser import CommandParser
from agent.llm_client import (
    DEFAULT_BASE_URL,
    DEFAULT_MODEL,
    DEFAULT_TIMEOUT_SECONDS,
    LLMClient,
)
from agent.tools import AgentTools
from control.follow_control import FollowController
from control.follow_session import FollowSession
from drone.drone_adapter import DroneAdapter
from drone.fake_adapter import FakeDroneAdapter
from drone.safety import SafetyConfig, SafetyManager
from drone.tello_adapter import TelloDroneAdapter
from swarm.fake_swarm import create_fake_swarm_nodes
from swarm.formation_sim import FormationSimulator
from swarm.swarm_manager import SwarmBatchResult, SwarmManager
from vision.camera import CameraStream
from vision.detector_factory import create_detector
from vision.target_detect import TargetDetector


CONFIG_PATH = Path(__file__).with_name("config.yaml")


def load_config(path: Path = CONFIG_PATH) -> dict:
    """Load YAML configuration for the prototype.

    PyYAML is used when installed. A small fallback parser keeps import tests
    working before dependencies are installed.
    """
    try:
        import yaml

        with path.open("r", encoding="utf-8") as file:
            return yaml.safe_load(file)
    except ModuleNotFoundError:
        config = {}
        for line in path.read_text(encoding="utf-8").splitlines():
            if not line.strip() or line.lstrip().startswith("#"):
                continue
            key, value = line.split(":", 1)
            config[key.strip()] = _parse_config_value(value.strip())
        return config


def _parse_config_value(value: str) -> object:
    """Parse a small subset of YAML scalars for local fallback loading."""
    if value.startswith(('"', "'")) and value.endswith(('"', "'")):
        return value[1:-1]

    lowered = value.lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False

    try:
        return int(value)
    except ValueError:
        pass

    try:
        return float(value)
    except ValueError:
        return value


def build_safety_manager() -> SafetyManager:
    """Create the safety manager from config.yaml."""
    return SafetyManager.from_dict(load_config())


def read_control_interval(config: dict) -> float:
    """Read control loop interval from config.yaml with a safe default."""
    try:
        interval = float(config.get("control_interval", 0.05))
    except (TypeError, ValueError):
        interval = 0.05
    if interval <= 0:
        return 0.05
    return max(0.02, min(0.2, interval))


def create_drone_adapter(
    use_fake: bool,
    verbose_fake_rc: bool = True,
    config: Optional[dict] = None,
) -> DroneAdapter:
    """Create either the fake drone adapter or the real Tello adapter."""
    if use_fake:
        config = config or {}
        return FakeDroneAdapter(
            verbose_rc=verbose_fake_rc,
            camera_width=int(config.get("camera_width", 640)),
            camera_height=int(config.get("camera_height", 480)),
            target_speed=int(config.get("fake_target_speed", 3)),
            target_lost_interval_seconds=float(
                config.get("fake_target_lost_interval_seconds", 12)
            ),
            target_lost_duration_seconds=float(
                config.get("fake_target_lost_duration_seconds", 2)
            ),
        )
    return TelloDroneAdapter()


def build_system(use_fake: bool = False) -> AgentController:
    """Create the natural-language Agent with safety-wrapped tools."""
    config = load_config()
    safety_manager = SafetyManager(SafetyConfig.from_dict(config))
    detector = create_detector(config)
    follow_controller = FollowController.from_config(
        safety_manager=safety_manager,
        config=config,
    )
    tools = AgentTools(
        drone=create_drone_adapter(use_fake, verbose_fake_rc=False, config=config),
        safety_manager=safety_manager,
        detector=detector,
        follow_controller=follow_controller,
        config=config,
        mode_label="FAKE" if use_fake else "REAL",
        frame_width=int(config.get("camera_width", 640)),
        frame_height=int(config.get("camera_height", 480)),
    )
    llm_client = LLMClient(
        base_url=str(config.get("llm_base_url", DEFAULT_BASE_URL)),
        model=str(config.get("llm_model", DEFAULT_MODEL)),
        timeout_seconds=float(config.get("llm_timeout_seconds", DEFAULT_TIMEOUT_SECONDS)),
        enabled=bool(config.get("llm_enabled", False)),
    )
    parser = CommandParser(llm_client=llm_client)
    return AgentController(tools=tools, parser=parser, llm_client=llm_client)


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


def run_agent(use_fake: bool = False) -> int:
    """Run the interactive rule-based Agent scheduler."""
    controller = build_system(use_fake=use_fake)
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
            cv2.imshow("DroneUmbrella Camera", debug_frame)
            last_mask = getattr(detector, "last_mask", None)
            if last_mask is not None:
                cv2.imshow("DroneUmbrella Red Mask", last_mask)

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


def run_follow(use_fake: bool = False) -> int:
    """Connect to the drone and run the shared visual follow session."""

    config = load_config()
    safety = SafetyManager.from_dict(config)
    drone = create_drone_adapter(use_fake, config=config)
    detector = create_detector(config)
    controller = FollowController.from_config(safety_manager=safety, config=config)

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
            window_name="DroneUmbrella Follow",
            state_label="FOLLOW",
            allow_pause=False,
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


def run_follow_dry_run(use_fake: bool = False) -> int:
    """Preview follow-control commands without takeoff or RC output."""
    try:
        import cv2
    except ModuleNotFoundError:
        print("缺少 opencv-contrib-python 依赖：请先安装 requirements.txt。")
        return 1

    config = load_config()
    safety = SafetyManager.from_dict(config)
    drone = create_drone_adapter(use_fake, config=config)
    camera = None
    detector = create_detector(config)
    controller = FollowController.from_config(safety_manager=safety, config=config)
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
                f"yaw={yaw}, "
                f"left_right={left_right}, "
                f"forward_back={forward_backward}, "
                f"up_down={up_down}"
            )

            debug_frame = detector.draw_debug(frame, target_result)
            cv2.rectangle(debug_frame, (12, 84), (620, 214), (0, 0, 0), -1)
            dry_run_lines = (
                f"dry-run rc: lr={left_right} fb={forward_backward} ud={up_down} yaw={yaw}",
                f"x_err={debug.horizontal_error} x_ratio={debug.horizontal_error_ratio:.2f}",
                f"y_err={debug.vertical_error} y_ratio={debug.vertical_error_ratio:.2f} area_ratio={debug.area_ratio:.3f}",
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
            cv2.imshow("DroneUmbrella Follow Dry Run", debug_frame)

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
    """Run a four-drone virtual-structure umbrella simulation."""
    target = (0.0, 0.0, 0.0)
    d = 1.2
    h = 1.5
    simulator = FormationSimulator(target=target, d=d, h=h)
    points = simulator.compute_umbrella_formation()
    output_path = Path(__file__).with_name("docs") / "swarm_formation.png"
    simulator.save_2d_plot(output_path)

    print("四机协同打伞仿真：虚拟结构法")
    print(f"- 行人目标中心 target=(x={target[0]:.2f}, y={target[1]:.2f}, z={target[2]:.2f})")
    print(f"- 水平偏移 d={d:.2f} m，飞行高度 h={h:.2f} m")
    for name, point in points.items():
        print(f"- {name}: x={point.x:.2f}, y={point.y:.2f}, z={point.z:.2f}")
    print(f"- 二维可视化已保存：{output_path}")
    return 0


def build_fake_swarm_manager(config: dict) -> SwarmManager:
    """Create a fake four-node swarm manager from config.yaml."""
    swarm_config = config.get("swarm", {})
    if not isinstance(swarm_config, dict):
        swarm_config = {}
    drone_configs = swarm_config.get("drones")
    nodes = create_fake_swarm_nodes(drone_configs if isinstance(drone_configs, list) else None)
    return SwarmManager.from_config(config, nodes)


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
    """Read fake swarm status without takeoff."""
    if not use_fake:
        print("swarm-status 当前只允许 --fake，避免误连真机。")
        return 1
    manager = build_fake_swarm_manager(load_config())
    print_swarm_batch(manager.connect_all())
    print_swarm_batch(manager.status_all())
    return 0


def run_swarm_connect_test(use_fake: bool = False) -> int:
    """Connect fake swarm nodes and send zero RC only."""
    if not use_fake:
        print("swarm-connect-test 当前只允许 --fake，避免误连真机。")
        return 1
    manager = build_fake_swarm_manager(load_config())
    print_swarm_batch(manager.connect_all())
    print_swarm_batch(manager.zero_rc_all())
    print_swarm_batch(manager.emergency_stop_all())
    return 0


def run_swarm_basic_test(use_fake: bool = False) -> int:
    """Run fake swarm connect, takeoff, zero RC, and landing sequence."""
    if not use_fake:
        print("swarm-basic-test 当前只允许 --fake，真机起降入口后续单独审核。")
        return 1
    answer = input("即将运行 Fake Swarm 起降流程。输入 YES 继续：").strip()
    if answer != "YES":
        print("已取消 Fake Swarm 基础测试：未收到 YES 确认。")
        return 0
    manager = build_fake_swarm_manager(load_config())
    print_swarm_batch(manager.connect_all())
    print_swarm_batch(manager.takeoff_sequence())
    print_swarm_batch(manager.zero_rc_all())
    print_swarm_batch(manager.land_sequence())
    print_swarm_batch(manager.emergency_stop_all())
    return 0


def parse_args() -> argparse.Namespace:
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(description="DroneUmbrella prototype controller")
    parser.add_argument(
        "--mode",
        choices=(
            "demo",
            "status",
            "safety-test",
            "follow-test",
            "follow-dry-run",
            "basic-flight-test",
            "camera-debug",
            "camera",
            "follow",
            "agent",
            "swarm-sim",
            "swarm-status",
            "swarm-connect-test",
            "swarm-basic-test",
        ),
        default="demo",
        help="运行模式：demo 启动骨架说明，status 读取无人机状态，safety-test 测试安全保护逻辑，follow-test 测试跟随方向逻辑，follow-dry-run 真机起飞前干跑验证，basic-flight-test 真机基础起降测试，camera-debug 调试颜色通道和红色 mask，camera 显示视频识别画面，follow 低速目标跟随，agent 规则版任务调度，swarm-sim 多机编队仿真，swarm-status/swarm-connect-test/swarm-basic-test 运行 Fake Swarm 验证。",
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
    if args.mode == "status":
        return run_status(use_fake=args.fake)
    if args.mode == "safety-test":
        return run_safety_test()
    if args.mode == "follow-test":
        return run_follow_test()
    if args.mode == "follow-dry-run":
        return run_follow_dry_run(use_fake=args.fake)
    if args.mode == "basic-flight-test":
        return run_basic_flight_test()
    if args.mode == "camera-debug":
        return run_camera_debug(use_fake=args.fake)
    if args.mode == "camera":
        return run_camera(use_fake=args.fake)
    if args.mode == "follow":
        return run_follow(use_fake=args.fake)
    if args.mode == "agent":
        return run_agent(use_fake=args.fake)
    if args.mode == "swarm-sim":
        return run_swarm_sim()
    if args.mode == "swarm-status":
        return run_swarm_status(use_fake=args.fake)
    if args.mode == "swarm-connect-test":
        return run_swarm_connect_test(use_fake=args.fake)
    if args.mode == "swarm-basic-test":
        return run_swarm_basic_test(use_fake=args.fake)

    controller = build_system(use_fake=args.fake)
    controller.describe()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
