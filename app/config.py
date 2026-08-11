"""Project configuration loading for PhantomFilmer."""

from pathlib import Path
from typing import Optional


CONFIG_PATH = Path(__file__).resolve().parents[1] / "config.yaml"
FOLLOW_MODES = {"follow", "follow-dry-run", "console", "fixed-demo", "reid-demo"}


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
        return _load_config_without_yaml(path)


def configured_obstacle_enabled(config: dict) -> bool:
    """Return whether obstacle avoidance is enabled in project config."""
    obstacle = config.get("obstacle", {})
    if not isinstance(obstacle, dict):
        return False
    return bool(obstacle.get("enabled", False))


def load_runtime_config(obstacle_enabled: Optional[bool] = None) -> dict:
    """Load config with an optional in-memory obstacle override."""
    config = load_config()
    if obstacle_enabled is None:
        return config

    obstacle = config.get("obstacle", {})
    runtime_config = dict(config)
    runtime_obstacle = dict(obstacle) if isinstance(obstacle, dict) else {}
    runtime_obstacle["enabled"] = obstacle_enabled
    runtime_config["obstacle"] = runtime_obstacle
    return runtime_config


def prompt_obstacle_enabled(default_enabled: bool) -> Optional[bool]:
    """Ask whether obstacle avoidance should be enabled for this run."""
    default_label = "开启" if default_enabled else "关闭"
    while True:
        try:
            answer = input(
                f"本次运行是否开启避障？[y/n]（配置默认：{default_label}）："
            ).strip().lower()
        except (EOFError, KeyboardInterrupt):
            print("\n已取消本次运行。")
            return None

        if not answer:
            return default_enabled
        if answer in {"y", "yes", "是", "开启"}:
            return True
        if answer in {"n", "no", "否", "关闭"}:
            return False
        print("请输入 y/n、是/否或直接回车使用配置默认值。")


def _load_config_without_yaml(path: Path) -> dict:
    """Parse the project's simple config.yaml shape when PyYAML is missing."""
    config = {}
    lines = path.read_text(encoding="utf-8").splitlines()
    index = 0
    while index < len(lines):
        raw = lines[index]
        stripped = raw.strip()
        if not stripped or stripped.startswith("#"):
            index += 1
            continue

        if raw.startswith("vision:"):
            vision, index = _parse_flat_block(lines, index + 1)
            config["vision"] = vision
            continue

        if raw.startswith("obstacle:"):
            obstacle, index = _parse_flat_block(lines, index + 1)
            config["obstacle"] = obstacle
            continue

        if ":" in raw and not raw.startswith(" "):
            key, value = raw.split(":", 1)
            config[key.strip()] = _parse_config_value(value.strip())
            # 顶层键后若跟有缩进行，说明存在本回退解析器不支持的嵌套块：
            # 显式报错而不是静默丢弃，避免配置在无 PyYAML 环境下被误解读。
            probe = index + 1
            while probe < len(lines):
                probe_line = lines[probe]
                if not probe_line.strip() or probe_line.lstrip().startswith("#"):
                    probe += 1
                    continue
                if probe_line.startswith(" "):
                    raise ValueError(
                        f"config 回退解析器不支持顶层键 '{key.strip()}' 下的嵌套块"
                        f"（第 {index + 1} 行）。请安装 PyYAML，"
                        "或在该解析器中为对应块补充解析逻辑。"
                    )
                break
        index += 1
    return config


def _parse_flat_block(lines: list[str], start_index: int) -> tuple[dict, int]:
    """Parse a simple indented YAML mapping used by the vision block."""
    values = {}
    index = start_index
    while index < len(lines):
        raw = lines[index]
        stripped = raw.strip()
        if raw and not raw.startswith(" ") and stripped:
            break
        if not stripped or stripped.startswith("#"):
            index += 1
            continue
        if ":" in stripped:
            key, value = stripped.split(":", 1)
            values[key.strip()] = _parse_config_value(value.strip())
        index += 1
    return values, index


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


def read_control_interval(config: dict) -> float:
    """Read control loop interval from config.yaml with a safe default."""
    try:
        interval = float(config.get("control_interval", 0.05))
    except (TypeError, ValueError):
        interval = 0.05
    if interval <= 0:
        return 0.05
    return max(0.02, min(0.2, interval))


def selected_detector_type(config: dict) -> str:
    """Return the normalized detector type selected by project config."""
    vision = config.get("vision", {})
    if not isinstance(vision, dict):
        return "red"
    return str(vision.get("detector_type", "red")).strip().lower()
