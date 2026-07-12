"""Rule-based Agent for high-level DroneUmbrella task scheduling."""

from agent.tools import AgentTools


class AgentController:
    """Parse fixed commands and call only safety-wrapped AgentTools."""

    COMMANDS = ("状态", "开始任务", "停止任务", "急停", "退出")

    def __init__(self, tools: AgentTools) -> None:
        self.tools = tools

    def describe(self) -> None:
        """Describe the current rule-based Agent boundary."""
        print("规则版 Agent 已就绪：只负责任务调度，不直接执行底层实时飞控。")

    def run(self) -> int:
        """Run the interactive rule-based command loop."""
        self.describe()
        print("可用命令：状态、开始任务、停止任务、急停、退出")

        while True:
            try:
                command = input("Agent> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\n收到退出信号，正在安全结束任务。")
                self.tools.close()
                return 0

            if command == "状态":
                self._show_status()
            elif command == "开始任务":
                self._start_task()
            elif command == "停止任务":
                self.tools.stop_task()
            elif command == "急停":
                self.tools.emergency_stop()
            elif command == "退出":
                self.tools.close()
                print("Agent 已退出。")
                return 0
            elif command:
                print("未知命令。可用命令：状态、开始任务、停止任务、急停、退出")

    def _show_status(self) -> None:
        """Print battery, height, and current mode."""
        status = self.tools.get_status()
        print(f"电量：{status['battery']}%")
        print(f"高度：{status['height']} cm")
        print(f"当前模式：{status['mode']}")

    def _start_task(self) -> None:
        """Ask for confirmation before calling the safe start-task tool."""
        allowed, message = self.tools.can_start_task()
        print(message)
        if not allowed:
            return

        answer = input("确认周围安全后输入 yes 开始任务，其他输入取消：").strip()
        if answer != "yes":
            print("已取消开始任务。")
            return

        self.tools.start_follow_task()
