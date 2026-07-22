"""Natural-language Console for high-level PhantomFilmer task scheduling."""

from typing import Optional

from console.command_parser import CommandParser
from console.commands import ConsoleCommand
from console.llm_client import LLMClient
from console.tools import ConsoleTools


class ConsoleController:
    """Parse safe high-level commands and call only safety-wrapped ConsoleTools."""

    COMMANDS = ("状态", "开始任务", "停止任务", "急停", "退出")

    def __init__(
        self,
        tools: ConsoleTools,
        parser: Optional[CommandParser] = None,
        llm_client: Optional[LLMClient] = None,
    ) -> None:
        self.tools = tools
        self.llm_client = llm_client
        self.parser = parser or CommandParser(llm_client=llm_client)

    def describe(self) -> None:
        """Describe the Console boundary and available interaction style."""
        print("自然语言控制台已就绪：支持固定命令和自然语言任务调度，不直接执行底层实时飞控。")
        if self.llm_client is not None and self.llm_client.enabled and not self.llm_client.is_available():
            print("提示：已启用在线模型解析，但当前未检测到 LLM_API_KEY，将只使用本地规则解析。")

    def run(self) -> int:
        """Run the interactive command loop."""
        self.describe()
        print("可用命令：状态、开始任务、停止任务、急停、退出")
        print("也可以直接输入自然语言，例如：帮我开始跟随目标。")

        while True:
            try:
                user_text = input("Console> ").strip()
            except (EOFError, KeyboardInterrupt):
                print("\n收到退出信号，正在安全结束任务。")
                self.tools.close()
                return 0

            action = self.parser.parse(user_text)
            if action == ConsoleCommand.GET_STATUS:
                self._show_status()
            elif action == ConsoleCommand.START_FOLLOW:
                self._start_task()
            elif action == ConsoleCommand.STOP_TASK:
                self.tools.stop_task()
            elif action == ConsoleCommand.EMERGENCY_STOP:
                self.tools.emergency_stop()
            elif action == ConsoleCommand.EXIT:
                self.tools.close()
                print("控制台已退出。")
                return 0
            elif user_text:
                print("无法安全识别该指令。你可以改说更明确一点，或使用固定命令：状态、开始任务、停止任务、急停、退出")

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
