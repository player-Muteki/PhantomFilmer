"""Natural-language to safe Agent command parsing."""

from typing import Iterable, Optional

from agent.commands import AgentCommand
from agent.llm_client import LLMClient


EXACT_COMMANDS = {
    "状态": AgentCommand.GET_STATUS,
    "开始任务": AgentCommand.START_FOLLOW,
    "停止任务": AgentCommand.STOP_TASK,
    "急停": AgentCommand.EMERGENCY_STOP,
    "退出": AgentCommand.EXIT,
}

NEGATION_MARKERS = (
    "不要",
    "别",
    "不准",
    "先别",
    "先不要",
    "不用",
    "not ",
    "don't ",
    "do not ",
)

UNCERTAIN_MARKERS = (
    "吗",
    "么",
    "？",
    "?",
    "能不能",
    "可不可以",
    "是否",
    "要不要",
    "是不是",
)

LOCAL_PATTERNS = (
    (AgentCommand.EMERGENCY_STOP, (
        "急停",
        "紧急停止",
        "立即急停",
        "马上急停",
        "立刻急停",
        "emergency stop",
    )),
    (AgentCommand.GET_STATUS, (
        "状态",
        "现在无人机怎么样",
        "看看现在无人机状态",
        "查看状态",
        "查询状态",
        "还有多少电",
        "电量多少",
        "当前电量",
        "无人机情况",
    )),
    (AgentCommand.START_FOLLOW, (
        "开始任务",
        "开始跟随",
        "启动跟随",
        "帮我开始跟随目标",
        "开始跟随目标",
        "启动无人机跟随",
        "可以开始工作了",
        "开始工作",
    )),
    (AgentCommand.STOP_TASK, (
        "停止任务",
        "停止当前任务",
        "先停一下",
        "停一下",
        "停止跟随",
        "结束任务",
        "取消任务",
    )),
    (AgentCommand.EXIT, (
        "退出",
        "退出系统",
        "结束系统",
        "关闭系统",
        "退出程序",
    )),
)


class CommandParser:
    """Parse user input into one whitelisted Agent action."""

    def __init__(self, llm_client: Optional[LLMClient] = None) -> None:
        self.llm_client = llm_client

    def parse(self, user_text: str) -> AgentCommand:
        """Resolve an action using local rules first, then LLM fallback."""
        normalized = user_text.strip()
        if not normalized:
            return AgentCommand.UNKNOWN

        exact_command = EXACT_COMMANDS.get(normalized)
        if exact_command is not None:
            return exact_command

        lowered = normalized.lower()
        for action, keywords in LOCAL_PATTERNS:
            if self._matches_local_pattern(lowered, keywords):
                if action != AgentCommand.GET_STATUS and self._is_uncertain_command(lowered):
                    return AgentCommand.UNKNOWN
                return action

        if self.llm_client is None:
            return AgentCommand.UNKNOWN
        return self.llm_client.classify(normalized)

    def _matches_local_pattern(self, user_text: str, keywords: Iterable[str]) -> bool:
        """Return True when a keyword matches without an obvious negation."""
        for keyword in keywords:
            lowered_keyword = keyword.lower()
            if lowered_keyword not in user_text:
                continue
            if self._is_negated(user_text, lowered_keyword):
                return False
            return True
        return False

    @staticmethod
    def _is_negated(user_text: str, keyword: str) -> bool:
        """Return True when a nearby negation marker blocks a local match."""
        keyword_index = user_text.find(keyword)
        if keyword_index < 0:
            return False
        window = user_text[max(0, keyword_index - 6):keyword_index]
        return any(marker in window for marker in NEGATION_MARKERS)

    @staticmethod
    def _is_uncertain_command(user_text: str) -> bool:
        """Return True when the user asks about an action instead of ordering it."""
        return any(marker in user_text for marker in UNCERTAIN_MARKERS)
