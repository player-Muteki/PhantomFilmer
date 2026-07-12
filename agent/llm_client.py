"""OpenAI-compatible LLM client for natural-language command classification."""

import json
import os
from typing import Optional
from urllib import error, request

from agent.commands import AgentCommand


DEFAULT_BASE_URL = "https://api.openai.com/v1/chat/completions"
DEFAULT_MODEL = "gpt-4o-mini"
DEFAULT_TIMEOUT_SECONDS = 8.0
SYSTEM_PROMPT = """你是一个无人机任务命令分类器。\n你只能把用户输入映射为以下动作之一：GET_STATUS, START_FOLLOW, STOP_TASK, EMERGENCY_STOP, EXIT, UNKNOWN。\n如果用户表达含糊、能力范围外、或不能安全确定意图，必须返回 UNKNOWN。\n禁止输出除 JSON 之外的任何内容。\n输出格式必须严格为：{\"action\": \"<ACTION>\"}"""


class LLMClient:
    """Classify natural-language input into one whitelisted Agent action."""

    def __init__(
        self,
        base_url: str = DEFAULT_BASE_URL,
        model: str = DEFAULT_MODEL,
        api_key: Optional[str] = None,
        timeout_seconds: float = DEFAULT_TIMEOUT_SECONDS,
        enabled: bool = True,
    ) -> None:
        self.base_url = base_url
        self.model = model
        self.api_key = api_key if api_key is not None else os.getenv("LLM_API_KEY")
        self.timeout_seconds = timeout_seconds
        self.enabled = enabled

    def is_available(self) -> bool:
        """Return whether online classification is configured and ready."""
        return self.enabled and bool(self.api_key)

    def classify(self, user_text: str) -> AgentCommand:
        """Return one safe action or UNKNOWN when classification is unavailable."""
        if not self.is_available() or not user_text.strip():
            return AgentCommand.UNKNOWN

        payload = {
            "model": self.model,
            "temperature": 0,
            "response_format": {"type": "json_object"},
            "messages": [
                {"role": "system", "content": SYSTEM_PROMPT},
                {"role": "user", "content": user_text},
            ],
        }
        request_data = json.dumps(payload).encode("utf-8")
        http_request = request.Request(
            self.base_url,
            data=request_data,
            headers={
                "Content-Type": "application/json",
                "Authorization": f"Bearer {self.api_key}",
            },
            method="POST",
        )

        try:
            with request.urlopen(http_request, timeout=self.timeout_seconds) as response:
                body = response.read().decode("utf-8")
        except error.HTTPError as exc:
            exc.read()
            return AgentCommand.UNKNOWN
        except (error.URLError, TimeoutError, ValueError):
            return AgentCommand.UNKNOWN

        return self._parse_response(body)

    def _parse_response(self, body: str) -> AgentCommand:
        """Extract one whitelisted action from a chat-completions response body."""
        try:
            response_json = json.loads(body)
            content = response_json["choices"][0]["message"]["content"]
            result = json.loads(content)
            action = str(result.get("action", "")).strip().upper()
        except (KeyError, IndexError, TypeError, json.JSONDecodeError):
            return AgentCommand.UNKNOWN

        try:
            return AgentCommand(action)
        except ValueError:
            return AgentCommand.UNKNOWN
