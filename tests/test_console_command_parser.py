"""Tests for natural-language command parsing."""

import json
import sys
import threading
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from console.command_parser import CommandParser
from console.commands import ConsoleCommand
from console.llm_client import LLMClient


class FakeLLMClient:
    """Return a fixed action for parser tests."""

    def __init__(self, action: ConsoleCommand) -> None:
        self.action = action

    def classify(self, user_text: str) -> ConsoleCommand:
        return self.action


class CommandParserTestCase(unittest.TestCase):
    """Verify local parsing, emergency fallback, and LLM handoff."""

    def test_exact_fixed_command(self) -> None:
        parser = CommandParser()
        self.assertEqual(parser.parse("状态"), ConsoleCommand.GET_STATUS)

    def test_local_status_phrase_works_without_llm(self) -> None:
        parser = CommandParser()
        self.assertEqual(parser.parse("看看现在无人机状态"), ConsoleCommand.GET_STATUS)

    def test_local_start_phrase_works_without_llm(self) -> None:
        parser = CommandParser()
        self.assertEqual(parser.parse("帮我开始跟随目标"), ConsoleCommand.START_FOLLOW)

    def test_local_stop_phrase_works_without_llm(self) -> None:
        parser = CommandParser()
        self.assertEqual(parser.parse("先停一下"), ConsoleCommand.STOP_TASK)

    def test_local_exit_phrase_works_without_llm(self) -> None:
        parser = CommandParser()
        self.assertEqual(parser.parse("退出系统"), ConsoleCommand.EXIT)

    def test_local_emergency_keyword_bypasses_llm(self) -> None:
        parser = CommandParser(llm_client=FakeLLMClient(ConsoleCommand.START_FOLLOW))
        self.assertEqual(parser.parse("立即急停"), ConsoleCommand.EMERGENCY_STOP)

    def test_negated_stop_phrase_returns_unknown(self) -> None:
        parser = CommandParser()
        self.assertEqual(parser.parse("先别停一下"), ConsoleCommand.UNKNOWN)

    def test_negated_emergency_phrase_returns_unknown(self) -> None:
        parser = CommandParser()
        self.assertEqual(parser.parse("不要急停"), ConsoleCommand.UNKNOWN)

    def test_unknown_without_llm(self) -> None:
        parser = CommandParser()
        self.assertEqual(parser.parse("帮我看看"), ConsoleCommand.UNKNOWN)

    def test_question_about_starting_returns_unknown(self) -> None:
        parser = CommandParser()
        self.assertEqual(parser.parse("现在可以开始工作了吗"), ConsoleCommand.UNKNOWN)

    def test_natural_language_uses_llm_result(self) -> None:
        parser = CommandParser(llm_client=FakeLLMClient(ConsoleCommand.START_FOLLOW))
        self.assertEqual(parser.parse("请进入目标追踪模式"), ConsoleCommand.START_FOLLOW)


class LLMClientTestCase(unittest.TestCase):
    """Verify response parsing and safe failure behavior."""

    def test_parse_valid_response(self) -> None:
        client = LLMClient(enabled=False, api_key="token")
        action = client._parse_response(
            '{"choices": [{"message": {"content": "{\\"action\\": \\\"GET_STATUS\\\"}"}}]}'
        )
        self.assertEqual(action, ConsoleCommand.GET_STATUS)

    def test_parse_invalid_action_returns_unknown(self) -> None:
        client = LLMClient(enabled=False, api_key="token")
        action = client._parse_response(
            '{"choices": [{"message": {"content": "{\\"action\\": \\\"FLY_FAST\\\"}"}}]}'
        )
        self.assertEqual(action, ConsoleCommand.UNKNOWN)

    def test_disabled_client_returns_unknown(self) -> None:
        client = LLMClient(enabled=False, api_key="token")
        self.assertEqual(client.classify("帮我开始跟随目标"), ConsoleCommand.UNKNOWN)

    def test_enabled_client_without_key_is_unavailable(self) -> None:
        client = LLMClient(enabled=True, api_key="")
        self.assertFalse(client.is_available())


class LLMClientHTTPIntegrationTestCase(unittest.TestCase):
    """Verify OpenAI-compatible HTTP classification over a local server."""

    def start_server(self, response_body: dict, status_code: int = 200):
        class Handler(BaseHTTPRequestHandler):
            last_request = None

            def do_POST(self):
                content_length = int(self.headers.get("Content-Length", "0"))
                body = self.rfile.read(content_length).decode("utf-8")
                Handler.last_request = {
                    "path": self.path,
                    "headers": dict(self.headers.items()),
                    "body": json.loads(body),
                }
                self.send_response(status_code)
                self.send_header("Content-Type", "application/json")
                self.end_headers()
                self.wfile.write(json.dumps(response_body).encode("utf-8"))

            def log_message(self, format, *args):
                return

        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        return server, Handler

    def test_http_classification_uses_expected_payload_and_headers(self) -> None:
        server, handler = self.start_server(
            {
                "choices": [
                    {"message": {"content": '{"action": "GET_STATUS"}'}}
                ]
            }
        )
        try:
            client = LLMClient(
                base_url=f"http://127.0.0.1:{server.server_port}/v1/chat/completions",
                model="test-model",
                api_key="secret-token",
                enabled=True,
                timeout_seconds=3,
            )
            action = client.classify("看看现在无人机状态")
        finally:
            server.shutdown()
            server.server_close()

        self.assertEqual(action, ConsoleCommand.GET_STATUS)
        self.assertEqual(handler.last_request["path"], "/v1/chat/completions")
        self.assertEqual(handler.last_request["headers"]["Authorization"], "Bearer secret-token")
        self.assertEqual(handler.last_request["body"]["model"], "test-model")
        self.assertEqual(handler.last_request["body"]["response_format"], {"type": "json_object"})
        self.assertEqual(handler.last_request["body"]["messages"][1]["content"], "看看现在无人机状态")

    def test_http_invalid_json_content_returns_unknown(self) -> None:
        server, _handler = self.start_server(
            {
                "choices": [
                    {"message": {"content": "not-json"}}
                ]
            }
        )
        try:
            client = LLMClient(
                base_url=f"http://127.0.0.1:{server.server_port}/v1/chat/completions",
                model="test-model",
                api_key="secret-token",
                enabled=True,
                timeout_seconds=3,
            )
            action = client.classify("帮我开始跟随目标")
        finally:
            server.shutdown()
            server.server_close()

        self.assertEqual(action, ConsoleCommand.UNKNOWN)

    def test_http_error_status_returns_unknown(self) -> None:
        server, _handler = self.start_server({"error": "bad request"}, status_code=500)
        try:
            client = LLMClient(
                base_url=f"http://127.0.0.1:{server.server_port}/v1/chat/completions",
                model="test-model",
                api_key="secret-token",
                enabled=True,
                timeout_seconds=3,
            )
            action = client.classify("帮我开始跟随目标")
        finally:
            server.shutdown()
            server.server_close()

        self.assertEqual(action, ConsoleCommand.UNKNOWN)


if __name__ == "__main__":
    unittest.main()
