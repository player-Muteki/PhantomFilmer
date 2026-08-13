"""Tests for natural-language command parsing via local rules."""

import sys
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from console.command_parser import CommandParser
from console.commands import ConsoleCommand


class CommandParserTestCase(unittest.TestCase):
    """Verify local rule parsing, negation, and uncertainty handling."""

    def test_exact_fixed_command(self) -> None:
        parser = CommandParser()
        self.assertEqual(parser.parse("状态"), ConsoleCommand.GET_STATUS)

    def test_local_status_phrase_works(self) -> None:
        parser = CommandParser()
        self.assertEqual(parser.parse("看看现在无人机状态"), ConsoleCommand.GET_STATUS)

    def test_local_start_phrase_works(self) -> None:
        parser = CommandParser()
        self.assertEqual(parser.parse("帮我开始跟随目标"), ConsoleCommand.START_FOLLOW)

    def test_local_stop_phrase_works(self) -> None:
        parser = CommandParser()
        self.assertEqual(parser.parse("先停一下"), ConsoleCommand.STOP_TASK)

    def test_local_exit_phrase_works(self) -> None:
        parser = CommandParser()
        self.assertEqual(parser.parse("退出系统"), ConsoleCommand.EXIT)

    def test_local_emergency_keyword_works(self) -> None:
        parser = CommandParser()
        self.assertEqual(parser.parse("立即急停"), ConsoleCommand.EMERGENCY_STOP)

    def test_negated_stop_phrase_returns_unknown(self) -> None:
        parser = CommandParser()
        self.assertEqual(parser.parse("先别停一下"), ConsoleCommand.UNKNOWN)

    def test_negated_emergency_phrase_returns_unknown(self) -> None:
        parser = CommandParser()
        self.assertEqual(parser.parse("不要急停"), ConsoleCommand.UNKNOWN)

    def test_unknown_phrase_returns_unknown(self) -> None:
        parser = CommandParser()
        self.assertEqual(parser.parse("帮我看看"), ConsoleCommand.UNKNOWN)

    def test_question_about_starting_returns_unknown(self) -> None:
        parser = CommandParser()
        self.assertEqual(parser.parse("现在可以开始工作了吗"), ConsoleCommand.UNKNOWN)


if __name__ == "__main__":
    unittest.main()
