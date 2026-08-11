"""Tests for the per-run CLI obstacle-avoidance selection."""

import argparse
import unittest
from unittest.mock import patch

import main
from app import config as app_config


class ObstaclePromptTestCase(unittest.TestCase):
    def test_affirmative_inputs_enable_obstacle_avoidance(self) -> None:
        for answer in ("y", "yes", "是", "开启"):
            with self.subTest(answer=answer), patch("builtins.input", return_value=answer):
                self.assertTrue(main.prompt_obstacle_enabled(False))

    def test_negative_inputs_disable_obstacle_avoidance(self) -> None:
        for answer in ("n", "no", "否", "关闭"):
            with self.subTest(answer=answer), patch("builtins.input", return_value=answer):
                self.assertFalse(main.prompt_obstacle_enabled(True))

    def test_empty_input_uses_configured_default(self) -> None:
        with patch("builtins.input", return_value=""):
            self.assertTrue(main.prompt_obstacle_enabled(True))
        with patch("builtins.input", return_value=""):
            self.assertFalse(main.prompt_obstacle_enabled(False))

    def test_invalid_input_reprompts(self) -> None:
        with patch("builtins.input", side_effect=("invalid", "y")) as prompt:
            self.assertTrue(main.prompt_obstacle_enabled(False))
        self.assertEqual(prompt.call_count, 2)

    def test_end_of_input_cancels(self) -> None:
        with patch("builtins.input", side_effect=EOFError):
            self.assertIsNone(main.prompt_obstacle_enabled(False))

    def test_keyboard_interrupt_cancels(self) -> None:
        with patch("builtins.input", side_effect=KeyboardInterrupt):
            self.assertIsNone(main.prompt_obstacle_enabled(False))


class RuntimeObstacleConfigTestCase(unittest.TestCase):
    def test_override_changes_only_runtime_obstacle_enabled(self) -> None:
        configured = {
            "camera_width": 640,
            "obstacle": {
                "enabled": False,
                "minimum_obstacle_area": 321,
            },
        }
        with patch.object(app_config, "load_config", return_value=configured):
            runtime = main.load_runtime_config(True)

        self.assertTrue(runtime["obstacle"]["enabled"])
        self.assertEqual(runtime["obstacle"]["minimum_obstacle_area"], 321)
        self.assertEqual(runtime["camera_width"], 640)
        self.assertFalse(configured["obstacle"]["enabled"])
        self.assertIsNot(runtime, configured)
        self.assertIsNot(runtime["obstacle"], configured["obstacle"])

    def test_false_override_replaces_true_default(self) -> None:
        configured = {"obstacle": {"enabled": True}}
        with patch.object(app_config, "load_config", return_value=configured):
            runtime = main.load_runtime_config(False)
        self.assertFalse(runtime["obstacle"]["enabled"])
        self.assertTrue(configured["obstacle"]["enabled"])

    def test_no_override_preserves_loaded_config(self) -> None:
        configured = {"obstacle": {"enabled": True}}
        with patch.object(app_config, "load_config", return_value=configured):
            runtime = main.load_runtime_config()
        self.assertIs(runtime, configured)

    def test_invalid_or_missing_obstacle_block_defaults_to_disabled(self) -> None:
        self.assertFalse(main.configured_obstacle_enabled({}))
        self.assertFalse(main.configured_obstacle_enabled({"obstacle": "invalid"}))


class ObstacleCliDispatchTestCase(unittest.TestCase):
    def args(self, mode: str) -> argparse.Namespace:
        return argparse.Namespace(mode=mode, fake=True)

    def test_follow_modes_prompt_and_forward_selection(self) -> None:
        cases = (
            ("follow", "run_follow"),
            ("follow-dry-run", "run_follow_dry_run"),
            ("console", "run_console"),
        )
        for mode, runner_name in cases:
            with self.subTest(mode=mode), patch.object(
                main, "parse_args", return_value=self.args(mode)
            ), patch.object(
                main, "load_config", return_value={"obstacle": {"enabled": False}}
            ), patch.object(
                main, "prompt_obstacle_enabled", return_value=True
            ) as prompt, patch.object(
                main, runner_name, return_value=17
            ) as runner:
                self.assertEqual(main.main(), 17)
            prompt.assert_called_once_with(False)
            runner.assert_called_once_with(use_fake=True, obstacle_enabled=True)

    def test_unrelated_modes_do_not_prompt(self) -> None:
        cases = (
            ("status", "run_status"),
            ("camera", "run_camera"),
            ("swarm-status", "run_swarm_status"),
        )
        for mode, runner_name in cases:
            with self.subTest(mode=mode), patch.object(
                main, "parse_args", return_value=self.args(mode)
            ), patch.object(main, "prompt_obstacle_enabled") as prompt, patch.object(
                main, runner_name, return_value=0
            ) as runner:
                self.assertEqual(main.main(), 0)
            prompt.assert_not_called()
            runner.assert_called_once_with(use_fake=True)

    def test_cancelled_prompt_does_not_start_any_follow_mode(self) -> None:
        with patch.object(
            main, "parse_args", return_value=self.args("follow")
        ), patch.object(
            main, "load_config", return_value={"obstacle": {"enabled": False}}
        ), patch.object(
            main, "prompt_obstacle_enabled", return_value=None
        ), patch.object(main, "run_follow") as run_follow, patch.object(
            main, "run_follow_dry_run"
        ) as run_dry_run, patch.object(main, "run_console") as run_console:
            self.assertEqual(main.main(), 0)
        run_follow.assert_not_called()
        run_dry_run.assert_not_called()
        run_console.assert_not_called()


if __name__ == "__main__":
    unittest.main()
