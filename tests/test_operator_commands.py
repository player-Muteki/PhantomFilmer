"""Tests for the headless operator command mailbox."""

import unittest
from unittest.mock import Mock

from control.operator_commands import OperatorCommand, OperatorCommandChannel
from vision.camera import CameraStream


class OperatorCommandChannelTestCase(unittest.TestCase):
    def test_bounded_channel_discards_the_oldest_non_safety_command(self) -> None:
        channel = OperatorCommandChannel(capacity=2)
        channel.submit(OperatorCommand.MOVE_FORWARD)
        channel.submit(OperatorCommand.MOVE_LEFT)
        newest = channel.submit(OperatorCommand.MOVE_RIGHT)

        self.assertEqual(channel.pending_count, 2)
        self.assertEqual(channel.receive().command, OperatorCommand.MOVE_LEFT)
        self.assertEqual(channel.receive(), newest)
        self.assertIsNone(channel.receive())

    def test_emergency_discards_pending_motion_and_is_delivered_next(self) -> None:
        channel = OperatorCommandChannel()
        channel.submit(OperatorCommand.MOVE_FORWARD)
        emergency = channel.submit(OperatorCommand.EMERGENCY_STOP)

        self.assertEqual(channel.pending_count, 1)
        self.assertEqual(channel.receive(), emergency)

    def test_sequence_stays_monotonic_across_clear(self) -> None:
        channel = OperatorCommandChannel()
        first = channel.submit(OperatorCommand.HOVER)
        channel.clear()
        second = channel.submit(OperatorCommand.STOP)

        self.assertGreater(second.sequence, first.sequence)

    def test_filtered_receive_preserves_a_mode_choice_until_control_ready(self) -> None:
        channel = OperatorCommandChannel()
        mode = channel.submit(OperatorCommand.SELECT_SIDE)
        hover = channel.submit(OperatorCommand.HOVER)

        self.assertEqual(
            channel.receive({OperatorCommand.HOVER}),
            hover,
        )
        self.assertEqual(channel.receive(), mode)

    def test_attached_camera_does_not_restart_or_stop_an_owned_stream(self) -> None:
        drone = Mock()
        camera = CameraStream(drone, manage_stream=False)

        camera.start()
        camera.stop()

        drone.stream_on.assert_not_called()
        drone.stream_off.assert_not_called()


if __name__ == "__main__":
    unittest.main()
