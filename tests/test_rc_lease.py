import unittest

from app.runtime.rc_lease import RcLeaseManager


class RcLeaseManagerTestCase(unittest.TestCase):
    def setUp(self) -> None:
        self.monotonic_now = 10.0
        self.wall_now = 1000.0
        self.manager = RcLeaseManager(
            ttl_seconds=1.0,
            monotonic_clock=lambda: self.monotonic_now,
            wall_clock=lambda: self.wall_now,
        )

    def test_accepts_fresh_in_order_packets_and_extends_expiry(self) -> None:
        lease = self.manager.acquire()
        self.monotonic_now += 0.5
        self.wall_now += 0.5

        refreshed = self.manager.validate_and_refresh(
            lease_id=lease.lease_id,
            sequence=1,
            issued_at_ms=self.wall_now * 1000,
        )

        self.assertEqual(refreshed.last_sequence, 1)
        self.assertGreater(refreshed.expires_at_ms, lease.expires_at_ms)

    def test_rejects_replay_out_of_order_and_stale_timestamp(self) -> None:
        lease = self.manager.acquire()
        issued_at = self.wall_now * 1000
        self.manager.validate_and_refresh(
            lease_id=lease.lease_id,
            sequence=2,
            issued_at_ms=issued_at,
        )

        with self.assertRaisesRegex(RuntimeError, "过期或乱序"):
            self.manager.validate_and_refresh(
                lease_id=lease.lease_id,
                sequence=2,
                issued_at_ms=issued_at + 1,
            )
        with self.assertRaisesRegex(RuntimeError, "issuedAt"):
            self.manager.validate_and_refresh(
                lease_id=lease.lease_id,
                sequence=3,
                issued_at_ms=issued_at - 6000,
            )

    def test_expired_or_released_lease_cannot_be_reused(self) -> None:
        expired = self.manager.acquire()
        self.monotonic_now += 1.1
        self.wall_now += 1.1
        with self.assertRaisesRegex(RuntimeError, "已过期"):
            self.manager.validate_and_refresh(
                lease_id=expired.lease_id,
                sequence=1,
                issued_at_ms=self.wall_now * 1000,
            )

        released = self.manager.acquire()
        self.assertTrue(self.manager.release(released.lease_id))
        with self.assertRaisesRegex(RuntimeError, "无效或已释放"):
            self.manager.validate_and_refresh(
                lease_id=released.lease_id,
                sequence=1,
                issued_at_ms=self.wall_now * 1000,
            )


if __name__ == "__main__":
    unittest.main()
