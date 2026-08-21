"""Exclusive, expiring authority lease for versioned manual RC commands."""

from __future__ import annotations

from dataclasses import dataclass
from math import isfinite
from secrets import token_urlsafe
from threading import RLock
from time import monotonic, time
from typing import Callable, Optional


@dataclass(frozen=True)
class RcLeaseSnapshot:
    lease_id: str
    last_sequence: int
    expires_at_ms: int

    def to_dict(self) -> dict[str, object]:
        return {
            "leaseId": self.lease_id,
            "lastSequence": self.last_sequence,
            "expiresAt": self.expires_at_ms,
        }


@dataclass
class _RcLease:
    lease_id: str
    expires_at_monotonic: float
    expires_at_ms: int
    last_sequence: int = 0
    last_issued_at_ms: float = 0.0


class RcLeaseManager:
    """Issue one sliding lease and reject stale or replayed control packets."""

    def __init__(
        self,
        *,
        ttl_seconds: float = 1.0,
        max_clock_skew_seconds: float = 5.0,
        monotonic_clock: Callable[[], float] = monotonic,
        wall_clock: Callable[[], float] = time,
    ) -> None:
        if ttl_seconds <= 0:
            raise ValueError("ttl_seconds must be positive")
        self._ttl_seconds = float(ttl_seconds)
        self._max_clock_skew_ms = float(max_clock_skew_seconds) * 1000.0
        self._monotonic = monotonic_clock
        self._wall = wall_clock
        self._lock = RLock()
        self._active: Optional[_RcLease] = None

    def acquire(self) -> RcLeaseSnapshot:
        with self._lock:
            now = self._monotonic()
            expires_at_ms = int((self._wall() + self._ttl_seconds) * 1000)
            self._active = _RcLease(
                lease_id=token_urlsafe(24),
                expires_at_monotonic=now + self._ttl_seconds,
                expires_at_ms=expires_at_ms,
            )
            return self._snapshot(self._active)

    def validate_and_refresh(
        self,
        *,
        lease_id: object,
        sequence: object,
        issued_at_ms: object,
    ) -> RcLeaseSnapshot:
        with self._lock:
            lease = self._active
            if lease is None or not isinstance(lease_id, str) or lease_id != lease.lease_id:
                raise RuntimeError("RC 控制租约无效或已释放。")
            now_monotonic = self._monotonic()
            if now_monotonic > lease.expires_at_monotonic:
                self._active = None
                raise RuntimeError("RC 控制租约已过期，请重新获取。")
            if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 1:
                raise RuntimeError("RC sequence 必须是从 1 开始递增的整数。")
            if sequence <= lease.last_sequence:
                raise RuntimeError("RC 指令已过期或乱序，已拒绝执行。")
            if isinstance(issued_at_ms, bool) or not isinstance(issued_at_ms, (int, float)):
                raise RuntimeError("RC issuedAt 时间戳无效。")
            issued = float(issued_at_ms)
            now_ms = self._wall() * 1000.0
            if not isfinite(issued) or abs(now_ms - issued) > self._max_clock_skew_ms:
                raise RuntimeError("RC issuedAt 已过期或超出允许时钟偏差。")
            if issued <= lease.last_issued_at_ms:
                raise RuntimeError("RC issuedAt 不是新鲜时间戳，已拒绝执行。")

            lease.last_sequence = sequence
            lease.last_issued_at_ms = issued
            lease.expires_at_monotonic = now_monotonic + self._ttl_seconds
            lease.expires_at_ms = int((self._wall() + self._ttl_seconds) * 1000)
            return self._snapshot(lease)

    def release(self, lease_id: object) -> bool:
        with self._lock:
            lease = self._active
            if lease is None:
                return False
            if not isinstance(lease_id, str) or lease_id != lease.lease_id:
                raise RuntimeError("不能释放其他客户端持有的 RC 控制租约。")
            self._active = None
            return True

    def revoke(self) -> None:
        with self._lock:
            self._active = None

    @staticmethod
    def _snapshot(lease: _RcLease) -> RcLeaseSnapshot:
        return RcLeaseSnapshot(
            lease_id=lease.lease_id,
            last_sequence=lease.last_sequence,
            expires_at_ms=lease.expires_at_ms,
        )
