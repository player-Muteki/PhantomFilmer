export type HeartbeatProbe = () => Promise<void>
export type HeartbeatHangHandler = () => void

export const HEARTBEAT_INTERVAL_MS = 5_000
export const HEARTBEAT_TIMEOUT_MS = 3_000
export const HEARTBEAT_FAILURE_LIMIT = 3

/**
 * Detect a sidecar that is still alive but no longer serving requests. A
 * hung process never exits, so liveness of the PID is not enough. Failures
 * are only counted while no other request is in flight: long commands
 * legitimately hold the service for seconds and must not look like a hang.
 */
export class HeartbeatMonitor {
  private timer: NodeJS.Timeout | null = null
  private failures = 0

  constructor(
    private readonly probe: HeartbeatProbe,
    private readonly isBusy: () => boolean,
    private readonly onHang: HeartbeatHangHandler,
    private readonly intervalMs: number = HEARTBEAT_INTERVAL_MS,
    private readonly failureLimit: number = HEARTBEAT_FAILURE_LIMIT
  ) {}

  start(): void {
    this.stop()
    this.failures = 0
    this.timer = setInterval(() => void this.tick(), this.intervalMs)
  }

  stop(): void {
    if (this.timer != null) {
      clearInterval(this.timer)
      this.timer = null
    }
  }

  private async tick(): Promise<void> {
    if (this.isBusy()) return
    try {
      await this.probe()
      this.failures = 0
    } catch {
      this.failures += 1
      if (this.failures >= this.failureLimit) {
        this.stop()
        this.onHang()
      }
    }
  }
}
