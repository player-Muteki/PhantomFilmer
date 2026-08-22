import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest'
import { HeartbeatMonitor } from './heartbeat'

describe('HeartbeatMonitor', () => {
  beforeEach(() => { vi.useFakeTimers() })
  afterEach(() => { vi.useRealTimers() })

  function setup(options: { busy?: () => boolean } = {}): {
    monitor: HeartbeatMonitor
    probe: ReturnType<typeof vi.fn>
    onHang: ReturnType<typeof vi.fn>
  } {
    const probe = vi.fn<() => Promise<void>>()
    const onHang = vi.fn()
    const monitor = new HeartbeatMonitor(
      probe,
      options.busy ?? (() => false),
      onHang,
      100,
      3
    )
    return { monitor, probe, onHang }
  }

  async function ticks(count: number): Promise<void> {
    for (let i = 0; i < count; i += 1) await vi.advanceTimersByTimeAsync(100)
  }

  it('does not declare a hang while probes keep succeeding', async () => {
    const { monitor, probe, onHang } = setup()
    probe.mockResolvedValue(undefined)
    monitor.start()
    await ticks(6)
    expect(probe).toHaveBeenCalledTimes(6)
    expect(onHang).not.toHaveBeenCalled()
    monitor.stop()
  })

  it('resets the failure window after a successful probe', async () => {
    const { monitor, probe, onHang } = setup()
    let call = 0
    probe.mockImplementation(() => {
      call += 1
      return call === 3 ? Promise.resolve(undefined) : Promise.reject(new Error('timeout'))
    })
    monitor.start()
    await ticks(5)
    expect(onHang).not.toHaveBeenCalled()
    monitor.stop()
  })

  it('declares a hang once after three consecutive failures and stops probing', async () => {
    const { monitor, probe, onHang } = setup()
    probe.mockRejectedValue(new Error('timeout'))
    monitor.start()
    await ticks(10)
    expect(probe).toHaveBeenCalledTimes(3)
    expect(onHang).toHaveBeenCalledTimes(1)
    monitor.stop()
  })

  it('skips judgment entirely while another request is in flight', async () => {
    const { monitor, probe, onHang } = setup({ busy: () => true })
    probe.mockRejectedValue(new Error('timeout'))
    monitor.start()
    await ticks(5)
    expect(probe).not.toHaveBeenCalled()
    expect(onHang).not.toHaveBeenCalled()
    monitor.stop()
  })

  it('clears the failure window when restarted', async () => {
    const { monitor, probe, onHang } = setup()
    probe.mockRejectedValue(new Error('timeout'))
    monitor.start()
    await ticks(2)
    monitor.stop()
    monitor.start()
    await ticks(2)
    expect(onHang).not.toHaveBeenCalled()
    await ticks(1)
    expect(onHang).toHaveBeenCalledTimes(1)
    monitor.stop()
  })
})
