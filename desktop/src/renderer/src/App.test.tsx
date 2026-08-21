import { fireEvent, render, screen, waitFor, within } from '@testing-library/react'
import { beforeEach, describe, expect, it, vi } from 'vitest'
import type { BackendState, DroneStatus, PhantomFilmerApi } from '../../preload/api'
import App from './App'

const readyBackend: BackendState = {
  status: 'ready',
  version: '0.1.0',
  logDir: '/tmp/phantomfilmer/logs',
  airborne: false,
  restartAllowed: true
}

const groundStatus: DroneStatus = {
  battery: 76,
  heightCm: 12,
  frontTofCm: 120,
  frontTofState: 'clear',
  controlHz: 30,
  flightState: '地面待机',
  phase: '检查',
  videoReady: false,
  airborne: false,
  canTakeoff: true,
  rcEnabled: false,
  preflight: { sdk: true, video: true, battery: true, bottomTof: true, frontTof: true }
}

function createApi(overrides: Partial<PhantomFilmerApi> = {}): PhantomFilmerApi {
  return {
    connect: vi.fn().mockResolvedValue(groundStatus),
    status: vi.fn().mockResolvedValue(groundStatus),
    takeoff: vi.fn().mockResolvedValue({ ...groundStatus, airborne: true, rcEnabled: true, canTakeoff: false, phase: '手动飞行', flightState: '手动悬停' }),
    land: vi.fn().mockResolvedValue(groundStatus),
    hover: vi.fn().mockResolvedValue(groundStatus),
    moveRc: vi.fn().mockResolvedValue({ ok: true, flightState: '手动飞行' }),
    stop: vi.fn().mockResolvedValue({ ok: true }),
    emergencyLand: vi.fn().mockResolvedValue({ ok: true }),
    getVideoUrl: vi.fn().mockResolvedValue('http://127.0.0.1:1234/video?token=once'),
    getBackendState: vi.fn().mockResolvedValue(readyBackend),
    restartBackend: vi.fn().mockResolvedValue(readyBackend),
    getRuntimeCapabilities: vi.fn().mockResolvedValue({
      apiVersion: '1',
      commands: ['device.connect', 'flight.takeoff', 'flight.land'],
      missions: ['manual'],
      eventReplay: true,
      rcLease: { required: true, ttlMs: 1000 }
    }),
    getRuntimeSnapshot: vi.fn().mockResolvedValue({
      sequence: 0,
      phase: 'disconnected',
      mission: 'idle',
      controlMode: 'none',
      connected: false,
      airborne: false,
      streaming: false,
      flightState: '未连接',
      allowedActions: ['connect'],
      telemetry: {}
    }),
    getRuntimeEvents: vi.fn().mockResolvedValue({
      apiVersion: '1',
      latestSequence: 0,
      resetRequired: false,
      events: []
    }),
    startMission: vi.fn().mockResolvedValue({ ok: true, mission: 'follow' }),
    stopMission: vi.fn().mockResolvedValue({ ok: true }),
    emergencyStopMission: vi.fn().mockResolvedValue({ ok: true }),
    selectControlMode: vi.fn().mockResolvedValue({ ok: true, mode: 'normal' }),
    toggleMissionPause: vi.fn().mockResolvedValue({ ok: true }),
    listProfiles: vi.fn().mockResolvedValue([]),
    enrollProfile: vi.fn().mockResolvedValue(null),
    onBackendState: vi.fn().mockReturnValue(() => undefined),
    ...overrides
  }
}

describe('desktop flight console', () => {
  beforeEach(() => {
    window.phantomFilmer = createApi()
  })

  it('shows offline diagnostics and disables backend restart after an airborne crash', async () => {
    window.phantomFilmer = createApi({
      getBackendState: vi.fn().mockResolvedValue({
        ...readyBackend,
        status: 'offline',
        airborne: true,
        restartAllowed: false,
        error: '最后一次遥测显示真机在空中'
      })
    })

    render(<App />)

    expect(await screen.findByText('后端中断 · 最后状态为空中')).toBeInTheDocument()
    expect(screen.getByRole('button', { name: /禁止自动重启/ })).toBeDisabled()
    expect(screen.getByRole('button', { name: '连接真机' })).toBeDisabled()
  })

  it('requires a second takeoff click', async () => {
    render(<App />)
    const connect = await screen.findByRole('button', { name: '连接真机' })
    fireEvent.click(connect)
    const takeoff = await screen.findByRole('button', { name: /^起飞/ })

    fireEvent.click(takeoff)
    expect(window.phantomFilmer.takeoff).not.toHaveBeenCalled()
    fireEvent.click(screen.getByRole('button', { name: /^确认起飞/ }))

    await waitFor(() => expect(window.phantomFilmer.takeoff).toHaveBeenCalledTimes(1))
  })

  it('enables keyboard RC only while airborne and hovers on release', async () => {
    const airborne = { ...groundStatus, airborne: true, rcEnabled: true, canTakeoff: false, phase: '手动飞行' as const }
    window.phantomFilmer = createApi({
      connect: vi.fn().mockResolvedValue(airborne),
      status: vi.fn().mockResolvedValue(airborne),
      hover: vi.fn().mockResolvedValue(airborne)
    })
    render(<App />)
    fireEvent.click(await screen.findByRole('button', { name: '连接真机' }))
    await waitFor(() => expect(screen.getByRole('button', { name: /前进/ })).toBeEnabled())

    fireEvent.keyDown(window, { code: 'KeyW' })
    await waitFor(() => expect(window.phantomFilmer.moveRc).toHaveBeenCalledWith({ leftRight: 0, forwardBack: 20, upDown: 0, yaw: 0 }))
    fireEvent.keyUp(window, { code: 'KeyW' })
    await waitFor(() => expect(window.phantomFilmer.hover).toHaveBeenCalled())
  })

  it('restarts a recoverable backend from diagnostics', async () => {
    window.phantomFilmer = createApi({
      getBackendState: vi.fn().mockResolvedValue({ ...readyBackend, status: 'offline', error: 'sidecar missing' })
    })
    render(<App />)
    fireEvent.click(await screen.findByRole('button', { name: /重启后端/ }))

    await waitFor(() => expect(window.phantomFilmer.restartBackend).toHaveBeenCalledTimes(1))
    expect(await screen.findByText('本地后端已恢复，请重新连接真机')).toBeInTheDocument()
  })

  it('shows only mission capabilities reported by the v1 runtime', async () => {
    render(<App />)
    fireEvent.click(await screen.findByRole('button', { name: '任务与人物' }))

    expect(await screen.findByRole('heading', { name: '任务与人物' })).toBeInTheDocument()
    expect(screen.getByRole('button', { name: '前往飞行控制' })).toBeEnabled()
    expect(screen.getAllByRole('button', { name: '等待任务接口' })).toHaveLength(3)
    expect(screen.getAllByText('后端尚未开放')).toHaveLength(3)
  })

  it('renders the authoritative runtime snapshot in diagnostics', async () => {
    window.phantomFilmer = createApi({
      getRuntimeSnapshot: vi.fn().mockResolvedValue({
        sequence: 42,
        phase: 'preflight',
        mission: 'manual',
        controlMode: 'none',
        connected: true,
        airborne: false,
        streaming: true,
        flightState: '地面待机',
        allowedActions: ['refresh_status', 'takeoff'],
        telemetry: {}
      })
    })
    render(<App />)
    fireEvent.click(await screen.findByRole('button', { name: '运行诊断' }))

    expect(await screen.findByText('42')).toBeInTheDocument()
    expect(screen.getByText('refresh_status')).toBeInTheDocument()
    expect(screen.getByText('takeoff')).toBeInTheDocument()
  })

  it('requires profile input and a second click before starting an automatic mission', async () => {
    window.phantomFilmer = createApi({
      getRuntimeCapabilities: vi.fn().mockResolvedValue({
        apiVersion: '1',
        commands: ['mission.start'],
        missions: ['manual', 'follow', 'reid_follow', 'fixed_demo'],
        eventReplay: true,
        rcLease: { required: true, ttlMs: 1000 },
        missionReadiness: { available: true, missingAssets: [], profileRequired: true }
      }),
      listProfiles: vi.fn().mockResolvedValue([{ name: 'operator-a', photoCount: 3 }])
    })
    render(<App />)
    fireEvent.click(await screen.findByRole('button', { name: '任务与人物' }))
    fireEvent.change(await screen.findByRole('combobox', { name: '人物档案' }), { target: { value: 'operator-a' } })
    const followCard = screen.getByRole('heading', { name: '普通自动跟随' }).closest('article')
    expect(followCard).not.toBeNull()
    const start = within(followCard as HTMLElement).getByRole('button', { name: '启动任务' })

    fireEvent.click(start)
    expect(window.phantomFilmer.startMission).not.toHaveBeenCalled()
    fireEvent.click(within(followCard as HTMLElement).getByRole('button', { name: '再次点击确认起飞' }))

    await waitFor(() => expect(window.phantomFilmer.startMission).toHaveBeenCalledWith({
      mission: 'follow',
      profileName: 'operator-a',
      initialControlMode: 'normal',
      obstacleEnabled: false
    }))
  })
})
