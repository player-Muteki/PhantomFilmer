import { act, fireEvent, render, screen, waitFor, within } from '@testing-library/react'
import { beforeEach, describe, expect, it, vi } from 'vitest'
import type { BackendState, DroneStatus, PhantomFilmerApi } from '../../preload/api'
import App from './App'

const readyBackend: BackendState = { status: 'ready', version: '0.1.0', logDir: '/tmp/phantomfilmer/logs', airborne: false, restartAllowed: true }
const groundStatus: DroneStatus = {
  battery: 76, heightCm: 12, frontTofCm: 120, frontTofState: 'clear', controlHz: 30,
  flightState: '地面待机', phase: '检查', videoReady: false, airborne: false, canTakeoff: true, rcEnabled: false,
  preflight: { sdk: true, video: true, battery: true, bottomTof: true, frontTof: true }
}

function runtimeSnapshot(overrides: Record<string, unknown> = {}): ReturnType<PhantomFilmerApi['getRuntimeSnapshot']> {
  return Promise.resolve({ sequence: 1, phase: 'preflight', mission: 'idle', controlMode: 'none', connected: true, airborne: false, streaming: true, flightState: '地面待机', allowedActions: ['start_mission'], telemetry: {}, ...overrides } as never)
}

function createApi(overrides: Partial<PhantomFilmerApi> = {}): PhantomFilmerApi {
  return {
    connect: vi.fn().mockResolvedValue(groundStatus), status: vi.fn().mockResolvedValue(groundStatus),
    takeoff: vi.fn(), land: vi.fn(), hover: vi.fn().mockResolvedValue(groundStatus), moveRc: vi.fn().mockResolvedValue({ ok: true, flightState: '手动飞行' }),
    inputKey: vi.fn().mockResolvedValue({ ok: true, operatorSequence: 1, key: 'm' }), stop: vi.fn(), emergencyLand: vi.fn().mockResolvedValue({ ok: true }),
    getVideoUrl: vi.fn().mockResolvedValue('http://127.0.0.1:1234/video?token=once'), getBackendState: vi.fn().mockResolvedValue(readyBackend), restartBackend: vi.fn().mockResolvedValue(readyBackend),
    getRuntimeCapabilities: vi.fn().mockResolvedValue({ apiVersion: '1', commands: ['mission.start'], missions: ['follow'], eventReplay: true, rcLease: { required: true, ttlMs: 1000 }, missionReadiness: { available: true, missingAssets: [], profileRequired: true } }),
    getRuntimeSnapshot: vi.fn().mockImplementation(() => runtimeSnapshot()), getRuntimeEvents: vi.fn().mockResolvedValue({ apiVersion: '1', latestSequence: 0, resetRequired: false, events: [] }),
    startMission: vi.fn().mockResolvedValue({ ok: true, mission: 'follow' }), stopMission: vi.fn().mockResolvedValue({ ok: true }), emergencyStopMission: vi.fn().mockResolvedValue({ ok: true }), selectControlMode: vi.fn(), toggleMissionPause: vi.fn(),
    listProfiles: vi.fn().mockResolvedValue([{ name: '甲', photoCount: 3 }, { name: '乙', photoCount: 4 }]), enrollProfile: vi.fn().mockResolvedValue(null), startPreview: vi.fn(), stopPreview: vi.fn(), onBackendState: vi.fn().mockReturnValue(() => undefined),
    ...overrides
  }
}

async function connect(): Promise<void> {
  fireEvent.click(await screen.findByRole('button', { name: '连接真机' }))
}

describe('desktop follow console', () => {
  beforeEach(() => { vi.restoreAllMocks(); window.phantomFilmer = createApi() })

  it('keeps only the core single-screen workflow', async () => {
    render(<App />)
    expect(await screen.findByRole('heading', { name: '任务与人物' })).toBeInTheDocument()
    expect(screen.getByRole('heading', { name: '飞行控制' })).toBeInTheDocument()
    expect(screen.getByRole('button', { name: '起飞' })).toBeInTheDocument()
    expect(screen.queryByRole('button', { name: '运行诊断' })).not.toBeInTheDocument()
    expect(screen.queryByText('固定航线演示')).not.toBeInTheDocument()
    expect(screen.queryByText('人物跟拍任务')).not.toBeInTheDocument()
  })

  it('uses the newly selected profile for the mission and keeps it displayed', async () => {
    render(<App />)
    await screen.findByRole('option', { name: '乙' })
    const profile = screen.getByRole('combobox', { name: '人物档案' })
    fireEvent.change(profile, { target: { value: '乙' } })
    expect(profile).toHaveValue('乙')
    expect(screen.getAllByText('乙').length).toBeGreaterThan(0)

    await connect()
    const launch = screen.getByRole('button', { name: '起飞' })
    await waitFor(() => expect(launch).toBeEnabled())
    fireEvent.click(launch)
    fireEvent.click(screen.getByRole('button', { name: '确认起飞' }))

    await waitFor(() => expect(window.phantomFilmer.startMission).toHaveBeenCalledWith({ mission: 'follow', profileName: '乙', initialControlMode: 'manual', obstacleEnabled: false }))
    expect(screen.getByRole('combobox', { name: '人物档案' })).toHaveValue('乙')
  })

  it('loads persisted profiles after the backend transitions from starting to ready', async () => {
    let backendListener: ((state: BackendState) => void) | undefined
    const listProfiles = vi.fn().mockResolvedValue([{ name: '陈苇航', photoCount: 4 }])
    window.phantomFilmer = createApi({
      getBackendState: vi.fn().mockResolvedValue({ ...readyBackend, status: 'starting', restartAllowed: false }),
      listProfiles,
      onBackendState: vi.fn().mockImplementation((listener) => {
        backendListener = listener
        return () => undefined
      })
    })
    render(<App />)
    await waitFor(() => expect(backendListener).toBeDefined())
    expect(listProfiles).not.toHaveBeenCalled()

    await act(async () => { backendListener?.(readyBackend) })

    expect(await screen.findByRole('option', { name: '陈苇航' })).toBeInTheDocument()
    expect(screen.getByRole('combobox', { name: '人物档案' })).toHaveValue('陈苇航')
  })

  it('asks before overwriting an existing profile and forwards the overwrite choice', async () => {
    render(<App />)
    await screen.findByRole('option', { name: '甲' })
    fireEvent.change(screen.getByRole('textbox', { name: '新人物档案名' }), { target: { value: '甲' } })
    const confirm = vi.spyOn(window, 'confirm').mockReturnValue(true)
    fireEvent.click(screen.getByRole('button', { name: '新建档案' }))

    expect(confirm).toHaveBeenCalledWith('人物档案“甲”已存在。\n\n是否选择新的参考照片并覆盖原档案？')
    await waitFor(() => expect(window.phantomFilmer.enrollProfile).toHaveBeenCalledWith('甲', true))
  })

  it('does not open the photo chooser when overwrite is cancelled', async () => {
    render(<App />)
    await screen.findByRole('option', { name: '甲' })
    fireEvent.change(screen.getByRole('textbox', { name: '新人物档案名' }), { target: { value: '甲' } })
    vi.spyOn(window, 'confirm').mockReturnValue(false)
    fireEvent.click(screen.getByRole('button', { name: '新建档案' }))

    expect(window.phantomFilmer.enrollProfile).not.toHaveBeenCalled()
  })

  it('does not show or require ground preview before launch', async () => {
    render(<App />)
    expect(screen.queryByRole('region', { name: '地面人物识别预览' })).not.toBeInTheDocument()
    expect(window.phantomFilmer.startPreview).not.toHaveBeenCalled()
  })

  it('keeps mode buttons directly above the video and sends manual takeover', async () => {
    window.phantomFilmer = createApi({
      getRuntimeSnapshot: vi.fn().mockImplementation(() => runtimeSnapshot({ mission: 'follow', controlMode: 'normal', airborne: true, phase: 'airborne', flightState: 'FOLLOWING' })),
      connect: vi.fn().mockResolvedValue({ ...groundStatus, airborne: true, canTakeoff: false, rcEnabled: false, phase: '手动飞行', flightState: 'FOLLOWING' })
    })
    render(<App />)
    await connect()
    const controls = await screen.findByLabelText('自动任务空中控制')
    fireEvent.click(within(controls).getByRole('button', { name: '手动接管' }))
    await waitFor(() => expect(window.phantomFilmer.inputKey).toHaveBeenCalledWith('m'))
  })

  it('allows the default normal mode to be selected at control ready', async () => {
    window.phantomFilmer = createApi({
      getRuntimeSnapshot: vi.fn().mockImplementation(() => runtimeSnapshot({ mission: 'follow', controlMode: 'normal', airborne: true, phase: 'airborne', flightState: 'CONTROL_READY' })),
      connect: vi.fn().mockResolvedValue({ ...groundStatus, airborne: true, canTakeoff: false, rcEnabled: false, phase: '手动飞行', flightState: 'CONTROL_READY' })
    })
    render(<App />)
    await connect()
    const normal = await screen.findByRole('button', { name: '普通跟随' })
    expect(normal).toBeEnabled()
    fireEvent.click(normal)
    await waitFor(() => expect(window.phantomFilmer.inputKey).toHaveBeenCalledWith('1'))
  })

  it('keeps keyboard RC available only after manual takeover is active', async () => {
    const airborne = { ...groundStatus, airborne: true, rcEnabled: true, canTakeoff: false, phase: '手动飞行' as const }
    window.phantomFilmer = createApi({ connect: vi.fn().mockResolvedValue(airborne), status: vi.fn().mockResolvedValue(airborne) })
    render(<App />)
    await connect()
    await waitFor(() => expect(screen.getByRole('button', { name: /前进/ })).toBeEnabled())
    fireEvent.keyDown(window, { code: 'KeyW' })
    await waitFor(() => expect(window.phantomFilmer.moveRc).toHaveBeenCalledWith({ leftRight: 0, forwardBack: 20, upDown: 0, yaw: 0 }))
    fireEvent.keyUp(window, { code: 'KeyW' })
    await waitFor(() => expect(window.phantomFilmer.hover).toHaveBeenCalled())
  })

  it('shows recovery controls when the backend is offline', async () => {
    window.phantomFilmer = createApi({ getBackendState: vi.fn().mockResolvedValue({ ...readyBackend, status: 'offline', error: 'sidecar missing' }) })
    render(<App />)
    fireEvent.click(await screen.findByRole('button', { name: '重启后端' }))
    await waitFor(() => expect(window.phantomFilmer.restartBackend).toHaveBeenCalledTimes(1))
    expect(await screen.findByText('本地后端已恢复，请重新连接真机。')).toBeInTheDocument()
  })
})
