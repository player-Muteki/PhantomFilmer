import { fireEvent, render, screen, waitFor, within } from '@testing-library/react'
import { beforeEach, describe, expect, it, vi } from 'vitest'
import type { BackendState, DroneStatus, PhantomFilmerApi } from '../../preload/api'
import App from './App'

const readyBackend: BackendState = { status: 'ready', version: '0.1.0', logDir: '/tmp/phantomfilmer/logs', airborne: false, restartAllowed: true }
const groundStatus: DroneStatus = {
  battery: 76, heightCm: 12, frontTofCm: 120, frontTofState: 'clear', controlHz: 30,
  flightState: '地面待机', phase: '检查', videoReady: false, airborne: false, canTakeoff: true, rcEnabled: false,
  preflight: { sdk: true, video: true, battery: true, bottomTof: true, frontTof: true }
}
const missionAllowed = ['stop_mission', 'emergency_stop_mission', 'select_control_mode', 'toggle_mission_pause']

function runtimeSnapshot(overrides: Record<string, unknown> = {}): ReturnType<PhantomFilmerApi['getRuntimeSnapshot']> {
  return Promise.resolve({ sequence: 1, phase: 'preflight', mission: 'idle', controlMode: 'none', connected: true, airborne: false, streaming: true, flightState: '地面待机', allowedActions: ['start_mission'], telemetry: {}, ...overrides } as never)
}

function createApi(overrides: Partial<PhantomFilmerApi> = {}): PhantomFilmerApi {
  return {
    connect: vi.fn().mockResolvedValue(groundStatus), status: vi.fn().mockResolvedValue(groundStatus),
    takeoff: vi.fn(), land: vi.fn(), hover: vi.fn().mockResolvedValue(groundStatus), moveRc: vi.fn().mockResolvedValue({ ok: true, flightState: '手动飞行' }),
    inputKey: vi.fn().mockResolvedValue({ ok: true, operatorSequence: 1, key: 'm' }), stop: vi.fn().mockResolvedValue({ ok: true }), emergencyLand: vi.fn().mockResolvedValue({ ok: true }),
    getVideoUrl: vi.fn().mockResolvedValue('http://127.0.0.1:1234/video?token=once'), getBackendState: vi.fn().mockResolvedValue(readyBackend), restartBackend: vi.fn().mockResolvedValue(readyBackend),
    getRuntimeCapabilities: vi.fn().mockResolvedValue({ apiVersion: '1', commands: ['mission.start'], missions: ['follow'], eventReplay: true, rcLease: { required: true, ttlMs: 1000 }, safety: { minTakeoffBattery: 20, lowBatteryLand: 5, maxHeightCm: 220, maxRcSpeed: 35, minDescentHeightCm: 40, maxAscentHeightCm: 200, frontStopDistanceCm: 60, telemetryMaxAgeMs: 3000 }, missionReadiness: { available: true, missingAssets: [], profileRequired: true } }),
    getRuntimeSnapshot: vi.fn().mockImplementation(() => runtimeSnapshot()), getRuntimeEvents: vi.fn().mockResolvedValue({ apiVersion: '1', latestSequence: 0, resetRequired: false, events: [] }),
    startMission: vi.fn().mockResolvedValue({ ok: true, mission: 'follow' }), stopMission: vi.fn().mockResolvedValue({ ok: true }), emergencyStopMission: vi.fn().mockResolvedValue({ ok: true }), selectControlMode: vi.fn(), toggleMissionPause: vi.fn().mockResolvedValue({ ok: true }),
    listProfiles: vi.fn().mockResolvedValue([{ name: '甲', photoCount: 3 }, { name: '乙', photoCount: 4 }]),
    getProfile: vi.fn().mockImplementation(async (name: string) => ({ name, photoCount: name === '甲' ? 3 : 4, photos: [] })),
    pickProfilePhotos: vi.fn().mockResolvedValue(['/tmp/a.jpg', '/tmp/b.jpg']), enrollProfile: vi.fn().mockResolvedValue(null),
    renameProfile: vi.fn().mockImplementation(async (_name: string, newName: string) => ({ name: newName, photoCount: 3, photos: [] })),
    deleteProfile: vi.fn().mockImplementation(async (name: string) => ({ name, deletedAt: new Date().toISOString(), recoverable: true })),
    openLogDir: vi.fn().mockResolvedValue(undefined), startPreview: vi.fn(), stopPreview: vi.fn(), onBackendState: vi.fn().mockReturnValue(() => undefined),
    ...overrides
  }
}

function mount(overrides: Partial<PhantomFilmerApi> = {}): void {
  window.phantomFilmer = createApi(overrides)
  render(<App />)
}

async function connectDrone(): Promise<void> {
  fireEvent.click(await screen.findByRole('button', { name: '连接真机' }))
  await screen.findByText('真机已连接')
}

function switchTab(name: string): void {
  fireEvent.click(screen.getByRole('tab', { name }))
}

async function startMissionFlow(): Promise<void> {
  const launch = screen.getByRole('button', { name: '起飞' })
  await waitFor(() => expect(launch).toBeEnabled())
  fireEvent.click(launch)
  fireEvent.click(screen.getByRole('button', { name: '确认起飞' }))
  await waitFor(() => expect(window.phantomFilmer.startMission).toHaveBeenCalled())
}

async function enterMission(missionOverrides: Record<string, unknown> = {}, statusOverrides: Partial<DroneStatus> = {}): Promise<void> {
  mount({
    getRuntimeSnapshot: vi.fn().mockImplementation(() => runtimeSnapshot({
      mission: 'follow', controlMode: 'normal', airborne: true, phase: 'airborne', flightState: 'FOLLOWING',
      allowedActions: missionAllowed, ...missionOverrides
    })),
    connect: vi.fn().mockResolvedValue({ ...groundStatus, airborne: true, canTakeoff: false, rcEnabled: false, phase: '手动飞行', flightState: 'FOLLOWING', ...statusOverrides })
  })
  await connectDrone()
  await screen.findByLabelText('自动任务空中控制')
}

describe('desktop follow console', () => {
  beforeEach(() => { vi.restoreAllMocks(); window.phantomFilmer = createApi() })

  it('renders the operator console with setup tabs and diagnostics entry', async () => {
    mount()
    expect(await screen.findByRole('heading', { name: '任务与人物' })).toBeInTheDocument()
    expect(screen.getByRole('button', { name: '起飞' })).toBeInTheDocument()
    expect(screen.queryByRole('button', { name: '日志' })).not.toBeInTheDocument()
    expect(screen.queryByRole('button', { name: '运行诊断' })).not.toBeInTheDocument()
  })

  it('uses one compact command bar above the video and one safety bar below it', async () => {
    mount()
    await screen.findByRole('heading', { name: '任务与人物' })
    expect(document.querySelectorAll('.topbar > .flight-command-bar')).toHaveLength(1)
    expect(document.querySelectorAll('.flight-panel > .flight-footer')).toHaveLength(1)
    expect(document.querySelector('.flight-strip')).not.toBeInTheDocument()
    expect(document.querySelector('.mode-row')).not.toBeInTheDocument()
    const footer = screen.getByLabelText('遥测与飞行安全控制')
    expect(within(footer).getByRole('status')).toHaveTextContent('本地后端已就绪，请连接真机')
  })

  it('keeps all three setup tabs mounted and reveals content on switch', async () => {
    mount()
    const profilesTab = await screen.findByRole('tab', { name: '人物档案' })
    expect(profilesTab).toHaveAttribute('aria-selected', 'true')
    expect(screen.getByRole('tabpanel', { name: '人物档案' })).toBeVisible()
    expect(screen.queryByRole('tabpanel', { name: '起飞准备' })).toBeNull()
    expect(screen.queryByRole('tabpanel', { name: '运行事件' })).toBeNull()

    switchTab('起飞准备')
    expect(screen.getByRole('tab', { name: '起飞准备' })).toHaveAttribute('aria-selected', 'true')
    expect(document.querySelector('.mission-type')).not.toBeInTheDocument()
    expect(screen.getByText('前向 ToF 安全保护')).toBeVisible()
    expect(screen.queryByRole('checkbox', { name: /ToF/ })).not.toBeInTheDocument()
    expect(screen.getByLabelText('起飞预检清单')).toBeVisible()

    switchTab('运行事件')
    expect(screen.getByRole('tabpanel', { name: '运行事件' })).toBeVisible()
    switchTab('人物档案')
    expect(screen.getByRole('combobox', { name: '人物档案' })).toBeVisible()
  })

  it('supports arrow-key navigation between setup tabs', async () => {
    mount()
    const profilesTab = await screen.findByRole('tab', { name: '人物档案' })
    profilesTab.focus()
    fireEvent.keyDown(profilesTab, { key: 'ArrowRight' })
    expect(screen.getByRole('tab', { name: '起飞准备' })).toHaveAttribute('aria-selected', 'true')
    fireEvent.keyDown(screen.getByRole('tab', { name: '起飞准备' }), { key: 'ArrowLeft' })
    expect(screen.getByRole('tab', { name: '人物档案' })).toHaveAttribute('aria-selected', 'true')
  })

  it('uses the newly selected profile for the mission and keeps it displayed', async () => {
    mount()
    await screen.findByRole('option', { name: '乙' })
    const profile = screen.getByRole('combobox', { name: '人物档案' })
    fireEvent.change(profile, { target: { value: '乙' } })
    expect(profile).toHaveValue('乙')
    expect(screen.getAllByText('乙').length).toBeGreaterThan(0)

    await connectDrone()
    await startMissionFlow()

    await waitFor(() => expect(window.phantomFilmer.startMission).toHaveBeenCalledWith({ mission: 'follow', profileName: '乙', initialControlMode: 'manual' }))
    expect(screen.getByRole('combobox', { name: '人物档案' })).toHaveValue('乙')
  })

  it('shows the preflight checklist with per-item status once connected', async () => {
    const preflight = { sdk: true, video: false, battery: true, bottomTof: true, frontTof: false }
    mount({ connect: vi.fn().mockResolvedValue({ ...groundStatus, preflight, videoReady: false }) })
    await connectDrone()
    switchTab('起飞准备')
    const checklist = await screen.findByLabelText('起飞预检清单')
    expect(within(checklist).getAllByRole('listitem')).toHaveLength(5)
    expect(within(checklist).getByText('视频流').closest('li')).toHaveClass('fail')
    expect(within(checklist).getByText('前向 ToF').closest('li')).toHaveClass('fail')
    expect(within(checklist).getByText('SDK 通信').closest('li')).toHaveClass('pass')
  })

  it('enrolls in two steps: pick photos, confirm, then write the profile', async () => {
    mount()
    await screen.findByRole('option', { name: '甲' })
    fireEvent.change(screen.getByRole('textbox', { name: '新人物档案名' }), { target: { value: '丙' } })
    fireEvent.click(screen.getByRole('button', { name: '新建档案' }))

    expect(await window.phantomFilmer.pickProfilePhotos).toHaveBeenCalled()
    expect(await screen.findByText(/已为「丙」选择 2 张参考照片/)).toBeInTheDocument()
    expect(window.phantomFilmer.enrollProfile).not.toHaveBeenCalled()

    fireEvent.click(screen.getByRole('button', { name: '确认建档' }))
    await waitFor(() => expect(window.phantomFilmer.enrollProfile).toHaveBeenCalledWith('丙', ['/tmp/a.jpg', '/tmp/b.jpg'], false))
  })

  it('asks before overwriting an existing profile and forwards the overwrite choice', async () => {
    mount()
    await screen.findByRole('option', { name: '甲' })
    fireEvent.change(screen.getByRole('textbox', { name: '新人物档案名' }), { target: { value: '甲' } })
    fireEvent.click(screen.getByRole('button', { name: '新建档案' }))
    await screen.findByText(/已为「甲」选择 2 张参考照片/)

    const confirm = vi.spyOn(window, 'confirm').mockReturnValue(true)
    fireEvent.click(screen.getByRole('button', { name: '确认建档' }))

    expect(confirm).toHaveBeenCalledWith('人物档案“甲”已存在。\n\n是否使用新选择的照片覆盖原档案？')
    await waitFor(() => expect(window.phantomFilmer.enrollProfile).toHaveBeenCalledWith('甲', ['/tmp/a.jpg', '/tmp/b.jpg'], true))
  })

  it('does not write the profile when overwrite is cancelled at the confirm step', async () => {
    mount()
    await screen.findByRole('option', { name: '甲' })
    fireEvent.change(screen.getByRole('textbox', { name: '新人物档案名' }), { target: { value: '甲' } })
    fireEvent.click(screen.getByRole('button', { name: '新建档案' }))
    await screen.findByText(/已为「甲」选择 2 张参考照片/)

    vi.spyOn(window, 'confirm').mockReturnValue(false)
    fireEvent.click(screen.getByRole('button', { name: '确认建档' }))

    expect(window.phantomFilmer.enrollProfile).not.toHaveBeenCalled()
  })

  it('updates photos and renames the selected profile while grounded', async () => {
    mount()
    await screen.findByRole('option', { name: '甲' })
    vi.spyOn(window, 'confirm').mockReturnValue(true)
    fireEvent.click(screen.getByRole('button', { name: '更新照片' }))
    await waitFor(() => expect(window.phantomFilmer.enrollProfile).toHaveBeenCalledWith('甲', ['/tmp/a.jpg', '/tmp/b.jpg'], true))

    fireEvent.click(screen.getByRole('button', { name: '重命名' }))
    fireEvent.change(screen.getByRole('textbox', { name: '新档案名' }), { target: { value: '甲-新版' } })
    fireEvent.click(screen.getByRole('button', { name: '确认重命名' }))
    await waitFor(() => expect(window.phantomFilmer.renameProfile).toHaveBeenCalledWith('甲', '甲-新版'))
  })

  it('requires confirmation and deletes the selected profile recoverably', async () => {
    const listProfiles = vi.fn()
      .mockResolvedValueOnce([{ name: '甲', photoCount: 3 }, { name: '乙', photoCount: 4 }])
      .mockResolvedValueOnce([{ name: '乙', photoCount: 4 }])
    mount({ listProfiles })
    await screen.findByRole('option', { name: '甲' })
    vi.spyOn(window, 'confirm').mockReturnValue(true)
    fireEvent.click(screen.getByRole('button', { name: '删除档案' }))
    await waitFor(() => expect(window.phantomFilmer.deleteProfile).toHaveBeenCalledWith('甲'))
    await waitFor(() => expect(screen.getByRole('combobox', { name: '人物档案' })).toHaveValue('乙'))
  })

  it('does not show or require ground preview before launch', async () => {
    mount()
    expect(screen.queryByRole('region', { name: '地面人物识别预览' })).not.toBeInTheDocument()
    expect(window.phantomFilmer.startPreview).not.toHaveBeenCalled()
  })

  it('keeps mode buttons directly above the video and sends manual takeover', async () => {
    await enterMission()
    const controls = screen.getByLabelText('自动任务空中控制')
    fireEvent.click(within(controls).getByRole('button', { name: '手动接管' }))
    await waitFor(() => expect(window.phantomFilmer.inputKey).toHaveBeenCalledWith('m'))
  })

  it('allows the default normal mode to be selected at control ready', async () => {
    await enterMission({ flightState: 'CONTROL_READY' }, { flightState: 'CONTROL_READY' })
    const normal = await screen.findByRole('button', { name: '普通跟随' })
    expect(normal).toBeEnabled()
    fireEvent.click(normal)
    await waitFor(() => expect(window.phantomFilmer.inputKey).toHaveBeenCalledWith('1'))
  })

  it('shows the control-ready guidance banner over the video while awaiting mode selection', async () => {
    await enterMission({ flightState: 'CONTROL_READY' }, { flightState: 'CONTROL_READY', videoReady: true })
    expect(await screen.findByText('已到达悬停高度，请选择跟随模式（普通 / 侧向 / 前向 / 手动）')).toBeInTheDocument()
  })

  it('renders the Chinese flight state and pause control in the status strip', async () => {
    await enterMission({ flightState: 'LOW_BATTERY_LANDING' }, { flightState: 'LOW_BATTERY_LANDING' })
    expect(await screen.findByText('低电量，降落中')).toBeInTheDocument()
    const controls = screen.getByLabelText('自动任务空中控制')
    fireEvent.click(within(controls).getByRole('button', { name: '暂停任务' }))
    await waitFor(() => expect(window.phantomFilmer.toggleMissionPause).toHaveBeenCalledTimes(1))
  })

  it('toggles the mission pause from the keyboard p key', async () => {
    await enterMission()
    await screen.findByText('自动跟随中')
    fireEvent.keyDown(window, { key: 'p' })
    await waitFor(() => expect(window.phantomFilmer.toggleMissionPause).toHaveBeenCalledTimes(1))
  })

  it('requires a second keyboard q press within the arm window before stopping the mission', async () => {
    await enterMission()
    fireEvent.keyDown(window, { key: 'q' })
    expect(window.phantomFilmer.stopMission).not.toHaveBeenCalled()
    fireEvent.keyDown(window, { key: 'q' })
    await waitFor(() => expect(window.phantomFilmer.stopMission).toHaveBeenCalledTimes(1))
  })

  it('keeps keyboard RC available only after manual takeover is active', async () => {
    const airborne = { ...groundStatus, airborne: true, rcEnabled: true, canTakeoff: false, phase: '手动飞行' as const }
    mount({ connect: vi.fn().mockResolvedValue(airborne), status: vi.fn().mockResolvedValue(airborne) })
    await connectDrone()
    await waitFor(() => expect(screen.getByRole('button', { name: /前进/ })).toBeEnabled())
    fireEvent.keyDown(window, { code: 'KeyW' })
    await waitFor(() => expect(window.phantomFilmer.moveRc).toHaveBeenCalledWith({ leftRight: 0, forwardBack: 20, upDown: 0, yaw: 0 }))
    fireEvent.keyUp(window, { code: 'KeyW' })
    await waitFor(() => expect(window.phantomFilmer.hover).toHaveBeenCalled())
  })

  it('ignores keyboard RC while input fields have focus', async () => {
    const airborne = { ...groundStatus, airborne: true, rcEnabled: true, canTakeoff: false, phase: '手动飞行' as const }
    mount({ connect: vi.fn().mockResolvedValue(airborne), status: vi.fn().mockResolvedValue(airborne) })
    await connectDrone()
    await waitFor(() => expect(screen.getByRole('button', { name: /前进/ })).toBeEnabled())
    const input = screen.getByRole('textbox', { name: '新人物档案名' })
    input.focus()
    fireEvent.keyDown(input, { code: 'KeyW' })
    expect(window.phantomFilmer.moveRc).not.toHaveBeenCalled()
  })

  it('disables keyboard RC via the manual keyboard toggle without touching the semantic keys', async () => {
    const airborne = { ...groundStatus, airborne: true, rcEnabled: true, canTakeoff: false, phase: '手动飞行' as const }
    mount({ connect: vi.fn().mockResolvedValue(airborne), status: vi.fn().mockResolvedValue(airborne) })
    await connectDrone()
    await waitFor(() => expect(screen.getByRole('button', { name: /前进/ })).toBeEnabled())
    fireEvent.click(screen.getByRole('checkbox', { name: '键盘控制' }))
    fireEvent.keyDown(window, { code: 'KeyW' })
    expect(window.phantomFilmer.moveRc).not.toHaveBeenCalled()
  })

  it('offers a manual retry for the video session after the stream breaks', async () => {
    const airborne = { ...groundStatus, airborne: true, videoReady: true, rcEnabled: false, canTakeoff: false, phase: '手动飞行' as const }
    mount({ connect: vi.fn().mockResolvedValue(airborne), status: vi.fn().mockResolvedValue(airborne) })
    await connectDrone()
    const image = await screen.findByRole('img', { name: '无人机实时视频流' })
    fireEvent.error(image)
    const retry = await screen.findByRole('button', { name: '重建视频会话' })
    fireEvent.click(retry)
    await waitFor(() => expect(vi.mocked(window.phantomFilmer.getVideoUrl).mock.calls.length).toBeGreaterThanOrEqual(2))
  })

  it('disconnects the drone from the top bar when grounded and idle', async () => {
    mount()
    await connectDrone()
    fireEvent.click(screen.getByRole('button', { name: '断开真机' }))
    await waitFor(() => expect(window.phantomFilmer.stop).toHaveBeenCalledTimes(1))
    expect(await screen.findByText('已断开真机连接。')).toBeInTheDocument()
  })

  it('shows recovery controls when the backend is offline', async () => {
    mount({ getBackendState: vi.fn().mockResolvedValue({ ...readyBackend, status: 'offline', error: 'sidecar missing' }) })
    fireEvent.click(await screen.findByRole('button', { name: '重启后端' }))
    await waitFor(() => expect(window.phantomFilmer.restartBackend).toHaveBeenCalledTimes(1))
    expect(await screen.findByText('本地后端已恢复，请重新连接真机。')).toBeInTheDocument()
  })

  it('renders the manual takeover controls as an overlay inside the video panel', async () => {
    const airborne = { ...groundStatus, airborne: true, rcEnabled: true, canTakeoff: false, phase: '手动飞行' as const }
    mount({ connect: vi.fn().mockResolvedValue(airborne), status: vi.fn().mockResolvedValue(airborne) })
    await connectDrone()
    const hud = await screen.findByLabelText('手动控制')
    expect(hud).toBeVisible()
    expect(hud.closest('.video-panel')).not.toBeNull()
  })

  it('renders runtime events in the events tab once they arrive', async () => {
    mount({
      getRuntimeEvents: vi.fn().mockResolvedValue({
        apiVersion: '1', latestSequence: 2, resetRequired: false,
        events: [
          { sequence: 1, occurredAt: 1755800000000, type: 'profile.enrolled', payload: {} },
          { sequence: 2, occurredAt: 1755800001000, type: 'flight.safety_landing.started', payload: {} }
        ]
      })
    })
    switchTab('运行事件')
    expect(await screen.findByText('安全降落已触发')).toBeInTheDocument()
    expect(screen.getByText('人物档案已建档')).toBeInTheDocument()
  })
})
