import { useCallback, useEffect, useMemo, useRef, useState, type ReactElement } from 'react'
import type { BackendState, DroneStatus, RcCommand } from '../../preload/api'
import { Icon } from './Icons'
import { useRuntimeFeed } from './app/useRuntimeFeed'

type ConnectionState = 'disconnected' | 'connecting' | 'connected' | 'error'
type ArmedAction = 'start' | 'stop' | 'emergency' | null
type PartialRcCommand = Partial<RcCommand>
type MissionMode = 'manual' | 'normal' | 'side' | 'front'
type Control = [string, string, string, PartialRcCommand, 'up' | 'left' | 'center' | 'right' | 'down']

const emptyCommand: RcCommand = { leftRight: 0, forwardBack: 0, upDown: 0, yaw: 0 }
const missionKey: Record<MissionMode, string> = { manual: 'm', normal: '1', side: '2', front: '3' }
const initialBackend: BackendState = { status: 'starting', version: '—', logDir: '—', airborne: false, restartAllowed: false }

function errorMessage(error: unknown, fallback: string): string {
  return error instanceof Error ? error.message : fallback
}

function missionModeLabel(mode: string): string {
  return ({ manual: '手动接管', normal: '普通跟随', side: '侧向跟随', front: '前向跟随' } as Record<string, string>)[mode] ?? mode
}

export default function App(): ReactElement {
  const [backend, setBackend] = useState<BackendState>(initialBackend)
  const [connection, setConnection] = useState<ConnectionState>('disconnected')
  const [status, setStatus] = useState<DroneStatus>({})
  const [notice, setNotice] = useState('桌面后端正在启动…')
  const [armedAction, setArmedAction] = useState<ArmedAction>(null)
  const [actionBusy, setActionBusy] = useState<string | null>(null)
  const [speed, setSpeed] = useState(20)
  const [activeControl, setActiveControl] = useState<string | null>(null)
  const [videoUrl, setVideoUrl] = useState<string | null>(null)
  const [profiles, setProfiles] = useState<Array<{ name: string; photoCount?: number | null }>>([])
  const [profileName, setProfileName] = useState('')
  const [enrollmentName, setEnrollmentName] = useState('')
  const controlTimer = useRef<number | null>(null)
  const rcInFlight = useRef(false)

  const backendReady = backend.status === 'ready'
  const runtime = useRuntimeFeed(backendReady)
  const connected = backendReady && connection === 'connected'
  const videoReady = connected && status.videoReady === true
  const airborne = connected && status.airborne === true
  const activeMission = runtime.snapshot?.mission
  const missionRunning = activeMission != null && !['idle', 'manual'].includes(activeMission)
  const missionPaused = status.paused ?? runtime.snapshot?.telemetry.paused === true
  const awaitingModeSelection = status.flightState === 'CONTROL_READY' || runtime.snapshot?.flightState === 'CONTROL_READY'
  const manualControlEnabled = airborne && status.rcEnabled === true
  const controlsLocked = !backendReady || actionBusy != null
  const missionReady = runtime.capabilities?.missionReadiness?.available === true
  const canStartMission = new Set(runtime.capabilities?.missions ?? []).has('follow')
    && missionReady
    && profileName.trim().length > 0
    && !missionRunning
    && new Set(runtime.snapshot?.allowedActions ?? []).has('start_mission')

  const refreshProfiles = useCallback(async (): Promise<void> => {
    try {
      const items = await window.phantomFilmer.listProfiles()
      setProfiles(items)
      setProfileName((current) => current && items.some((item) => item.name === current) ? current : items[0]?.name ?? '')
    } catch (error) {
      setNotice(errorMessage(error, '人物档案读取失败'))
    }
  }, [])

  useEffect(() => {
    let mounted = true
    void window.phantomFilmer.getBackendState().then((state) => {
      if (!mounted) return
      setBackend(state)
      setNotice(state.status === 'ready' ? '本地后端已就绪，请连接真机' : state.error ?? '桌面后端正在启动…')
      if (state.status === 'ready') void refreshProfiles()
    })
    const unsubscribe = window.phantomFilmer.onBackendState((state) => {
      setBackend(state)
      if (state.status === 'ready') void refreshProfiles()
      if (state.status === 'offline') {
        setConnection('error')
        setActiveControl(null)
        setVideoUrl(null)
        setNotice(state.error ?? '本地后端已停止。')
      }
    })
    return () => { mounted = false; unsubscribe() }
  }, [refreshProfiles])

  useEffect(() => {
    if (!armedAction) return
    const timer = window.setTimeout(() => setArmedAction(null), 4000)
    return () => window.clearTimeout(timer)
  }, [armedAction])

  useEffect(() => {
    if (!connected) return
    let cancelled = false
    const refresh = async (): Promise<void> => {
      try {
        const next = await window.phantomFilmer.status()
        if (!cancelled) {
          setStatus(next)
          if (next.safetyReason) setNotice(next.safetyReason)
        }
      } catch (error) {
        if (!cancelled) setNotice(errorMessage(error, '真机遥测暂时不可用'))
      }
    }
    const timer = window.setInterval(() => void refresh(), 1000)
    return () => { cancelled = true; window.clearInterval(timer) }
  }, [connected])

  useEffect(() => {
    if (!videoReady) { setVideoUrl(null); return }
    let cancelled = false
    void window.phantomFilmer.getVideoUrl().then((url) => { if (!cancelled) setVideoUrl(url) }).catch((error) => {
      if (!cancelled) setNotice(errorMessage(error, '无法创建安全视频会话'))
    })
    return () => { cancelled = true }
  }, [videoReady])

  const connectDrone = async (): Promise<void> => {
    if (!backendReady) return
    setConnection('connecting')
    setActionBusy('connect')
    setNotice('正在连接真机并检查视频…')
    try {
      const next = await window.phantomFilmer.connect()
      setStatus(next)
      setConnection('connected')
      setNotice(next.videoReady ? '真机与视频流均已就绪' : '真机已连接，正在等待有效视频帧')
    } catch (error) {
      setConnection('error')
      setStatus({})
      setNotice(errorMessage(error, '未连接到真机'))
    } finally { setActionBusy(null) }
  }

  const enrollProfile = async (): Promise<void> => {
    const name = enrollmentName.trim()
    const overwrite = profiles.some((profile) => profile.name === name)
    if (overwrite && !window.confirm(`人物档案“${name}”已存在。\n\n是否选择新的参考照片并覆盖原档案？`)) return
    setActionBusy('enroll')
    try {
      const profile = await window.phantomFilmer.enrollProfile(name, overwrite)
      if (profile) {
        setProfiles((current) => [...current.filter((item) => item.name !== profile.name), profile])
        setProfileName(profile.name)
        setEnrollmentName('')
        setNotice(`人物档案“${profile.name}”已创建并选中。`)
      }
    } catch (error) { setNotice(errorMessage(error, '人物建档失败')) } finally { setActionBusy(null) }
  }

  const startMission = async (): Promise<void> => {
    if (armedAction !== 'start') {
      setArmedAction('start')
      setNotice('请在 4 秒内再次点击“确认起飞”。')
      return
    }
    setActionBusy('start')
    setArmedAction(null)
    try {
      await window.phantomFilmer.startMission({ mission: 'follow', profileName: profileName.trim(), initialControlMode: 'manual', obstacleEnabled: false })
      setNotice('正在起飞并上升至 150 cm；到达后请选择跟随模式。')
    } catch (error) { setNotice(errorMessage(error, '自动任务启动失败')) } finally { setActionBusy(null) }
  }

  const runMissionCommand = async (action: 'stop' | 'emergency' | MissionMode): Promise<void> => {
    if ((action === 'stop' || action === 'emergency') && armedAction !== action) {
      setArmedAction(action)
      setNotice(action === 'stop' ? '请再次点击确认停止并降落。' : '请再次点击确认急停。')
      return
    }
    setActionBusy(`mission-${action}`)
    setArmedAction(null)
    try {
      if (action === 'stop') {
        await window.phantomFilmer.stopMission()
        setNotice('任务已停止并降落。')
      } else if (action === 'emergency') {
        if (missionRunning) await window.phantomFilmer.emergencyStopMission()
        else await window.phantomFilmer.emergencyLand()
        setNotice('紧急降落指令已发送。')
      } else {
        await window.phantomFilmer.inputKey(missionKey[action])
        setNotice(action === 'manual'
          ? runtime.snapshot?.controlMode === 'manual' ? '正在退出手动，重新识别并恢复自动跟随…' : '正在切换到手动接管…'
          : `正在切换到${missionModeLabel(action)}…`)
      }
      setStatus(await window.phantomFilmer.status())
    } catch (error) { setNotice(errorMessage(error, '任务操作失败，请检查真机状态')) } finally { setActionBusy(null) }
  }

  const sendRc = useCallback(async (partial: PartialRcCommand): Promise<void> => {
    if (rcInFlight.current) return
    rcInFlight.current = true
    try { await window.phantomFilmer.moveRc({ ...emptyCommand, ...partial }) } catch (error) { setNotice(errorMessage(error, '手动控制被安全系统拒绝')) } finally { rcInFlight.current = false }
  }, [])

  const stopControl = useCallback(async (): Promise<void> => {
    if (controlTimer.current != null) { window.clearInterval(controlTimer.current); controlTimer.current = null }
    setActiveControl(null)
    if (!manualControlEnabled) return
    try { setStatus(await window.phantomFilmer.hover()) } catch (error) { setNotice(errorMessage(error, '悬停指令失败')) }
  }, [manualControlEnabled])

  const startControl = useCallback((name: string, command: PartialRcCommand): void => {
    if (!manualControlEnabled || actionBusy) return
    if (controlTimer.current != null) window.clearInterval(controlTimer.current)
    setActiveControl(name)
    void sendRc(command)
    controlTimer.current = window.setInterval(() => void sendRc(command), 180)
  }, [actionBusy, manualControlEnabled, sendRc])

  useEffect(() => {
    const commands: Record<string, [string, PartialRcCommand]> = {
      KeyW: ['forward', { forwardBack: speed }], KeyS: ['back', { forwardBack: -speed }], KeyA: ['left', { leftRight: -speed }], KeyD: ['right', { leftRight: speed }],
      KeyR: ['up', { upDown: speed }], KeyF: ['down', { upDown: -speed }], KeyJ: ['yaw-left', { yaw: -speed }], KeyL: ['yaw-right', { yaw: speed }]
    }
    const keyDown = (event: KeyboardEvent): void => {
      const key = event.key.toLowerCase()
      const missionKeyPressed = key === 'm' || (!manualControlEnabled && ['a', 's', 'f', '1', '2', '3', 'q', 'e'].includes(key))
      if (missionRunning && !event.repeat && missionKeyPressed) {
        event.preventDefault()
        void window.phantomFilmer.inputKey(key).then(async () => setStatus(await window.phantomFilmer.status())).catch((error) => setNotice(errorMessage(error, '飞行模式按键被后端拒绝')))
        return
      }
      if (event.code === 'Space') { event.preventDefault(); if (!event.repeat) void stopControl(); return }
      const entry = commands[event.code]
      if (!entry || event.repeat) return
      event.preventDefault()
      startControl(entry[0], entry[1])
    }
    const keyUp = (event: KeyboardEvent): void => { if (commands[event.code]) { event.preventDefault(); void stopControl() } }
    window.addEventListener('keydown', keyDown)
    window.addEventListener('keyup', keyUp)
    window.addEventListener('blur', stopControl)
    return () => { window.removeEventListener('keydown', keyDown); window.removeEventListener('keyup', keyUp); window.removeEventListener('blur', stopControl) }
  }, [manualControlEnabled, missionRunning, speed, startControl, stopControl])

  const retryBackend = async (): Promise<void> => {
    setActionBusy('restart')
    try {
      const next = await window.phantomFilmer.restartBackend()
      setBackend(next)
      setConnection('disconnected')
      setStatus({})
      await refreshProfiles()
      setNotice('本地后端已恢复，请重新连接真机。')
    } catch (error) { setNotice(errorMessage(error, '后端重启失败')) } finally { setActionBusy(null) }
  }

  const batteryLabel = useMemo(() => status.battery == null || !connected ? '—' : `${status.battery}%`, [connected, status.battery])
  const heightLabel = connected ? `${status.heightCm ?? '—'} cm` : '—'
  const tofLabel = !connected ? '—' : typeof status.frontTofCm === 'number' ? `${status.frontTofCm} cm` : status.frontTofState === 'out_of_range' ? '远' : '—'
  const activeProfile = profileName || '未选择'

  return <main className="app-shell core-shell">
    <header className="topbar core-topbar"><div className="brand" aria-label="PhantomFilmer 桌面飞控台"><span className="brand-mark" aria-hidden="true"><i /><i /><i /></span><div><strong>PhantomFilmer</strong><small>FOLLOWING CONSOLE</small></div></div><div className="system-summary"><StatusPill good={backendReady} label={backendReady ? '后端在线' : '后端离线'} /><StatusPill good={connected} label={connected ? '真机已连接' : '真机未连接'} /></div></header>
    {backend.status === 'offline' && <section className={`diagnostic ${backend.airborne ? 'critical' : ''}`} role="alert"><div className="diagnostic-icon"><Icon name={backend.airborne ? 'emergency' : 'activity'} /></div><div><h1>{backend.airborne ? '后端中断 · 最后状态为空中' : '本地后端未运行'}</h1><p>{backend.error ?? '请检查后端后重试。'}</p></div><button className="button secondary" disabled={!backend.restartAllowed || actionBusy != null} onClick={() => void retryBackend()}>{backend.airborne ? '禁止自动重启' : '重启后端'}</button></section>}
    <section className="core-layout" aria-label="自动跟随控制台">
      <aside className="profile-panel"><h1>任务与人物</h1><label className="profile-picker"><span>当前档案</span><select aria-label="人物档案" value={profileName} onChange={(event) => setProfileName(event.target.value)} disabled={missionRunning || actionBusy != null}><option value="">请选择人物档案</option>{profiles.map((profile) => <option key={profile.name} value={profile.name}>{profile.name}</option>)}</select></label><div className="profile-create"><input aria-label="新人物档案名" value={enrollmentName} onChange={(event) => setEnrollmentName(event.target.value)} placeholder="新档案名" disabled={connected || actionBusy != null} /><button disabled={!enrollmentName.trim() || connected || actionBusy != null} onClick={() => void enrollProfile()}>新建档案</button></div><div className="profile-summary"><span>当前档案</span><strong>{activeProfile}</strong><small>{profiles.find((profile) => profile.name === profileName)?.photoCount ?? 0} 张参考照片</small></div><button className={`launch-button ${armedAction === 'start' ? 'armed' : ''}`} disabled={!canStartMission || controlsLocked} onClick={() => void startMission()}>{armedAction === 'start' ? '确认起飞' : '起飞'}</button><div className={`readiness ${canStartMission ? 'ready' : ''}`}><i>{canStartMission ? '✓' : '—'}</i><span>{canStartMission ? '已满足起飞条件' : connected ? '请完成起飞检查并选择档案' : '请先连接无人机'}</span></div>{runtime.capabilities?.missionReadiness && !missionReady && <p className="runtime-error">缺少运行资产：{runtime.capabilities.missionReadiness.missingAssets.join('、')}</p>}</aside>
      <section className="flight-panel"><header className="flight-header"><div><h1>飞行控制</h1><span>当前档案：<strong>{activeProfile}</strong></span></div></header><div className="mode-row" aria-label="自动任务空中控制">{(['normal', 'side', 'front', 'manual'] as const).map((mode) => <button key={mode} className={runtime.snapshot?.controlMode === mode ? 'active' : ''} disabled={!missionRunning || controlsLocked || missionPaused || (!awaitingModeSelection && mode !== 'manual' && runtime.snapshot?.controlMode === mode)} onClick={() => void runMissionCommand(mode)}>{mode === 'manual' && runtime.snapshot?.controlMode === 'manual' ? '退出手动并恢复自动' : missionModeLabel(mode)}</button>)}</div><section className={`video-panel core-video ${videoUrl ? 'streaming' : ''}`}>{videoUrl ? <img src={videoUrl} alt="无人机实时视频流" onError={() => { setVideoUrl(null); setNotice('视频会话中断，请检查真机连接') }} /> : <div className="video-empty"><div className="reticle" aria-hidden="true"><span /><span /><span /><span /><i /></div><h2>{connection === 'connecting' ? '正在建立链路' : backendReady ? '等待真机视频' : '后端离线'}</h2><p>连接无人机后即可显示实时视频与人物识别框。</p><button className="button primary" disabled={!backendReady || connection === 'connecting' || connected} onClick={() => void connectDrone()}><Icon name="link" />{connection === 'connecting' ? '连接中…' : connection === 'error' ? '重新连接真机' : '连接真机'}</button></div>}<div className="notice" role="status"><b>{actionBusy ? '处理中' : '状态'}</b><span>{notice}</span></div></section><footer className="flight-footer"><TelemetryMetric label="电量" value={batteryLabel} /><TelemetryMetric label="高度" value={heightLabel} /><TelemetryMetric label="前向 ToF" value={tofLabel} /><button className={`stop-button ${armedAction === 'stop' ? 'armed' : ''}`} disabled={!missionRunning || controlsLocked} onClick={() => void runMissionCommand('stop')}>{armedAction === 'stop' ? '确认停止并降落' : '停止并降落'}</button><button className={`emergency-button ${armedAction === 'emergency' ? 'armed' : ''}`} disabled={!airborne || controlsLocked} onClick={() => void runMissionCommand('emergency')}>{armedAction === 'emergency' ? '再次确认急停' : '急停'}</button></footer>{manualControlEnabled && <section className="manual-controls compact"><div className="manual-heading"><div><p className="eyebrow">MANUAL TAKEOVER</p><h2>手动控制</h2></div><div className="speed-select" aria-label="速度档位">{[15, 20, 30].map((value) => <button key={value} className={speed === value ? 'active' : ''} disabled={controlsLocked} onClick={() => setSpeed(value)}>{value}</button>)}</div></div><div className="pads"><ControlPad title="平面移动" active={activeControl} enabled={!controlsLocked} onStart={startControl} onStop={stopControl} controls={[['forward', 'W', '前进', { forwardBack: speed }, 'up'], ['left', 'A', '左移', { leftRight: -speed }, 'left'], ['hover-a', 'SPACE', '悬停', {}, 'center'], ['right', 'D', '右移', { leftRight: speed }, 'right'], ['back', 'S', '后退', { forwardBack: -speed }, 'down']]} /><ControlPad title="高度与偏航" active={activeControl} enabled={!controlsLocked} onStart={startControl} onStop={stopControl} controls={[['up', 'R', '上升', { upDown: speed }, 'up'], ['yaw-left', 'J', '左转', { yaw: -speed }, 'left'], ['hover-b', 'SPACE', '悬停', {}, 'center'], ['yaw-right', 'L', '右转', { yaw: speed }, 'right'], ['down', 'F', '下降', { upDown: -speed }, 'down']]} /></div></section>}</section>
    </section>
  </main>
}

function ControlPad({ title, controls, active, enabled, onStart, onStop }: { title: string; controls: Control[]; active: string | null; enabled: boolean; onStart: (name: string, command: PartialRcCommand) => void; onStop: () => Promise<void> }): ReactElement {
  return <div className="control-pad"><span className="control-title">{title}</span><div className="pad-grid">{controls.map(([name, key, label, command, position]) => <button key={name} className={`control-key ${position} ${active === name ? 'active' : ''}`} disabled={!enabled} onPointerDown={(event) => { event.preventDefault(); if (name.startsWith('hover')) void onStop(); else onStart(name, command) }} onPointerUp={() => void onStop()} onPointerCancel={() => void onStop()}><kbd>{key}</kbd><span>{label}</span></button>)}</div></div>
}

function StatusPill({ good, label }: { good: boolean; label: string }): ReactElement {
  return <div className="status-pill"><i className={good ? 'good' : ''} /><span>{label}</span></div>
}

function TelemetryMetric({ label, value }: { label: string; value: string }): ReactElement {
  return <div className="telemetry-metric"><span>{label}</span><strong>{value}</strong></div>
}
