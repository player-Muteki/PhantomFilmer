import { useCallback, useEffect, useMemo, useRef, useState, type ReactElement } from 'react'
import type { BackendState, DroneStatus, FlightPhase, GroundPreviewStatus, RcCommand } from '../../preload/api'
import { Icon } from './Icons'
import { useRuntimeFeed, type RuntimeFeed } from './app/useRuntimeFeed'

type ConnectionState = 'disconnected' | 'connecting' | 'connected' | 'error'
type ArmedAction = 'takeoff' | 'land' | 'emergency' | null
type PartialRcCommand = Partial<RcCommand>
type WorkspacePage = 'flight' | 'missions' | 'diagnostics'
type MissionMode = 'manual' | 'normal' | 'side' | 'front'

const phases: FlightPhase[] = ['连接', '检查', '起飞', '手动飞行', '降落']
const emptyCommand: RcCommand = { leftRight: 0, forwardBack: 0, upDown: 0, yaw: 0 }
const initialBackend: BackendState = {
  status: 'starting',
  version: '—',
  logDir: '—',
  airborne: false,
  restartAllowed: false
}

function errorMessage(error: unknown, fallback: string): string {
  return error instanceof Error ? error.message : fallback
}

function missionModeLabel(mode: string): string {
  return ({ manual: '手动接管', normal: '普通跟随', side: '侧向跟随', front: '前向跟随', none: '等待选择' } as Record<string, string>)[mode] ?? mode
}

function missionKindLabel(mission?: string): string {
  return ({ follow: '普通自动跟随', reid_follow: '人物跟拍任务', fixed_demo: '固定航线演示' } as Record<string, string>)[mission ?? ''] ?? mission ?? '自动任务'
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
  const [activePage, setActivePage] = useState<WorkspacePage>('flight')
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
  const manualControlEnabled = airborne && (!missionRunning || status.rcEnabled === true)
  const controlsLocked = !backendReady || actionBusy != null
  const currentPhase = status.phase ?? '连接'
  const currentPhaseIndex = connected ? phases.indexOf(currentPhase) : -1

  useEffect(() => {
    let mounted = true
    void window.phantomFilmer.getBackendState().then((state) => {
      if (!mounted) return
      setBackend(state)
      setNotice(state.status === 'ready' ? '本地后端已就绪，请连接真机' : state.error ?? '本地后端正在启动…')
    })
    const unsubscribe = window.phantomFilmer.onBackendState((state) => {
      setBackend(state)
      if (state.status === 'offline') {
        setConnection('error')
        setActiveControl(null)
        setVideoUrl(null)
        setNotice(state.error ?? '本地后端已停止。')
      }
    })
    return () => {
      mounted = false
      unsubscribe()
    }
  }, [])

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
    return () => {
      cancelled = true
      window.clearInterval(timer)
    }
  }, [connected])

  useEffect(() => {
    if (!videoReady) {
      setVideoUrl(null)
      return
    }
    setVideoUrl(null)
    let cancelled = false
    void window.phantomFilmer.getVideoUrl().then((url) => {
      if (!cancelled) setVideoUrl(url)
    }).catch((error) => {
      if (!cancelled) setNotice(errorMessage(error, '无法创建安全视频会话'))
    })
    return () => {
      cancelled = true
    }
  }, [activePage, videoReady])

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
    } finally {
      setActionBusy(null)
    }
  }

  const runStatusAction = async (action: 'takeoff' | 'land' | 'hover'): Promise<void> => {
    setActionBusy(action)
    try {
      const next = await window.phantomFilmer[action]()
      setStatus(next)
      setNotice(action === 'takeoff' ? '起飞完成，手动控制已启用' : action === 'land' ? '降落完成，视频保持连接' : '已悬停')
    } catch (error) {
      setNotice(errorMessage(error, '飞行指令执行失败'))
    } finally {
      setActionBusy(null)
      setArmedAction(null)
    }
  }

  const disconnect = async (emergency = false): Promise<void> => {
    setActionBusy(emergency ? 'emergency' : 'stop')
    setArmedAction(null)
    setNotice(emergency ? '紧急降落指令已发送' : '正在安全停止；如已起飞将先降落…')
    try {
      if (emergency) await window.phantomFilmer.emergencyLand()
      else await window.phantomFilmer.stop()
      setConnection('disconnected')
      setStatus({})
      setVideoUrl(null)
      setNotice(emergency ? '紧急降落完成，连接已关闭' : '任务已停止，真机连接已关闭')
    } catch (error) {
      setNotice(errorMessage(error, '指令发送失败，请立即检查真机'))
    } finally {
      setActionBusy(null)
    }
  }

  const runMissionCommand = async (
    action: 'pause' | 'stop' | 'emergency' | MissionMode
  ): Promise<void> => {
    setActionBusy(`mission-${action}`)
    setArmedAction(null)
    try {
      if (action === 'pause') {
        await window.phantomFilmer.toggleMissionPause()
        setNotice(missionPaused ? '正在恢复自动任务…' : '正在暂停并清零任务输出…')
      } else if (action === 'stop') {
        await window.phantomFilmer.stopMission()
        setNotice('自动任务已停止并完成降落')
      } else if (action === 'emergency') {
        await window.phantomFilmer.emergencyStopMission()
        setNotice('自动任务已急停并完成降落')
      } else {
        await window.phantomFilmer.selectControlMode(action)
        setNotice(action === 'manual' ? '正在切换到手动接管…' : `正在切换到${missionModeLabel(action)}…`)
      }
      const next = await window.phantomFilmer.status()
      setStatus(next)
    } catch (error) {
      setNotice(errorMessage(error, '任务操作失败，请检查真机状态'))
    } finally {
      setActionBusy(null)
    }
  }

  const confirmAction = (action: Exclude<ArmedAction, null>): void => {
    if (armedAction !== action) {
      setArmedAction(action)
      setNotice(action === 'takeoff' ? '请再次点击确认起飞' : action === 'land' ? '请再次点击确认正常降落' : '请再次点击确认紧急降落')
      return
    }
    if (action === 'takeoff') void runStatusAction('takeoff')
    if (action === 'land') void runStatusAction('land')
    if (action === 'emergency') {
      if (missionRunning) void runMissionCommand('emergency')
      else void disconnect(true)
    }
  }

  const sendRc = useCallback(async (partial: PartialRcCommand): Promise<void> => {
    if (rcInFlight.current) return
    rcInFlight.current = true
    try {
      await window.phantomFilmer.moveRc({ ...emptyCommand, ...partial })
    } catch (error) {
      setNotice(errorMessage(error, '手动控制被安全系统拒绝'))
    } finally {
      rcInFlight.current = false
    }
  }, [])

  const stopControl = useCallback(async (): Promise<void> => {
    if (controlTimer.current != null) {
      window.clearInterval(controlTimer.current)
      controlTimer.current = null
    }
    setActiveControl(null)
    if (!manualControlEnabled) return
    try {
      const next = await window.phantomFilmer.hover()
      setStatus(next)
      setNotice('已悬停')
    } catch (error) {
      setNotice(errorMessage(error, '悬停指令失败'))
    }
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
      KeyW: ['forward', { forwardBack: speed }],
      KeyS: ['back', { forwardBack: -speed }],
      KeyA: ['left', { leftRight: -speed }],
      KeyD: ['right', { leftRight: speed }],
      KeyR: ['up', { upDown: speed }],
      KeyF: ['down', { upDown: -speed }],
      KeyJ: ['yaw-left', { yaw: -speed }],
      KeyL: ['yaw-right', { yaw: speed }]
    }
    const keyDown = (event: KeyboardEvent): void => {
      if (event.code === 'Space') {
        event.preventDefault()
        if (!event.repeat) void stopControl()
        return
      }
      const entry = commands[event.code]
      if (!entry || event.repeat) return
      event.preventDefault()
      startControl(entry[0], entry[1])
    }
    const keyUp = (event: KeyboardEvent): void => {
      if (!commands[event.code]) return
      event.preventDefault()
      void stopControl()
    }
    window.addEventListener('keydown', keyDown)
    window.addEventListener('keyup', keyUp)
    window.addEventListener('blur', stopControl)
    return () => {
      window.removeEventListener('keydown', keyDown)
      window.removeEventListener('keyup', keyUp)
      window.removeEventListener('blur', stopControl)
    }
  }, [speed, startControl, stopControl])

  const retryBackend = async (): Promise<void> => {
    setActionBusy('restart')
    setNotice('正在重新启动本地后端…')
    try {
      const next = await window.phantomFilmer.restartBackend()
      setBackend(next)
      setConnection('disconnected')
      setStatus({})
      setNotice('本地后端已恢复，请重新连接真机')
    } catch (error) {
      setNotice(errorMessage(error, '后端重启失败'))
    } finally {
      setActionBusy(null)
    }
  }

  const batteryLabel = useMemo(() => status.battery == null || !connected ? '—' : `${status.battery}%`, [connected, status.battery])
  const preflightItems = [
    ['SDK 连接', status.preflight?.sdk],
    ['有效视频帧', status.preflight?.video],
    [`电量 ≥ ${runtime.capabilities?.safety?.minTakeoffBattery ?? 20}%`, status.preflight?.battery],
    ['底部 ToF', status.preflight?.bottomTof],
    ['前向 ToF', status.preflight?.frontTof]
  ] as const

  return (
    <main className="app-shell">
      <header className="topbar">
        <div className="brand" aria-label="PhantomFilmer 桌面飞控台">
          <span className="brand-mark" aria-hidden="true"><i /><i /><i /></span>
          <div><strong>PhantomFilmer</strong><small>REAL DEVICE CONSOLE</small></div>
        </div>
        <div className="route-line" aria-hidden="true"><span /><i /><i /><i /><i /><i /></div>
        <div className="system-summary">
          <StatusPill good={backendReady} label={backendReady ? '后端在线' : '后端离线'} />
          <StatusPill good={connected} label={connected ? '真机已连接' : '真机未连接'} />
          <div className="battery"><span>BAT</span><strong>{batteryLabel}</strong></div>
        </div>
      </header>

      {backend.status === 'offline' && (
        <section className={`diagnostic ${backend.airborne ? 'critical' : ''}`} role="alert">
          <div className="diagnostic-icon"><Icon name={backend.airborne ? 'emergency' : 'activity'} /></div>
          <div>
            <p className="eyebrow">BACKEND DIAGNOSTICS</p>
            <h1>{backend.airborne ? '后端中断 · 最后状态为空中' : '本地后端未运行'}</h1>
            <p>{backend.error ?? '检查 sidecar 文件与诊断日志后重试。'}</p>
            <dl><div><dt>版本</dt><dd>{backend.version}</dd></div><div><dt>日志目录</dt><dd>{backend.logDir}</dd></div></dl>
          </div>
          <button className="button secondary" disabled={!backend.restartAllowed || actionBusy != null} onClick={() => void retryBackend()}>
            <Icon name="restart" />{backend.airborne ? '禁止自动重启' : '重启后端'}
          </button>
        </section>
      )}

      <nav className="workspace-nav" aria-label="主要工作区">
        <button className={activePage === 'flight' ? 'active' : ''} onClick={() => setActivePage('flight')}>飞行控制</button>
        <button className={activePage === 'missions' ? 'active' : ''} onClick={() => setActivePage('missions')}>任务与人物</button>
        <button className={activePage === 'diagnostics' ? 'active' : ''} onClick={() => setActivePage('diagnostics')}>运行诊断</button>
        <span>API {runtime.capabilities?.apiVersion ?? '—'} · SEQ {runtime.snapshot?.sequence ?? 0}</span>
      </nav>

      {activePage === 'flight' && <>
      {missionRunning && (
        <section className="mission-command-bar" aria-label="自动任务空中控制">
          <div>
            <p className="eyebrow">ACTIVE MISSION</p>
            <strong>{missionKindLabel(activeMission)} · {missionPaused ? '已暂停' : status.flightState ?? '运行中'}</strong>
            <span>当前模式：{missionModeLabel(runtime.snapshot?.controlMode ?? 'none')}</span>
          </div>
          <div className="mission-command-modes">
            {(['normal', 'side', 'front', 'manual'] as const).map((mode) => (
              <button key={mode} className={runtime.snapshot?.controlMode === mode ? 'active' : ''} disabled={controlsLocked || missionPaused || runtime.snapshot?.controlMode === mode} onClick={() => void runMissionCommand(mode)}>{missionModeLabel(mode)}</button>
            ))}
          </div>
          <div className="mission-command-safety">
            <button disabled={controlsLocked} onClick={() => void runMissionCommand('pause')}>{missionPaused ? '继续任务' : '暂停悬停'}</button>
            <button disabled={controlsLocked} onClick={() => void runMissionCommand('stop')}>停止并降落</button>
            <button className={`danger ${armedAction === 'emergency' ? 'armed' : ''}`} disabled={controlsLocked} onClick={() => confirmAction('emergency')}>{armedAction === 'emergency' ? '再次确认急停' : '任务急停'}</button>
          </div>
        </section>
      )}
      <section className="workspace" aria-label="真机控制台">
        <section className={`video-panel ${videoUrl ? 'streaming' : ''}`}>
          {videoUrl ? (
            <img src={videoUrl} alt="无人机实时视频流" onError={() => { setVideoUrl(null); setNotice('视频会话中断，请停止连接后重试') }} />
          ) : (
            <div className="video-empty">
              <div className="reticle" aria-hidden="true"><span /><span /><span /><span /><i /></div>
              <p className="eyebrow">AIRCRAFT OPTICAL FEED</p>
              <h1>{connection === 'connecting' ? '正在建立链路' : backendReady ? '等待真机视频' : '后端离线'}</h1>
              <p>视频流仅在 SDK 连接成功并收到首个有效帧后开启。</p>
              <button className="button primary" disabled={!backendReady || connection === 'connecting' || connected} onClick={() => void connectDrone()}>
                <Icon name="link" />{connection === 'connecting' ? '连接中…' : connection === 'error' ? '重新连接真机' : '连接真机'}
              </button>
            </div>
          )}
          <div className="feed-hud"><span><Icon name="video" />{videoReady ? 'LIVE' : 'STANDBY'}</span><span>{status.flightState ?? '未连接'}</span></div>
          <div className="notice" role="status"><b>{actionBusy ? '处理中' : '系统'}</b><span>{notice}</span></div>
        </section>

        <aside className="telemetry-panel">
          <div className="panel-heading"><div><p className="eyebrow">TELEMETRY</p><h2>飞行遥测</h2></div><span className={`pulse ${connected ? 'active' : ''}`} /></div>
          <div className="flight-state"><span>当前状态</span><strong>{connected ? status.flightState ?? '待机' : '等待连接'}</strong><small>{status.phase ?? '连接'}</small></div>
          <div className="metric-grid">
            <Metric label="高度" value={connected ? status.heightCm ?? '—' : '—'} unit="cm" />
            <Metric label="前向 ToF" value={connected ? status.frontTofCm ?? (status.frontTofState === 'out_of_range' ? '远' : '—') : '—'} unit={typeof status.frontTofCm === 'number' ? 'cm' : ''} />
            <Metric label="视频率" value={connected ? status.controlHz?.toFixed(1) ?? '—' : '—'} unit="Hz" />
          </div>
          <div className="preflight">
            <div className="subheading"><div><p className="eyebrow">PRE-FLIGHT GATES</p><h3>起飞检查</h3></div><span>{status.canTakeoff ? '全部通过' : '未就绪'}</span></div>
            <div className="checklist">{preflightItems.map(([label, ok]) => <div key={label}><i className={ok ? 'ok' : ''}>{ok ? '✓' : '—'}</i><span>{label}</span></div>)}</div>
          </div>
        </aside>
      </section>

      <section className="control-deck">
        <section className="flight-actions">
          <div className="subheading"><div><p className="eyebrow">FLIGHT ACTIONS</p><h2>飞行操作</h2></div><span className={airborne ? 'airborne-badge' : 'ground-badge'}>{airborne ? '空中' : '地面'}</span></div>
          <p className="help">起飞、降落和紧急动作需要在 4 秒内再次确认。</p>
          <div className="action-grid">
            <ActionButton icon="takeoff" label={armedAction === 'takeoff' ? '确认起飞' : '起飞'} hint="五项检查全部通过" armed={armedAction === 'takeoff'} disabled={!status.canTakeoff || controlsLocked} onClick={() => confirmAction('takeoff')} />
            <ActionButton icon="land" label={armedAction === 'land' ? '确认降落' : '正常降落'} hint={missionRunning ? '请使用任务停止' : '保留视频与连接'} armed={armedAction === 'land'} disabled={!airborne || missionRunning || controlsLocked} onClick={() => confirmAction('land')} />
            <ActionButton icon="hover" label="立即悬停" hint={missionRunning && !manualControlEnabled ? '自动模式请先暂停或接管' : '清零四个 RC 通道'} disabled={!manualControlEnabled || controlsLocked} onClick={() => void runStatusAction('hover')} />
          </div>
          <div className="disconnect-row">
            <button className="button secondary" disabled={!connected || missionRunning || controlsLocked} onClick={() => void disconnect(false)}>{missionRunning ? '请先停止任务' : '停止并断开'}</button>
            <button className={`button danger ${armedAction === 'emergency' ? 'armed' : ''}`} disabled={!airborne || controlsLocked} onClick={() => confirmAction('emergency')}><Icon name="emergency" />{armedAction === 'emergency' ? '再次确认紧急降落' : '紧急降落'}</button>
          </div>
        </section>

        <section className={`manual-controls ${manualControlEnabled ? '' : 'disabled-panel'}`}>
          <div className="subheading"><div><p className="eyebrow">MANUAL RC</p><h2>手动控制</h2></div><div className="speed-select" aria-label="速度档位">{[15, 20, 30].map((value) => <button key={value} className={speed === value ? 'active' : ''} disabled={controlsLocked} onClick={() => setSpeed(value)}>{value}</button>)}</div></div>
          <div className="pads">
            <ControlPad title="平面移动" active={activeControl} enabled={manualControlEnabled && !controlsLocked} onStart={startControl} onStop={stopControl} controls={[
              ['forward', 'W', '前进', { forwardBack: speed }, 'up'], ['left', 'A', '左移', { leftRight: -speed }, 'left'], ['hover-a', 'SPACE', '悬停', {}, 'center'], ['right', 'D', '右移', { leftRight: speed }, 'right'], ['back', 'S', '后退', { forwardBack: -speed }, 'down']
            ]} />
            <ControlPad title="高度与偏航" active={activeControl} enabled={manualControlEnabled && !controlsLocked} onStart={startControl} onStop={stopControl} controls={[
              ['up', 'R', '上升', { upDown: speed }, 'up'], ['yaw-left', 'J', '左转', { yaw: -speed }, 'left'], ['hover-b', 'SPACE', '悬停', {}, 'center'], ['yaw-right', 'L', '右转', { yaw: speed }, 'right'], ['down', 'F', '下降', { upDown: -speed }, 'down']
            ]} />
          </div>
          <p className="help centered">{missionRunning && !manualControlEnabled ? '自动任务运行中；切换到手动接管后启用此控制台。' : '按住持续运动，松开立即悬停；Python 后端 0.4 秒看门狗独立兜底。'}</p>
        </section>
      </section>

      <footer className="phase-bar" aria-label="飞行任务阶段">
        {phases.map((phase, index) => {
          const complete = currentPhaseIndex > index
          const active = connected && currentPhase === phase
          return <div className={`phase ${complete ? 'complete' : ''} ${active ? 'active' : ''}`} key={phase}><i>{complete ? '✓' : index + 1}</i><span>{phase}</span></div>
        })}
      </footer>
      </>}

      {activePage === 'missions' && <MissionWorkspace runtime={runtime} videoUrl={videoUrl} onOpenFlight={() => setActivePage('flight')} />}
      {activePage === 'diagnostics' && <DiagnosticsWorkspace runtime={runtime} backend={backend} />}
    </main>
  )
}

function MissionWorkspace({ runtime, videoUrl, onOpenFlight }: { runtime: RuntimeFeed; videoUrl: string | null; onOpenFlight: () => void }): ReactElement {
  const available = new Set(runtime.capabilities?.missions ?? [])
  const readiness = runtime.capabilities?.missionReadiness
  const [profileName, setProfileName] = useState('')
  const [profiles, setProfiles] = useState<Array<{ name: string; photoCount?: number | null }>>([])
  const [enrollmentName, setEnrollmentName] = useState('')
  const [initialMode, setInitialMode] = useState<'manual' | 'normal' | 'side' | 'front'>('normal')
  const [obstacleEnabled, setObstacleEnabled] = useState(false)
  const [armedMission, setArmedMission] = useState<string | null>(null)
  const [armedMissionAction, setArmedMissionAction] = useState<'stop' | 'emergency' | null>(null)
  const [missionBusy, setMissionBusy] = useState<string | null>(null)
  const [missionError, setMissionError] = useState<string | null>(null)
  const [missionNotice, setMissionNotice] = useState<string | null>(null)
  const activeMission = runtime.snapshot?.mission
  const missionRunning = activeMission != null && !['idle', 'manual'].includes(activeMission)
  const missionPaused = runtime.snapshot?.telemetry.paused === true
  const preview = (runtime.snapshot?.telemetry.preview ?? null) as GroundPreviewStatus | null
  const allowedActions = new Set(runtime.snapshot?.allowedActions ?? [])
  useEffect(() => {
    if (!armedMission && !armedMissionAction) return
    const timer = window.setTimeout(() => {
      setArmedMission(null)
      setArmedMissionAction(null)
    }, 4000)
    return () => window.clearTimeout(timer)
  }, [armedMission, armedMissionAction])
  useEffect(() => {
    void window.phantomFilmer.listProfiles().then((items) => {
      setProfiles(items)
      setProfileName((current) => current || items[0]?.name || '')
    }).catch((error) => setMissionError(error instanceof Error ? error.message : '人物档案读取失败。'))
  }, [])
  const enrollProfile = async (): Promise<void> => {
    setMissionError(null)
    setMissionNotice(null)
    setMissionBusy('enroll')
    try {
      const profile = await window.phantomFilmer.enrollProfile(enrollmentName.trim(), false)
      if (!profile) return
      setProfiles((current) => [...current.filter((item) => item.name !== profile.name), profile])
      setProfileName(profile.name)
      setEnrollmentName('')
      setMissionNotice(`人物档案“${profile.name}”已创建。`)
    } catch (error) {
      setMissionError(error instanceof Error ? error.message : '人物建档失败。')
    } finally {
      setMissionBusy(null)
    }
  }
  const startMission = async (mission: 'follow' | 'reid_follow' | 'fixed_demo'): Promise<void> => {
    if (armedMission !== mission) {
      setArmedMission(mission)
      setMissionNotice('请在 4 秒内再次点击，确认任务将从地面起飞。')
      return
    }
    setArmedMission(null)
    setMissionError(null)
    setMissionNotice(null)
    setMissionBusy('start')
    try {
      await window.phantomFilmer.startMission({ mission, profileName: profileName.trim(), initialControlMode: initialMode, obstacleEnabled })
      setMissionNotice(`${missionKindLabel(mission)}已启动，请前往飞行控制查看视频与空中操作。`)
    } catch (error) {
      setMissionError(error instanceof Error ? error.message : '任务启动失败。')
    } finally {
      setMissionBusy(null)
    }
  }
  const controlMission = async (action: 'pause' | 'stop' | 'emergency' | MissionMode): Promise<void> => {
    if ((action === 'stop' || action === 'emergency') && armedMissionAction !== action) {
      setArmedMissionAction(action)
      setMissionNotice(action === 'stop' ? '再次点击确认停止任务并降落。' : '再次点击确认任务急停并降落。')
      return
    }
    setArmedMissionAction(null)
    setMissionError(null)
    setMissionNotice(null)
    setMissionBusy(action)
    try {
      if (action === 'pause') await window.phantomFilmer.toggleMissionPause()
      else if (action === 'stop') await window.phantomFilmer.stopMission()
      else if (action === 'emergency') await window.phantomFilmer.emergencyStopMission()
      else await window.phantomFilmer.selectControlMode(action)
      setMissionNotice(
        action === 'pause' ? (missionPaused ? '任务恢复请求已发送。' : '任务暂停请求已发送。')
          : action === 'stop' ? '任务已停止并降落。'
            : action === 'emergency' ? '任务已急停并降落。'
              : `正在切换到${missionModeLabel(action)}。`
      )
    } catch (error) {
      setMissionError(error instanceof Error ? error.message : '任务操作失败。')
    } finally {
      setMissionBusy(null)
    }
  }
  const controlPreview = async (): Promise<void> => {
    setMissionError(null)
    setMissionNotice(null)
    setMissionBusy(preview?.active ? 'preview-stop' : 'preview-start')
    try {
      if (preview?.active) {
        await window.phantomFilmer.stopPreview()
        setMissionNotice('地面识别预览已停止。')
      } else {
        await window.phantomFilmer.startPreview(profileName.trim())
        setMissionNotice('模型已就绪；请让目标人物进入画面并等待连续确认。')
      }
    } catch (error) {
      setMissionError(error instanceof Error ? error.message : '地面识别预览操作失败。')
    } finally {
      setMissionBusy(null)
    }
  }
  const missions = [
    ['manual', '手动飞行', '起降、悬停、四通道 RC 与独占控制租约'],
    ['follow', '普通自动跟随', 'ReID 锁定、距离与画面中心闭环'],
    ['reid_follow', '人物跟拍任务', '本地人物档案、丢失确认与有界搜索'],
    ['fixed_demo', '固定航线演示', '预设航线完成后衔接自动跟随']
  ] as const
  return (
    <section className="secondary-workspace" aria-label="任务与人物">
      <header className="workspace-title">
        <div><p className="eyebrow">MISSION WORKSPACE</p><h1>任务与人物</h1></div>
        <p>能力由 sidecar `/api/v1/capabilities` 决定；未开放功能不会向真机发送命令。</p>
      </header>
      <div className="mission-layout">
        <section className="mission-catalog">
          <div className="subheading"><div><p className="eyebrow">MISSION TYPES</p><h2>飞行任务</h2></div></div>
          <section className={`ground-preview ${preview?.confirmed ? 'confirmed' : ''}`} aria-label="地面人物识别预览">
            <div className="ground-preview-feed">
              {videoUrl ? <img src={videoUrl} alt="地面人物识别预览画面" /> : <div><strong>等待视频连接</strong><span>先在飞行控制连接真机并确认有效视频帧。</span></div>}
            </div>
            <div className="ground-preview-status">
              <p className="eyebrow">GROUND REID CHECK</p>
              <h3>{preview?.confirmed ? '目标人物已确认' : preview?.active ? (preview.found ? '正在稳定确认目标' : '正在搜索目标人物') : '起飞前人物确认'}</h3>
              <dl>
                <div><dt>档案</dt><dd>{preview?.profileName ?? (profileName || '—')}</dd></div>
                <div><dt>连续帧</dt><dd>{preview?.stableFrames ?? 0} / {preview?.requiredStableFrames ?? runtime.capabilities?.preview?.stableFrames ?? '—'}</dd></div>
                <div><dt>相似度</dt><dd>{typeof preview?.similarity === 'number' ? preview.similarity.toFixed(3) : '—'}</dd></div>
                <div><dt>朝向</dt><dd>{typeof preview?.orientationDeg === 'number' ? `${preview.orientationDeg.toFixed(1)}°` : '—'}</dd></div>
                <div><dt>候选人数</dt><dd>{preview?.candidateCount ?? 0}</dd></div>
                <div><dt>识别率</dt><dd>{preview?.fps ? `${preview.fps.toFixed(1)} Hz` : '—'}</dd></div>
              </dl>
              <button disabled={missionBusy != null || missionRunning || (!preview?.active && (readiness?.available !== true || !profileName.trim() || !allowedActions.has('start_preview')))} onClick={() => void controlPreview()}>{missionBusy === 'preview-start' ? '正在加载模型…' : missionBusy === 'preview-stop' ? '正在停止…' : preview?.active ? '停止识别预览' : '启动识别预览'}</button>
              {preview?.error && <p className="runtime-error">{preview.error}</p>}
            </div>
          </section>
          <div className="mission-grid">
            {missions.map(([id, title, detail]) => {
              const implemented = available.has(id)
              const isManual = id === 'manual'
              const ready = isManual || readiness?.available === true
              const enabled = implemented && ready
              const previewRequired = runtime.capabilities?.preview?.requiredForAutomaticMission === true
              const previewReady = !previewRequired || (preview?.confirmed === true && preview.profileName === profileName)
              const disabled = !enabled || missionBusy != null || (!isManual && (!profileName.trim() || missionRunning || !previewReady || !allowedActions.has('start_mission')))
              const label = isManual ? '前往飞行控制' : !implemented ? '等待任务接口' : !ready ? '模型资产未就绪' : missionRunning ? '已有任务运行中' : !previewReady ? '请先完成地面人物确认' : !allowedActions.has('start_mission') ? '请先连接并完成预检' : armedMission === id ? '再次点击确认起飞' : '启动任务'
              return <article className={enabled ? 'available' : 'unavailable'} key={id}><span>{enabled ? '已开放' : implemented ? '运行资产未就绪' : '后端尚未开放'}</span><h3>{title}</h3><p>{detail}</p><button disabled={disabled} onClick={isManual ? onOpenFlight : () => void startMission(id)}>{label}</button></article>
            })}
          </div>
        </section>
        <aside className="mission-flow">
          <p className="eyebrow">MISSION SETUP</p><h2>任务设置</h2>
          <label className="mission-field"><span>人物档案</span><select value={profileName} onChange={(event) => setProfileName(event.target.value)} disabled={missionRunning}><option value="">请选择人物档案</option>{profiles.map((profile) => <option key={profile.name} value={profile.name}>{profile.name} · {profile.photoCount ?? '?'} 张照片</option>)}</select></label>
          <div className="profile-enroll"><input aria-label="新人物档案名" value={enrollmentName} onChange={(event) => setEnrollmentName(event.target.value)} placeholder="新档案名" disabled={runtime.snapshot?.connected === true || missionBusy != null} /><button disabled={!enrollmentName.trim() || runtime.snapshot?.connected === true || missionBusy != null} onClick={() => void enrollProfile()}>{missionBusy === 'enroll' ? '正在建档…' : '选择照片并建档'}</button></div>
          <label className="mission-field"><span>初始控制模式</span><select value={initialMode} onChange={(event) => setInitialMode(event.target.value as typeof initialMode)} disabled={missionRunning}><option value="normal">普通跟随</option><option value="side">侧向跟随</option><option value="front">前向跟随</option><option value="manual">手动接管</option></select></label>
          <label className="mission-check"><input type="checkbox" checked={obstacleEnabled} onChange={(event) => setObstacleEnabled(event.target.checked)} disabled={missionRunning} /><span>普通模式启用自动避障</span></label>
          {readiness && !readiness.available && <p className="boundary-note">缺少运行资产：{readiness.missingAssets.join('、')}。任务按钮保持禁用。</p>}
          {missionError && <p className="runtime-error">{missionError}</p>}
          {missionNotice && <p className="runtime-notice" role="status">{missionNotice}</p>}
          {missionRunning && <div className="mission-live"><strong>{missionKindLabel(activeMission)} · {missionPaused ? '已暂停' : runtime.snapshot?.flightState}</strong><div><button disabled={missionBusy != null} onClick={() => void controlMission('pause')}>{missionPaused ? '继续任务' : '暂停悬停'}</button><button disabled={missionBusy != null} onClick={() => void controlMission('stop')}>{armedMissionAction === 'stop' ? '再次确认停止并降落' : '停止并降落'}</button><button className="danger" disabled={missionBusy != null} onClick={() => void controlMission('emergency')}>{armedMissionAction === 'emergency' ? '再次确认急停并降落' : '急停并降落'}</button></div><div className="mode-actions">{(['normal', 'side', 'front', 'manual'] as const).map((mode) => <button className={runtime.snapshot?.controlMode === mode ? 'active' : ''} disabled={missionBusy != null || missionPaused || runtime.snapshot?.controlMode === mode} key={mode} onClick={() => void controlMission(mode)}>{missionModeLabel(mode)}</button>)}</div></div>}
          <p className="eyebrow flow-label">OPERATOR FLOW</p><h2>运行前工作流</h2>
          <ol>
            <li><b>1</b><div><strong>选择或注册人物</strong><span>本地照片、特征摘要与模型兼容性</span></div></li>
            <li><b>2</b><div><strong>地面预览确认</strong><span>视频、ReID 结果、朝向与性能检查</span></div></li>
            <li><b>3</b><div><strong>选择任务和安全参数</strong><span>普通 / 侧向 / 前向、搜索与避障</span></div></li>
            <li><b>4</b><div><strong>起飞后二次选择</strong><span>基础高度悬停，操作员明确授权任务</span></div></li>
          </ol>
          <p className="boundary-note">人物档案与自动任务已接入共享 FollowSession；地面 ReID 识别预览仍在下一阶段接入，启动前请先从飞行控制确认视频、环境和遥测。</p>
        </aside>
      </div>
    </section>
  )
}

function DiagnosticsWorkspace({ runtime, backend }: { runtime: RuntimeFeed; backend: BackendState }): ReactElement {
  const snapshot = runtime.snapshot
  return (
    <section className="secondary-workspace" aria-label="运行诊断">
      <header className="workspace-title">
        <div><p className="eyebrow">RUNTIME DIAGNOSTICS</p><h1>运行诊断</h1></div>
        <p>命令、快照和事件序号均来自 Python 权威运行时。</p>
      </header>
      <div className="diagnostics-grid">
        <section className="runtime-summary">
          <h2>当前快照</h2>
          <dl>
            <div><dt>后端</dt><dd>{backend.status}</dd></div>
            <div><dt>阶段</dt><dd>{snapshot?.phase ?? '—'}</dd></div>
            <div><dt>任务</dt><dd>{snapshot?.mission ?? '—'}</dd></div>
            <div><dt>控制模式</dt><dd>{snapshot?.controlMode ?? '—'}</dd></div>
            <div><dt>事件序号</dt><dd>{snapshot?.sequence ?? 0}</dd></div>
            <div><dt>RC 租约</dt><dd>{runtime.capabilities?.rcLease.required ? `${runtime.capabilities.rcLease.ttlMs} ms` : '—'}</dd></div>
          </dl>
          <h3>允许操作</h3>
          <div className="action-tags">{snapshot?.allowedActions.map((action) => <span key={action}>{action}</span>) ?? <span>none</span>}</div>
          {runtime.error && <p className="runtime-error">{runtime.error}</p>}
        </section>
        <section className="event-log">
          <div className="subheading"><div><p className="eyebrow">EVENT REPLAY</p><h2>最近事件</h2></div><span>{runtime.events.length} / 50</span></div>
          {runtime.events.length === 0 ? <p className="event-empty">尚无新事件；执行连接或状态刷新后会出现在这里。</p> : <ol>{[...runtime.events].reverse().map((event) => <li key={event.sequence}><time>#{event.sequence}</time><strong>{event.type}</strong><span>{String(event.payload.command ?? event.payload.leaseId ?? '')}</span></li>)}</ol>}
        </section>
      </div>
    </section>
  )
}

type Control = [string, string, string, PartialRcCommand, 'up' | 'left' | 'center' | 'right' | 'down']

function ControlPad({ title, controls, active, enabled, onStart, onStop }: { title: string; controls: Control[]; active: string | null; enabled: boolean; onStart: (name: string, command: PartialRcCommand) => void; onStop: () => Promise<void> }): ReactElement {
  return <div className="control-pad"><span className="control-title">{title}</span><div className="pad-grid">{controls.map(([name, key, label, command, position]) => <button key={name} className={`control-key ${position} ${active === name ? 'active' : ''}`} disabled={!enabled} onPointerDown={(event) => { event.preventDefault(); if (name.startsWith('hover')) void onStop(); else onStart(name, command) }} onPointerUp={() => void onStop()} onPointerCancel={() => void onStop()}><kbd>{key}</kbd><span>{label}</span></button>)}</div></div>
}

function ActionButton({ icon, label, hint, disabled, armed, onClick }: { icon: 'takeoff' | 'land' | 'hover'; label: string; hint: string; disabled: boolean; armed?: boolean; onClick: () => void }): ReactElement {
  return <button className={`action-button ${armed ? 'armed' : ''}`} disabled={disabled} onClick={onClick}><Icon name={icon} /><strong>{label}</strong><small>{hint}</small></button>
}

function Metric({ label, value, unit }: { label: string; value: string | number; unit: string }): ReactElement {
  return <article><span>{label}</span><strong>{value}</strong><small>{unit}</small></article>
}

function StatusPill({ good, label }: { good: boolean; label: string }): ReactElement {
  return <div className="status-pill"><i className={good ? 'good' : ''} /><span>{label}</span></div>
}
