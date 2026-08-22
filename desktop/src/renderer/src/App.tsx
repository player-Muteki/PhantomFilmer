import { useCallback, useEffect, useMemo, useRef, useState, type ReactElement } from 'react'
import type { BackendState, DroneStatus, ProfileDetails, ProfileSummary, RcCommand } from '../../preload/api'
import { Icon } from './Icons'
import { useRuntimeFeed } from './app/useRuntimeFeed'
import { emptyCommand, missionKey, missionModeLabel, type MissionMode, type MissionType, type PartialRcCommand } from './app/types'
import { errorMessage, type ArmedAction, type ConnectionState } from './app/ui'
import { FlightStatusStrip } from './components/FlightStatusStrip'
import { ManualControls } from './components/ManualControls'
import { ModeRow } from './components/ModeRow'
import { ProfilePanel } from './components/ProfilePanel'
import type { SetupTab } from './components/SetupTabs'
import { TelemetryFooter } from './components/TelemetryFooter'
import { VideoPanel } from './components/VideoPanel'

const initialBackend: BackendState = { status: 'starting', version: '—', logDir: '—', airborne: false, restartAllowed: false }
const VIDEO_AUTO_RETRIES = 3

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
  const [videoBroken, setVideoBroken] = useState(false)
  const [profiles, setProfiles] = useState<ProfileSummary[]>([])
  const [profileName, setProfileName] = useState('')
  const [profileDetails, setProfileDetails] = useState<ProfileDetails | null>(null)
  const [enrollmentName, setEnrollmentName] = useState('')
  const [pendingPhotos, setPendingPhotos] = useState<string[] | null>(null)
  const [missionType, setMissionType] = useState<MissionType>('follow')
  const [keyboardControl, setKeyboardControl] = useState(true)
  const [activeTab, setActiveTab] = useState<SetupTab>('profiles')
  const controlTimer = useRef<number | null>(null)
  const rcInFlight = useRef(false)
  const videoRetries = useRef(0)
  const videoRetryTimer = useRef<number | null>(null)

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
  const allowed = useMemo(
    () => new Set(missionRunning ? (runtime.snapshot?.allowedActions ?? ['stop_mission', 'emergency_stop_mission', 'select_control_mode', 'toggle_mission_pause']) : (runtime.snapshot?.allowedActions ?? [])),
    [missionRunning, runtime.snapshot?.allowedActions]
  )
  const canStartMission = new Set(runtime.capabilities?.missions ?? []).has(missionType)
    && missionReady
    && profileName.trim().length > 0
    && !missionRunning
    && allowed.has('start_mission')
  const canSelectMode = missionRunning && allowed.has('select_control_mode')
  const canTogglePause = missionRunning && allowed.has('toggle_mission_pause')
  const stopEnabled = missionRunning && allowed.has('stop_mission') && !controlsLocked
  const emergencyEnabled = (missionRunning && allowed.has('emergency_stop_mission')) || (airborne && !missionRunning)
  const safetyAlert = connected && status.safetyReason ? status.safetyReason : null
  const canReconnectVideo = videoReady && videoUrl == null && videoBroken
  const safety = runtime.capabilities?.safety

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
    let cancelled = false
    if (!backendReady || !profileName) {
      setProfileDetails(null)
      return
    }
    void window.phantomFilmer.getProfile(profileName).then((details) => {
      if (!cancelled) setProfileDetails(details)
    }).catch((error) => {
      if (!cancelled) setNotice(errorMessage(error, '人物档案详情读取失败'))
    })
    return () => { cancelled = true }
  }, [backendReady, profileName])

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
        if (!cancelled) setStatus(next)
      } catch (error) {
        if (!cancelled) setNotice(errorMessage(error, '真机遥测暂时不可用'))
      }
    }
    const timer = window.setInterval(() => void refresh(), 1000)
    return () => { cancelled = true; window.clearInterval(timer) }
  }, [connected])

  const retryVideo = useCallback(async (): Promise<void> => {
    try {
      const url = await window.phantomFilmer.getVideoUrl()
      videoRetries.current = 0
      setVideoBroken(false)
      setVideoUrl(url)
    } catch (error) {
      setVideoBroken(true)
      setNotice(errorMessage(error, '无法创建安全视频会话'))
      scheduleVideoRetry()
    }
  }, [])

  function scheduleVideoRetry(): void {
    if (videoRetryTimer.current != null) return
    if (videoRetries.current >= VIDEO_AUTO_RETRIES) return
    videoRetries.current += 1
    videoRetryTimer.current = window.setTimeout(() => {
      videoRetryTimer.current = null
      void retryVideo()
    }, 1000 * videoRetries.current)
  }

  const handleVideoError = useCallback((): void => {
    setVideoUrl(null)
    setVideoBroken(true)
    scheduleVideoRetry()
  }, [retryVideo])

  const retryVideoManually = useCallback((): void => {
    videoRetries.current = 0
    if (videoRetryTimer.current != null) {
      window.clearTimeout(videoRetryTimer.current)
      videoRetryTimer.current = null
    }
    void retryVideo()
  }, [retryVideo])

  useEffect(() => {
    if (!videoReady) {
      if (videoRetryTimer.current != null) {
        window.clearTimeout(videoRetryTimer.current)
        videoRetryTimer.current = null
      }
      videoRetries.current = 0
      setVideoBroken(false)
      setVideoUrl(null)
      return
    }
    void retryVideo()
  }, [videoReady, retryVideo])

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

  const disconnectDrone = async (): Promise<void> => {
    if (!connected || missionRunning || airborne || actionBusy != null) return
    setActionBusy('disconnect')
    try {
      await window.phantomFilmer.stop()
      setConnection('disconnected')
      setStatus({})
      setNotice('已断开真机连接。')
    } catch (error) {
      setNotice(errorMessage(error, '断开真机失败'))
    } finally { setActionBusy(null) }
  }

  const openLogs = async (): Promise<void> => {
    try {
      await window.phantomFilmer.openLogDir()
    } catch (error) {
      setNotice(errorMessage(error, '无法打开日志目录'))
    }
  }

  const pickPhotos = async (): Promise<void> => {
    const name = enrollmentName.trim()
    if (!name) return
    setActionBusy('enroll-pick')
    try {
      const paths = await window.phantomFilmer.pickProfilePhotos()
      if (paths && paths.length > 0) setPendingPhotos(paths)
    } catch (error) {
      setNotice(errorMessage(error, '选择参考照片失败'))
    } finally { setActionBusy(null) }
  }

  const confirmEnroll = async (): Promise<void> => {
    const name = enrollmentName.trim()
    if (!name || pendingPhotos == null) return
    const overwrite = profiles.some((profile) => profile.name === name)
    if (overwrite && !window.confirm(`人物档案“${name}”已存在。\n\n是否使用新选择的照片覆盖原档案？`)) return
    setActionBusy('enroll')
    try {
      const profile = await window.phantomFilmer.enrollProfile(name, pendingPhotos, overwrite)
      if (profile) {
        setProfiles((current) => [...current.filter((item) => item.name !== profile.name), profile])
        setProfileName(profile.name)
        setEnrollmentName('')
        setPendingPhotos(null)
        setNotice(`人物档案“${profile.name}”已创建并选中。`)
      }
    } catch (error) { setNotice(errorMessage(error, '人物建档失败')) } finally { setActionBusy(null) }
  }

  const cancelEnroll = (): void => {
    setPendingPhotos(null)
  }

  const replaceProfilePhotos = async (): Promise<void> => {
    if (!profileName || connected || actionBusy != null) return
    setActionBusy('profile-update')
    try {
      const paths = await window.phantomFilmer.pickProfilePhotos()
      if (!paths?.length) return
      if (!window.confirm(`是否使用新选择的 ${paths.length} 张照片替换人物档案“${profileName}”的全部参考特征？`)) return
      const profile = await window.phantomFilmer.enrollProfile(profileName, paths, true)
      if (profile) {
        await refreshProfiles()
        setProfileDetails(await window.phantomFilmer.getProfile(profileName))
        setNotice(`人物档案“${profileName}”的参考照片已更新。`)
      }
    } catch (error) {
      setNotice(errorMessage(error, '人物档案更新失败'))
    } finally { setActionBusy(null) }
  }

  const renameProfile = async (requestedName: string): Promise<void> => {
    if (!profileName || connected || actionBusy != null) return
    const nextName = requestedName.trim()
    if (!nextName || nextName === profileName) return
    setActionBusy('profile-rename')
    try {
      const profile = await window.phantomFilmer.renameProfile(profileName, nextName)
      await refreshProfiles()
      setProfileName(profile.name)
      setProfileDetails(profile)
      setNotice(`人物档案已重命名为“${profile.name}”。`)
    } catch (error) {
      setNotice(errorMessage(error, '人物档案重命名失败'))
    } finally { setActionBusy(null) }
  }

  const deleteProfile = async (): Promise<void> => {
    if (!profileName || connected || actionBusy != null) return
    if (!window.confirm(`确认删除人物档案“${profileName}”？\n\n档案会移入本地回收目录，不会立即永久擦除。`)) return
    const deletedName = profileName
    setActionBusy('profile-delete')
    try {
      await window.phantomFilmer.deleteProfile(deletedName)
      setProfileDetails(null)
      await refreshProfiles()
      setNotice(`人物档案“${deletedName}”已删除。`)
    } catch (error) {
      setNotice(errorMessage(error, '人物档案删除失败'))
    } finally { setActionBusy(null) }
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
      await window.phantomFilmer.startMission({ mission: missionType, profileName: profileName.trim(), initialControlMode: 'manual' })
      setNotice('正在起飞并上升至 150 cm；到达后请选择跟随模式。')
    } catch (error) { setNotice(errorMessage(error, '自动任务启动失败')) } finally { setActionBusy(null) }
  }

  const runMissionCommand = useCallback(async (action: 'stop' | 'emergency' | MissionMode): Promise<void> => {
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
  }, [armedAction, missionRunning, runtime.snapshot?.controlMode])

  const togglePause = useCallback(async (): Promise<void> => {
    setActionBusy('mission-pause')
    try {
      await window.phantomFilmer.toggleMissionPause()
      setStatus(await window.phantomFilmer.status())
      setNotice('已发送暂停/继续指令。')
    } catch (error) { setNotice(errorMessage(error, '暂停指令失败')) } finally { setActionBusy(null) }
  }, [])

  const sendRc = useCallback(async (partial: PartialRcCommand): Promise<void> => {
    if (rcInFlight.current) return
    rcInFlight.current = true
    try { await window.phantomFilmer.moveRc({ ...emptyCommand, ...partial } as RcCommand) } catch (error) { setNotice(errorMessage(error, '手动控制被安全系统拒绝')) } finally { rcInFlight.current = false }
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
      const target = event.target as HTMLElement | null
      if (target != null && ['INPUT', 'SELECT', 'TEXTAREA'].includes(target.tagName)) return
      const key = event.key.toLowerCase()
      const missionKeyPressed = key === 'm' || key === 'p' || (!manualControlEnabled && ['a', 's', 'f', '1', '2', '3', 'q', 'e'].includes(key))
      if (missionRunning && !event.repeat && missionKeyPressed) {
        event.preventDefault()
        if (key === 'q') { void runMissionCommand('stop'); return }
        if (key === 'e') { void runMissionCommand('emergency'); return }
        if (key === 'p') { void togglePause(); return }
        void window.phantomFilmer.inputKey(key).then(async () => setStatus(await window.phantomFilmer.status())).catch((error) => setNotice(errorMessage(error, '飞行模式按键被后端拒绝')))
        return
      }
      if (!keyboardControl) return
      if (event.code === 'Space') { event.preventDefault(); if (!event.repeat) void stopControl(); return }
      const entry = commands[event.code]
      if (!entry || event.repeat) return
      event.preventDefault()
      startControl(entry[0], entry[1])
    }
    const keyUp = (event: KeyboardEvent): void => {
      if (!keyboardControl) return
      if (commands[event.code]) { event.preventDefault(); void stopControl() }
    }
    window.addEventListener('keydown', keyDown)
    window.addEventListener('keyup', keyUp)
    window.addEventListener('blur', stopControl)
    return () => {
      window.removeEventListener('keydown', keyDown)
      window.removeEventListener('keyup', keyUp)
      window.removeEventListener('blur', stopControl)
    }
  }, [keyboardControl, manualControlEnabled, missionRunning, runMissionCommand, speed, startControl, stopControl, togglePause])

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

  return <main className="app-shell core-shell">
    <header className="topbar core-topbar">
      <div className="brand" aria-label="PhantomFilmer 桌面飞控台">
        <span className="brand-mark" aria-hidden="true"><i /><i /><i /></span>
        <div><strong>PhantomFilmer</strong><small>FOLLOWING CONSOLE</small></div>
      </div>
      <div className="system-summary">
        <StatusPill good={backendReady} label={backendReady ? '后端在线' : '后端离线'} />
        <StatusPill good={connected} label={connected ? '真机已连接' : '真机未连接'} />
        <button className="button secondary" onClick={() => void openLogs()} title={`日志目录：${backend.logDir}`}>日志</button>
        <button className="button secondary" disabled={!connected || missionRunning || airborne || actionBusy != null} onClick={() => void disconnectDrone()}>断开真机</button>
      </div>
    </header>
    {backend.status === 'offline' && <section className={`diagnostic ${backend.airborne ? 'critical' : ''}`} role="alert">
      <div className="diagnostic-icon"><Icon name={backend.airborne ? 'emergency' : 'activity'} /></div>
      <div><h1>{backend.airborne ? '后端中断 · 最后状态为空中' : '本地后端未运行'}</h1><p>{backend.error ?? '请检查后端后重试。'}</p></div>
      <button className="button secondary" disabled={!backend.restartAllowed || actionBusy != null} onClick={() => void retryBackend()}>{backend.airborne ? '禁止自动重启' : '重启后端'}</button>
    </section>}
    <section className="core-layout" aria-label="自动跟随控制台">
      <ProfilePanel
        profiles={profiles}
        profileDetails={profileDetails}
        profileName={profileName}
        onProfileName={setProfileName}
        enrollmentName={enrollmentName}
        onEnrollmentName={setEnrollmentName}
        pendingPhotos={pendingPhotos}
        onPickPhotos={() => void pickPhotos()}
        onConfirmEnroll={() => void confirmEnroll()}
        onCancelEnroll={cancelEnroll}
        onReplaceProfile={() => void replaceProfilePhotos()}
        onRenameProfile={(nextName) => void renameProfile(nextName)}
        onDeleteProfile={() => void deleteProfile()}
        enrollBusy={actionBusy === 'enroll'}
        connected={connected}
        missionRunning={missionRunning}
        actionBusy={actionBusy != null}
        missionType={missionType}
        onMissionType={setMissionType}
        canStartMission={canStartMission}
        launchArmed={armedAction === 'start'}
        onLaunch={() => void startMission()}
        missingAssets={runtime.capabilities?.missionReadiness?.missingAssets ?? []}
        preflight={status.preflight}
        events={runtime.events}
        activeTab={activeTab}
        onActiveTab={setActiveTab}
      />
      <section className="flight-panel">
        <FlightStatusStrip
          flightState={status.flightState ?? runtime.snapshot?.flightState}
          controlHz={status.controlHz}
          paused={missionPaused === true}
          batteryFresh={status.batteryFresh}
          heightFresh={status.heightFresh}
          connected={connected}
          alert={safetyAlert}
          profile={profileName || null}
          syncError={runtime.error}
        />
        <ModeRow
          controlMode={runtime.snapshot?.controlMode}
          missionRunning={missionRunning}
          paused={missionPaused === true}
          awaitingModeSelection={awaitingModeSelection}
          controlsLocked={controlsLocked}
          canSelectMode={canSelectMode}
          canTogglePause={canTogglePause}
          onMode={(mode) => void runMissionCommand(mode)}
          onPause={() => void togglePause()}
        />
        <VideoPanel
          videoUrl={videoUrl}
          connection={connection}
          backendReady={backendReady}
          connected={connected}
          awaitingModeSelection={awaitingModeSelection}
          canReconnectVideo={canReconnectVideo}
          onConnect={() => void connectDrone()}
          onRetryVideo={retryVideoManually}
          onVideoError={handleVideoError}
          overlay={manualControlEnabled ? (
            <ManualControls
              speed={speed}
              onSpeed={setSpeed}
              activeControl={activeControl}
              keyboardEnabled={keyboardControl}
              onKeyboardToggle={setKeyboardControl}
              controlsLocked={controlsLocked}
              onStart={startControl}
              onStop={stopControl}
            />
          ) : airborne ? (
            <div className="manual-hint-badge">手动接管后此处显示控制板（键 M）</div>
          ) : null}
        >
          <div className="notice" role="status"><b>{actionBusy ? '处理中' : '状态'}</b><span>{notice}</span></div>
        </VideoPanel>
        <TelemetryFooter
          connected={connected}
          battery={status.battery ?? null}
          batteryFresh={status.batteryFresh}
          heightCm={status.heightCm ?? null}
          heightFresh={status.heightFresh}
          frontTofCm={status.frontTofCm ?? null}
          frontTofState={status.frontTofState}
          minTakeoffBattery={safety?.minTakeoffBattery}
          lowBatteryLand={safety?.lowBatteryLand}
          maxHeightCm={safety?.maxHeightCm}
          stopEnabled={stopEnabled}
          stopArmed={armedAction === 'stop'}
          emergencyEnabled={emergencyEnabled && !controlsLocked}
          emergencyArmed={armedAction === 'emergency'}
          onStop={() => void runMissionCommand('stop')}
          onEmergency={() => void runMissionCommand('emergency')}
        />
      </section>
    </section>
  </main>
}

function StatusPill({ good, label }: { good: boolean; label: string }): ReactElement {
  return <div className="status-pill"><i className={good ? 'good' : ''} /><span>{label}</span></div>
}
