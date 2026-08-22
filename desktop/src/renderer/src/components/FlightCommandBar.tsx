import type { ReactElement } from 'react'
import type { MissionMode } from '../app/types'
import { missionModeLabel } from '../app/types'
import { flightStateView } from '../app/flightStates'

type Props = {
  flightState: string | undefined
  controlHz: number | undefined
  paused: boolean
  batteryFresh: boolean | undefined
  heightFresh: boolean | undefined
  backendReady: boolean
  connected: boolean
  alert?: string | null
  profile?: string | null
  syncError?: string | null
  controlMode: string | undefined
  missionRunning: boolean
  awaitingModeSelection: boolean
  controlsLocked: boolean
  canSelectMode: boolean
  canTogglePause: boolean
  disconnectDisabled: boolean
  onMode: (mode: MissionMode) => void
  onPause: () => void
  onDisconnect: () => void
}

const MODES: MissionMode[] = ['normal', 'side', 'front', 'manual']

/** One fixed-height situational-awareness and airborne-command toolbar. */
export function FlightCommandBar(props: Props): ReactElement {
  const {
    flightState, controlHz, paused, batteryFresh, heightFresh, backendReady, connected, alert, profile, syncError,
    controlMode, missionRunning, awaitingModeSelection, controlsLocked, canSelectMode, canTogglePause,
    disconnectDisabled, onMode, onPause, onDisconnect
  } = props
  const view = flightStateView(flightState)
  const staleTelemetry = connected && (batteryFresh === false || heightFresh === false)

  return (
    <section className={`flight-command-bar tone-${view.tone}`} aria-label="自动任务空中控制">
      <div className="command-system" aria-label="系统连接状态">
        <span title={backendReady ? '本地后端已就绪' : '本地后端离线'}><i className={backendReady ? 'good' : ''} />{backendReady ? '后端在线' : '后端离线'}</span>
        <span title={connected ? '真机链路已连接' : '真机链路未连接'}><i className={connected ? 'good' : ''} />{connected ? '真机已连接' : '真机未连接'}</span>
      </div>
      <div className="command-state" title={alert ?? view.label}>
        <i aria-hidden="true" />
        <strong>{connected ? view.label : '未连接真机'}</strong>
        {paused && <span className="command-badge paused">已暂停</span>}
        {staleTelemetry && <span className="command-badge stale" title="电量或高度数据陈旧">遥测!</span>}
      </div>

      <div className="flight-mode-group">
        {MODES.map((mode) => (
          <button
            key={mode}
            className={controlMode === mode ? 'active' : ''}
            disabled={
              !missionRunning || controlsLocked || paused || !canSelectMode
              || (!awaitingModeSelection && mode !== 'manual' && controlMode === mode)
            }
            onClick={() => onMode(mode)}
            title={mode === 'manual' && controlMode === 'manual' ? '退出手动并恢复自动跟随' : missionModeLabel(mode)}
          >
            {mode === 'manual' && controlMode === 'manual' ? '退出手动' : missionModeLabel(mode)}
          </button>
        ))}
      </div>

      <div className="command-context">
        {alert ? (
          <span className="command-alert" role="alert" title={alert}>{alert}</span>
        ) : (
          <>
            {syncError && <span className="command-badge stale" title={syncError}>事件!</span>}
            <span className="command-hz" title="控制循环频率"><strong>{connected && controlHz != null ? controlHz.toFixed(0) : '—'}</strong> Hz</span>
            {profile && <span className="command-profile" title={`当前人物档案：${profile}`}>档·{profile}</span>}
          </>
        )}
      </div>

      <button
        className={`pause-button ${paused ? 'active' : ''}`}
        disabled={!missionRunning || controlsLocked || !canTogglePause}
        onClick={onPause}
        title="暂停后任务悬停冻结，再按一次继续（键盘 P）"
      >
        {paused ? '继续任务' : '暂停任务'}
      </button>
      <button className="disconnect-button" disabled={disconnectDisabled} onClick={onDisconnect}>断开真机</button>
    </section>
  )
}
