import type { ReactElement } from 'react'
import { flightStateView } from '../app/flightStates'

type Props = {
  flightState: string | undefined
  controlHz: number | undefined
  paused: boolean
  batteryFresh: boolean | undefined
  heightFresh: boolean | undefined
  connected: boolean
  alert?: string | null
  profile?: string | null
  syncError?: string | null
}

/** Persistent situational-awareness strip above the video panel. */
export function FlightStatusStrip({ flightState, controlHz, paused, batteryFresh, heightFresh, connected, alert, profile, syncError }: Props): ReactElement {
  const view = flightStateView(flightState)
  const staleTelemetry = connected && (batteryFresh === false || heightFresh === false)
  return (
    <section className={`flight-strip tone-${view.tone}`} aria-label="飞行态势">
      <span className="flight-state-label">
        <i aria-hidden="true" />
        {connected ? view.label : '未连接真机'}
      </span>
      {paused && <span className="strip-badge paused">已暂停</span>}
      {staleTelemetry && <span className="strip-badge stale">遥测数据陈旧</span>}
      {syncError && <span className="strip-badge stale" title={syncError}>事件流同步失败</span>}
      <span className="strip-metric" title="控制循环频率">
        控制频率 <strong>{connected && controlHz != null ? controlHz.toFixed(0) : '—'}</strong> Hz
      </span>
      {profile != null && <span className="strip-profile" title="当前人物档案">档·{profile}</span>}
      {alert && <span className="strip-alert" role="alert">{alert}</span>}
    </section>
  )
}
