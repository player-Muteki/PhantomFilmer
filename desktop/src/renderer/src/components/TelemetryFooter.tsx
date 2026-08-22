import type { ReactElement } from 'react'

type Tone = 'ok' | 'warn' | 'critical' | 'neutral'

type Props = {
  connected: boolean
  battery: number | null
  batteryFresh: boolean | undefined
  heightCm: number | null
  heightFresh: boolean | undefined
  frontTofCm: number | null
  frontTofState: string | undefined
  minTakeoffBattery: number | undefined
  lowBatteryLand: number | undefined
  maxHeightCm: number | undefined
  stopEnabled: boolean
  stopArmed: boolean
  emergencyEnabled: boolean
  emergencyArmed: boolean
  notice: string
  busy: boolean
  onStop: () => void
  onEmergency: () => void
}

function batteryTone(battery: number | null, min: number | undefined, low: number | undefined): Tone {
  if (battery == null) return 'neutral'
  if (low != null && battery <= low) return 'critical'
  if (min != null && battery < min) return 'warn'
  return 'ok'
}

function heightTone(heightCm: number | null, maxHeightCm: number | undefined): Tone {
  if (heightCm == null || maxHeightCm == null || heightCm <= 0) return 'neutral'
  return heightCm >= maxHeightCm * 0.8 ? 'warn' : 'ok'
}

function tofValue(frontTofCm: number | null, frontTofState: string | undefined, connected: boolean): string {
  if (!connected) return '—'
  if (frontTofState === 'out_of_range') return '远'
  return typeof frontTofCm === 'number' ? `${frontTofCm} cm` : '—'
}

export function TelemetryFooter(props: Props): ReactElement {
  const {
    connected, battery, batteryFresh, heightCm, heightFresh, frontTofCm, frontTofState,
    minTakeoffBattery, lowBatteryLand, maxHeightCm,
    stopEnabled, stopArmed, emergencyEnabled, emergencyArmed, notice, busy, onStop, onEmergency
  } = props
  const batteryLabel = !connected || battery == null ? '—' : `${battery}%`
  const heightLabel = !connected || heightCm == null ? '—' : `${heightCm} cm`
  const tofBlocked = connected && frontTofState === 'blocked'
  return (
    <footer className="flight-footer" aria-label="遥测与飞行安全控制">
      <div className="telemetry-group">
        <TelemetryMetric
          label="电量"
          value={batteryLabel}
          tone={batteryTone(battery, minTakeoffBattery, lowBatteryLand)}
          title={connected && batteryFresh === false ? '电量数据陈旧' : '当前电量'}
        />
        <TelemetryMetric
          label="高度"
          value={heightLabel}
          tone={heightTone(heightCm, maxHeightCm)}
          title={connected && heightFresh === false ? '高度数据陈旧' : '当前高度'}
        />
        <TelemetryMetric label="ToF" value={tofValue(frontTofCm, frontTofState, connected)} tone={tofBlocked ? 'critical' : 'neutral'} title="前向 ToF 距离" />
      </div>
      <div className="footer-notice" role="status" title={notice}>
        <b>{busy ? '处理中' : '状态'}</b><span>{notice}</span>
      </div>
      <div className="flight-safety-actions">
        <button
          className={`stop-button ${stopArmed ? 'armed' : ''}`}
          aria-label={stopArmed ? '确认停止并降落' : '停止并降落'}
          disabled={!stopEnabled}
          onClick={onStop}
        >
          {stopArmed ? '确认降落' : '停止并降落'}
        </button>
        <button
          className={`emergency-button ${emergencyArmed ? 'armed' : ''}`}
          aria-label={emergencyArmed ? '再次确认急停' : '急停'}
          disabled={!emergencyEnabled}
          onClick={onEmergency}
        >
          {emergencyArmed ? '确认急停' : '急停'}
        </button>
      </div>
    </footer>
  )
}

function TelemetryMetric({ label, value, tone, title }: { label: string; value: string; tone: Tone; title: string }): ReactElement {
  return (
    <div className={`telemetry-metric tone-${tone}`} title={title}>
      <span>{label}</span>
      <strong>{value}</strong>
    </div>
  )
}
