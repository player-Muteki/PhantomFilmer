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
    stopEnabled, stopArmed, emergencyEnabled, emergencyArmed, onStop, onEmergency
  } = props
  const batteryLabel = !connected || battery == null ? '—' : `${battery}%`
  const heightLabel = !connected || heightCm == null ? '—' : `${heightCm} cm`
  const tofBlocked = connected && frontTofState === 'blocked'
  return (
    <footer className="flight-footer">
      <TelemetryMetric
        label={connected && batteryFresh === false ? '电量（数据陈旧）' : '电量'}
        value={batteryLabel}
        tone={batteryTone(battery, minTakeoffBattery, lowBatteryLand)}
      />
      <TelemetryMetric
        label={connected && heightFresh === false ? '高度（数据陈旧）' : '高度'}
        value={heightLabel}
        tone={heightTone(heightCm, maxHeightCm)}
      />
      <TelemetryMetric label="前向 ToF" value={tofValue(frontTofCm, frontTofState, connected)} tone={tofBlocked ? 'critical' : 'neutral'} />
      <button className={`stop-button ${stopArmed ? 'armed' : ''}`} disabled={!stopEnabled} onClick={onStop}>
        {stopArmed ? '确认停止并降落' : '停止并降落'}
      </button>
      <button className={`emergency-button ${emergencyArmed ? 'armed' : ''}`} disabled={!emergencyEnabled} onClick={onEmergency}>
        {emergencyArmed ? '再次确认急停' : '急停'}
      </button>
    </footer>
  )
}

function TelemetryMetric({ label, value, tone }: { label: string; value: string; tone: Tone }): ReactElement {
  return (
    <div className={`telemetry-metric tone-${tone}`}>
      <span>{label}</span>
      <strong>{value}</strong>
    </div>
  )
}
