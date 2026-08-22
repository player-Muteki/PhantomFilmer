import { useMemo, useState, type ReactElement } from 'react'
import type { RuntimeEvent, RuntimeSnapshot } from '../../../preload/api'
import { eventView, type EventCategory, type EventTone } from '../app/eventLabels'
import { flightStateView } from '../app/flightStates'

type TimelineEntry = {
  key: string
  occurredAt: number
  category: EventCategory
  tone: EventTone
  title: string
  detail?: string
  mode?: string
}

const FILTERS: Array<{ key: 'all' | EventCategory; label: string }> = [
  { key: 'all', label: '全部' },
  { key: 'flight', label: '飞行' },
  { key: 'mode', label: '模式' },
  { key: 'obstacle', label: '避障' },
  { key: 'safety', label: '安全' },
  { key: 'profile', label: '档案' }
]

const MODE_LABELS: Record<string, string> = {
  none: '未选择', manual: '手动', normal: '普通', side: '侧向', front: '前向'
}
const PHASE_LABELS: Record<string, string> = {
  disconnected: '未连接', connecting: '连接中', preflight: '起飞准备', taking_off: '起飞中',
  airborne: '空中任务', landing: '降落中', stopping: '停止中', error: '异常'
}

function behaviorCategory(state: string): EventCategory {
  const value = state.toUpperCase()
  if (/(OBSTACLE|AVOID|BYPASS|CENTER_LOSS_ADVANCE)/.test(value)) return 'obstacle'
  if (/(LANDING|EMERGENCY|FAIL|ERROR|LOW_BATTERY|HEIGHT_LIMIT)/.test(value)) return 'safety'
  return 'behavior'
}

function formatTime(occurredAt: number): string {
  return new Date(occurredAt * 1000).toLocaleTimeString('zh-CN', {
    hour12: false,
    hour: '2-digit',
    minute: '2-digit',
    second: '2-digit',
    fractionalSecondDigits: 3
  })
}

function changedEntries(event: RuntimeEvent, previous: RuntimeSnapshot | null): TimelineEntry[] {
  const snapshot = event.snapshot
  if (!snapshot) return []
  const entries: TimelineEntry[] = []
  const suffix = (name: string): string => `${event.sequence}-${name}`
  if (!previous || snapshot.phase !== previous.phase) {
    entries.push({
      key: suffix('phase'), occurredAt: event.occurredAt, category: 'flight', tone: snapshot.phase === 'error' ? 'critical' : 'info',
      title: `飞行阶段 · ${PHASE_LABELS[snapshot.phase] ?? snapshot.phase}`, detail: snapshot.error ?? undefined, mode: MODE_LABELS[snapshot.controlMode]
    })
  }
  if (!previous || snapshot.controlMode !== previous.controlMode) {
    entries.push({
      key: suffix('mode'), occurredAt: event.occurredAt, category: 'mode', tone: 'info',
      title: `切换至${MODE_LABELS[snapshot.controlMode] ?? snapshot.controlMode}模式`, mode: MODE_LABELS[snapshot.controlMode]
    })
  }
  if (!previous || snapshot.flightState !== previous.flightState) {
    const view = flightStateView(snapshot.flightState)
    const category = behaviorCategory(snapshot.flightState)
    entries.push({
      key: suffix('behavior'), occurredAt: event.occurredAt, category,
      tone: view.tone === 'critical' ? 'critical' : view.tone === 'caution' ? 'warning' : 'success',
      title: view.label, detail: snapshot.error ?? undefined, mode: MODE_LABELS[snapshot.controlMode]
    })
  }
  return entries
}

export function buildTimelineEntries(events: RuntimeEvent[]): TimelineEntry[] {
  const entries: TimelineEntry[] = []
  let previous: RuntimeSnapshot | null = null
  for (const event of events) {
    const view = eventView(event.type, event.payload)
    const isStatusNoise = event.type === 'command.accepted'
      || (event.type === 'command.completed' && event.payload.command === 'device.status.refresh')
      || event.type === 'mission.manual_rc.accepted'
    if (!isStatusNoise && view.label) {
      entries.push({
        key: `${event.sequence}-event`, occurredAt: event.occurredAt,
        category: view.category, tone: view.tone, title: view.label,
        detail: typeof event.payload.reason === 'string' ? event.payload.reason
          : typeof event.payload.error === 'string' && event.payload.error ? event.payload.error : undefined,
        mode: event.snapshot ? MODE_LABELS[event.snapshot.controlMode] : undefined
      })
    }
    entries.push(...changedEntries(event, previous))
    if (event.snapshot) previous = event.snapshot
  }
  return entries.reverse()
}

/** Timestamped semantic visualization over the authoritative runtime event stream. */
export function EventTimeline({ events }: { events: RuntimeEvent[] }): ReactElement {
  const [filter, setFilter] = useState<'all' | EventCategory>('all')
  const entries = useMemo(() => buildTimelineEntries(events), [events])
  const visible = filter === 'all' ? entries : entries.filter((entry) => entry.category === filter || (filter === 'flight' && entry.category === 'behavior'))
  return (
    <div className="event-timeline" aria-label="运行事件时间线">
      <div className="timeline-filters" aria-label="事件筛选">
        {FILTERS.map((item) => (
          <button key={item.key} className={filter === item.key ? 'active' : ''} onClick={() => setFilter(item.key)}>{item.label}</button>
        ))}
      </div>
      <ol className="timeline-list">
        {visible.length === 0 && <li className="empty">暂无事件</li>}
        {visible.map((entry) => (
          <li key={entry.key} className={`tone-${entry.tone} category-${entry.category}`}>
            <time>{formatTime(entry.occurredAt)}</time>
            <i aria-hidden="true" />
            <div><strong>{entry.title}</strong>{entry.detail && <small>{entry.detail}</small>}</div>
            {entry.mode && <span className="timeline-mode">{entry.mode}</span>}
          </li>
        ))}
      </ol>
    </div>
  )
}
