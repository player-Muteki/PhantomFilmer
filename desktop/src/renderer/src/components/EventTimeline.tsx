import type { ReactElement } from 'react'
import type { RuntimeEvent } from '../../../preload/api'
import { eventView } from '../app/eventLabels'

function formatTime(occurredAt: number): string {
  return new Date(occurredAt).toLocaleTimeString('zh-CN', { hour12: false })
}

/** Operator timeline over the runtime event stream; lives inside the events tab. */
export function EventTimeline({ events }: { events: RuntimeEvent[] }): ReactElement {
  const ordered = [...events].reverse()
  return (
    <div className="event-timeline" aria-label="运行事件时间线">
      <ul className="timeline-list">
        {ordered.length === 0 && <li className="empty">暂无事件</li>}
        {ordered.map((event) => {
          const view = eventView(event.type)
          return (
            <li key={event.sequence} className={`tone-${view.tone}`}>
              <time>{formatTime(event.occurredAt)}</time>
              <span>{view.label}</span>
            </li>
          )
        })}
      </ul>
    </div>
  )
}
