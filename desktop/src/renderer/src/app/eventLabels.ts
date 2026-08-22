export type EventTone = 'info' | 'success' | 'warning' | 'critical'

export type EventView = { label: string; tone: EventTone }

const EVENT_LABELS: Record<string, EventView> = {
  'profile.enrolled': { label: '人物档案已建档', tone: 'success' },
  'rc.lease.acquired': { label: '获得手动控制权限', tone: 'info' },
  'rc.lease.released': { label: '释放手动控制权限', tone: 'info' },
  'mission.manual_rc.accepted': { label: '手动控制指令已接受', tone: 'info' },
  'mission.finished': { label: '任务结束', tone: 'success' },
  'mission.failed': { label: '任务失败', tone: 'critical' },
  'flight.safety_landing.started': { label: '安全降落已触发', tone: 'warning' },
  'flight.safety_landing.failed': { label: '安全降落失败', tone: 'critical' }
}

export function eventView(type: string): EventView {
  return EVENT_LABELS[type] ?? { label: type, tone: 'info' }
}
