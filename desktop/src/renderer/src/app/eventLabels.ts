export type EventTone = 'info' | 'success' | 'warning' | 'critical'
export type EventCategory = 'flight' | 'mode' | 'behavior' | 'obstacle' | 'safety' | 'profile' | 'system'
export type EventView = { label: string; tone: EventTone; category: EventCategory }

const EVENT_LABELS: Record<string, EventView> = {
  'profile.enrolled': { label: '人物档案已建档', tone: 'success', category: 'profile' },
  'profile.renamed': { label: '人物档案已重命名', tone: 'success', category: 'profile' },
  'profile.deleted': { label: '人物档案已删除', tone: 'warning', category: 'profile' },
  'rc.lease.acquired': { label: '获得手动控制权限', tone: 'info', category: 'mode' },
  'rc.lease.released': { label: '释放手动控制权限', tone: 'info', category: 'mode' },
  'mission.finished': { label: '任务结束', tone: 'success', category: 'flight' },
  'mission.failed': { label: '任务失败', tone: 'critical', category: 'safety' },
  'flight.safety_landing.started': { label: '安全降落已触发', tone: 'warning', category: 'safety' },
  'flight.safety_landing.completed': { label: '安全降落完成', tone: 'success', category: 'safety' },
  'flight.safety_landing.failed': { label: '安全降落失败', tone: 'critical', category: 'safety' },
  'command.rejected': { label: '操作被安全系统拒绝', tone: 'warning', category: 'safety' }
}

const COMMAND_LABELS: Record<string, EventView> = {
  'device.connect': { label: '真机连接完成', tone: 'success', category: 'system' },
  'mission.start': { label: '自动任务已启动', tone: 'success', category: 'flight' },
  'mission.stop': { label: '请求停止并降落', tone: 'warning', category: 'flight' },
  'mission.emergency_stop': { label: '请求任务急停', tone: 'critical', category: 'safety' },
  'mission.control_mode.select': { label: '控制模式选择已发送', tone: 'info', category: 'mode' },
  'mission.pause.toggle': { label: '任务暂停状态已切换', tone: 'info', category: 'mode' },
  'flight.land': { label: '降落指令已完成', tone: 'success', category: 'flight' },
  'flight.emergency_land': { label: '紧急降落指令已发送', tone: 'critical', category: 'safety' },
  'device.stop': { label: '真机连接已关闭', tone: 'info', category: 'system' }
}

export function eventView(type: string, payload: Record<string, unknown> = {}): EventView {
  if (type === 'command.completed' && typeof payload.command === 'string') {
    return COMMAND_LABELS[payload.command] ?? { label: '', tone: 'info', category: 'system' }
  }
  return EVENT_LABELS[type] ?? { label: type, tone: 'info', category: 'system' }
}
