export type FlightStateTone = 'normal' | 'caution' | 'critical'

type FlightStateEntry = { label: string; tone: FlightStateTone }

/** Chinese labels for every FollowSession session_state plus server-side states. */
const FLIGHT_STATES: Record<string, FlightStateEntry> = {
  IDLE: { label: '待命', tone: 'normal' },
  GROUND_TARGET_LOCK: { label: '地面目标锁定中', tone: 'normal' },
  GROUND_TAKEOFF_CONFIRMATION: { label: '等待起飞确认', tone: 'normal' },
  TAKEOFF_CANCELLED: { label: '起飞已取消', tone: 'caution' },
  VERIFYING_TAKEOFF_HEIGHT: { label: '校验起飞高度', tone: 'normal' },
  TAKEOFF_HEIGHT_READY: { label: '起飞高度就绪', tone: 'normal' },
  REACHING_BASE_HEIGHT: { label: '正在上升至基础高度', tone: 'normal' },
  BASE_HEIGHT_READY: { label: '已到达悬停高度', tone: 'normal' },
  CONTROL_READY: { label: '已到达悬停高度，请选择跟随模式', tone: 'normal' },
  FOLLOWING: { label: '自动跟随中', tone: 'normal' },
  SIDE_SAMPLING: { label: '侧向采样中', tone: 'normal' },
  FRONT_SAMPLING: { label: '前向采样中', tone: 'normal' },
  MODE_SWITCHING: { label: '模式切换中', tone: 'normal' },
  MANUAL: { label: '手动控制中', tone: 'normal' },
  PAUSED: { label: '任务已暂停', tone: 'caution' },
  STOPPED: { label: '任务已停止', tone: 'normal' },
  EMERGENCY_STOP: { label: '急停', tone: 'critical' },
  BASE_HEIGHT_FAILED: { label: '未能到达基础高度', tone: 'critical' },
  LOW_BATTERY_LANDING: { label: '低电量，降落中', tone: 'critical' },
  HEIGHT_LIMIT_LANDING: { label: '高度超限，降落中', tone: 'critical' },
  TARGET_LOST_LANDING: { label: '目标丢失，降落中', tone: 'critical' },
  FRAME_LOST_LANDING: { label: '图像丢失，降落中', tone: 'critical' },
  BASE_HEIGHT_TIMEOUT_LANDING: { label: '悬停超时，降落中', tone: 'critical' },
  OBSTACLE_FAILSAFE_LANDING: { label: '避障失效，降落中', tone: 'critical' },
  HEIGHT_SENSOR_LANDING: { label: '高度传感器失效，降落中', tone: 'critical' },
  SEARCHING_TARGET: { label: '目标丢失，搜索中', tone: 'caution' }
}

export function flightStateView(state: string | undefined): FlightStateEntry {
  if (!state) return { label: '—', tone: 'normal' }
  return FLIGHT_STATES[state] ?? { label: state, tone: 'normal' }
}
