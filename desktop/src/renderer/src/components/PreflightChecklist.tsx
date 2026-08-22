import type { ReactElement } from 'react'
import type { DroneStatus } from '../../../preload/api'

type Props = {
  preflight: DroneStatus['preflight'] | undefined
  connected: boolean
}

const ITEMS: Array<{ key: keyof NonNullable<DroneStatus['preflight']>; label: string; failureHint: string }> = [
  { key: 'sdk', label: 'SDK 通信', failureHint: '命令链路不可用' },
  { key: 'video', label: '视频流', failureHint: '等待有效视频帧' },
  { key: 'battery', label: '电量', failureHint: '电量不足或读取失败' },
  { key: 'bottomTof', label: '底部 ToF 高度计', failureHint: '高度读数失效' },
  { key: 'frontTof', label: '前向 ToF', failureHint: '前向测距失效' }
]

/** Five-item preflight checklist derived from the server-side readiness gates. */
export function PreflightChecklist({ preflight, connected }: Props): ReactElement {
  return (
    <div className="preflight-checklist" aria-label="起飞预检清单">
      <span className="checklist-title">起飞预检</span>
      <ul>
        {ITEMS.map((item) => {
          const passed = preflight ? preflight[item.key] === true : undefined
          return (
            <li key={item.key} className={passed == null ? 'unknown' : passed ? 'pass' : 'fail'}>
              <i aria-hidden="true">{passed == null ? '—' : passed ? '✓' : '✗'}</i>
              <span>{item.label}</span>
              {connected && passed === false && <small>{item.failureHint}</small>}
            </li>
          )
        })}
      </ul>
    </div>
  )
}
