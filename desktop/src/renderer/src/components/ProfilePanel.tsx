import type { ReactElement } from 'react'
import type { DroneStatus, RuntimeEvent } from '../../../preload/api'
import type { MissionType } from '../app/types'
import { missionTypeLabel } from '../app/types'
import { EventTimeline } from './EventTimeline'
import { PreflightChecklist } from './PreflightChecklist'
import { SetupTabs, type SetupTab } from './SetupTabs'

type Props = {
  profiles: Array<{ name: string; photoCount?: number | null }>
  profileName: string
  onProfileName: (name: string) => void
  enrollmentName: string
  onEnrollmentName: (name: string) => void
  pendingPhotos: string[] | null
  onPickPhotos: () => void
  onConfirmEnroll: () => void
  onCancelEnroll: () => void
  enrollBusy: boolean
  connected: boolean
  missionRunning: boolean
  actionBusy: boolean
  missionType: MissionType
  onMissionType: (type: MissionType) => void
  obstacleEnabled: boolean
  onObstacleEnabled: (enabled: boolean) => void
  canStartMission: boolean
  launchArmed: boolean
  onLaunch: () => void
  missingAssets: string[]
  preflight: DroneStatus['preflight']
  events: RuntimeEvent[]
  activeTab: SetupTab
  onActiveTab: (tab: SetupTab) => void
}

export function ProfilePanel(props: Props): ReactElement {
  const {
    profiles, profileName, onProfileName, enrollmentName, onEnrollmentName,
    pendingPhotos, onPickPhotos, onConfirmEnroll, onCancelEnroll, enrollBusy,
    connected, missionRunning, actionBusy,
    missionType, onMissionType, obstacleEnabled, onObstacleEnabled,
    canStartMission, launchArmed, onLaunch, missingAssets, preflight,
    events, activeTab, onActiveTab
  } = props
  const enrolling = pendingPhotos != null
  return (
    <aside className="profile-panel" data-mission={missionRunning ? 'on' : 'off'}>
      <h1>任务与人物</h1>
      <SetupTabs
        active={activeTab}
        onChange={onActiveTab}
        profilesPanel={(
          <div className="tab-section">
            <label className="profile-picker">
              <span>当前档案</span>
              <select
                aria-label="人物档案"
                value={profileName}
                onChange={(event) => onProfileName(event.target.value)}
                disabled={missionRunning || actionBusy}
              >
                <option value="">请选择人物档案</option>
                {profiles.map((profile) => (
                  <option key={profile.name} value={profile.name}>{profile.name}</option>
                ))}
              </select>
            </label>
            {enrolling ? (
              <div className="enroll-confirm">
                <p>
                  已为「{enrollmentName}」选择 {pendingPhotos.length} 张参考照片，确认后写入本地档案。
                </p>
                <div className="enroll-confirm-actions">
                  <button onClick={onConfirmEnroll} disabled={enrollBusy}>确认建档</button>
                  <button onClick={onCancelEnroll} disabled={enrollBusy}>取消</button>
                </div>
              </div>
            ) : (
              <div className="profile-create">
                <input
                  aria-label="新人物档案名"
                  value={enrollmentName}
                  onChange={(event) => onEnrollmentName(event.target.value)}
                  placeholder="新档案名"
                  disabled={connected || actionBusy}
                />
                <button disabled={!enrollmentName.trim() || connected || actionBusy} onClick={onPickPhotos}>
                  新建档案
                </button>
              </div>
            )}
            <div className="profile-summary">
              <span>当前档案</span>
              <strong>{profileName || '未选择'}</strong>
              <small>{profiles.find((profile) => profile.name === profileName)?.photoCount ?? 0} 张参考照片</small>
            </div>
          </div>
        )}
        preflightPanel={(
          <div className="tab-section">
            <div className="launch-settings">
              <label className="mission-type">
                <span>任务类型</span>
                <select
                  aria-label="任务类型"
                  value={missionType}
                  onChange={(event) => onMissionType(event.target.value as MissionType)}
                  disabled={missionRunning || actionBusy}
                >
                  <option value="follow">{missionTypeLabel('follow')}</option>
                  <option value="fixed_demo">{missionTypeLabel('fixed_demo')}</option>
                </select>
              </label>
              <label className="obstacle-toggle" title="仅普通跟随模式参与避障；侧向/前向模式设计上不避障">
                <input
                  type="checkbox"
                  checked={obstacleEnabled}
                  onChange={(event) => onObstacleEnabled(event.target.checked)}
                  disabled={missionRunning || actionBusy}
                />
                启用前向 ToF 避障
              </label>
              {obstacleEnabled && (
                <small className="obstacle-hint">避障仅在普通跟随模式生效；侧向/前向跟随不避障。</small>
              )}
            </div>
            <PreflightChecklist preflight={preflight} connected={connected} />
          </div>
        )}
        eventsPanel={<EventTimeline events={events} />}
      />
      <div className={`readiness ${canStartMission ? 'ready' : ''}`}>
        <i>{canStartMission ? '✓' : '—'}</i>
        <span>{canStartMission ? '已满足起飞条件' : connected ? '请完成起飞检查并选择档案' : '请先连接无人机'}</span>
      </div>
      {missingAssets.length > 0 && (
        <p className="runtime-error">缺少运行资产：{missingAssets.join('、')}</p>
      )}
      <button
        className={`launch-button ${launchArmed ? 'armed' : ''}`}
        disabled={!canStartMission || actionBusy}
        onClick={onLaunch}
      >
        {launchArmed ? '确认起飞' : '起飞'}
      </button>
    </aside>
  )
}
