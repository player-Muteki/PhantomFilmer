import { useEffect, useState, type ReactElement } from 'react'
import type { DroneStatus, ProfileDetails, ProfileSummary, RuntimeEvent } from '../../../preload/api'
import type { MissionType } from '../app/types'
import { missionTypeLabel } from '../app/types'
import { EventTimeline } from './EventTimeline'
import { PreflightChecklist } from './PreflightChecklist'
import { SetupTabs, type SetupTab } from './SetupTabs'

type Props = {
  profiles: ProfileSummary[]
  profileDetails: ProfileDetails | null
  profileName: string
  onProfileName: (name: string) => void
  enrollmentName: string
  onEnrollmentName: (name: string) => void
  pendingPhotos: string[] | null
  onPickPhotos: () => void
  onConfirmEnroll: () => void
  onCancelEnroll: () => void
  onReplaceProfile: () => void
  onRenameProfile: (nextName: string) => void
  onDeleteProfile: () => void
  enrollBusy: boolean
  connected: boolean
  missionRunning: boolean
  actionBusy: boolean
  missionType: MissionType
  onMissionType: (type: MissionType) => void
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
    profiles, profileDetails, profileName, onProfileName, enrollmentName, onEnrollmentName,
    pendingPhotos, onPickPhotos, onConfirmEnroll, onCancelEnroll, enrollBusy,
    onReplaceProfile, onRenameProfile, onDeleteProfile,
    connected, missionRunning, actionBusy,
    missionType, onMissionType,
    canStartMission, launchArmed, onLaunch, missingAssets, preflight,
    events, activeTab, onActiveTab
  } = props
  const enrolling = pendingPhotos != null
  const [renaming, setRenaming] = useState(false)
  const [renameValue, setRenameValue] = useState(profileName)

  useEffect(() => {
    setRenameValue(profileName)
    setRenaming(false)
  }, [profileName])

  const submitRename = (): void => {
    const nextName = renameValue.trim()
    if (!nextName || nextName === profileName) return
    onRenameProfile(nextName)
  }

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
              <div><span>当前档案</span><strong title={profileName}>{profileName || '未选择'}</strong></div>
              <small title={profileDetails?.modelName ? `模型：${profileDetails.modelName}` : undefined}>
                {profiles.find((profile) => profile.name === profileName)?.photoCount ?? 0} 张{profileDetails?.modelName ? ` · ${profileDetails.modelName}` : ''}
              </small>
            </div>
            <div className="profile-actions" aria-label="人物档案操作">
              <button disabled={!profileName || connected || actionBusy} onClick={onReplaceProfile}>更新照片</button>
              <button
                disabled={!profileName || connected || actionBusy}
                onClick={() => {
                  setRenameValue(profileName)
                  setRenaming(true)
                }}
              >重命名</button>
              <button className="danger" disabled={!profileName || connected || actionBusy} onClick={onDeleteProfile}>删除档案</button>
            </div>
            {renaming && (
              <div className="profile-rename" role="group" aria-label="重命名人物档案">
                <input
                  aria-label="新档案名"
                  value={renameValue}
                  onChange={(event) => setRenameValue(event.target.value)}
                  onKeyDown={(event) => {
                    if (event.key === 'Enter') submitRename()
                    if (event.key === 'Escape') setRenaming(false)
                  }}
                  disabled={actionBusy}
                  autoFocus
                />
                <button disabled={!renameValue.trim() || renameValue.trim() === profileName || actionBusy} onClick={submitRename}>
                  确认重命名
                </button>
                <button disabled={actionBusy} onClick={() => setRenaming(false)}>取消</button>
              </div>
            )}
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
              <div className="safety-required" title="该安全保护由后端强制开启，不能在任务界面关闭">
                <i aria-hidden="true">✓</i>
                <span><strong>前向 ToF 安全保护</strong><small>所有模式强制启用 · 普通跟随支持自动绕行</small></span>
              </div>
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
