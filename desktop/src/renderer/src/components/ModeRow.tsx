import type { ReactElement } from 'react'
import type { MissionMode } from '../app/types'
import { missionModeLabel } from '../app/types'

type Props = {
  controlMode: string | undefined
  missionRunning: boolean
  paused: boolean
  awaitingModeSelection: boolean
  controlsLocked: boolean
  canSelectMode: boolean
  canTogglePause: boolean
  onMode: (mode: MissionMode) => void
  onPause: () => void
}

const MODES: MissionMode[] = ['normal', 'side', 'front', 'manual']

export function ModeRow({
  controlMode, missionRunning, paused, awaitingModeSelection, controlsLocked, canSelectMode, canTogglePause, onMode, onPause
}: Props): ReactElement {
  return (
    <div className="mode-row" aria-label="自动任务空中控制">
      {MODES.map((mode) => (
        <button
          key={mode}
          className={controlMode === mode ? 'active' : ''}
          disabled={
            !missionRunning ||
            controlsLocked ||
            paused ||
            !canSelectMode ||
            (!awaitingModeSelection && mode !== 'manual' && controlMode === mode)
          }
          onClick={() => onMode(mode)}
        >
          {mode === 'manual' && controlMode === 'manual' ? '退出手动并恢复自动' : missionModeLabel(mode)}
        </button>
      ))}
      <button
        className={`pause-button ${paused ? 'active' : ''}`}
        disabled={!missionRunning || controlsLocked || !canTogglePause}
        onClick={onPause}
        title="暂停后任务悬停冻结，再按一次继续（键盘 P）"
      >
        {paused ? '继续任务' : '暂停任务'}
      </button>
    </div>
  )
}
