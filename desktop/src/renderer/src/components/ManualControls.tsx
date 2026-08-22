import type { ReactElement } from 'react'
import type { Control, PartialRcCommand } from '../app/types'

type Props = {
  speed: number
  onSpeed: (value: number) => void
  activeControl: string | null
  keyboardEnabled: boolean
  onKeyboardToggle: (enabled: boolean) => void
  controlsLocked: boolean
  onStart: (name: string, command: PartialRcCommand) => void
  onStop: () => void
}

const SPEEDS = [15, 20, 30]

export function ManualControls({ speed, onSpeed, activeControl, keyboardEnabled, onKeyboardToggle, controlsLocked, onStart, onStop }: Props): ReactElement {
  return (
    <section className="manual-overlay compact" aria-label="手动控制">
      <div className="manual-heading">
        <div>
          <p className="eyebrow">MANUAL TAKEOVER</p>
          <h2>手动控制</h2>
        </div>
        <div className="manual-tools">
          <label className="keyboard-toggle" title="关闭后键盘不再产生飞行指令，防止误触；屏幕控制板不受影响">
            <input
              type="checkbox"
              checked={keyboardEnabled}
              onChange={(event) => onKeyboardToggle(event.target.checked)}
            />
            键盘控制
          </label>
          <div className="speed-select" aria-label="速度档位">
            {SPEEDS.map((value) => (
              <button key={value} className={speed === value ? 'active' : ''} disabled={controlsLocked} onClick={() => onSpeed(value)}>
                {value}
              </button>
            ))}
          </div>
        </div>
      </div>
      <div className="pads">
        <ControlPad
          title="平面移动"
          active={activeControl}
          enabled={!controlsLocked}
          onStart={onStart}
          onStop={onStop}
          controls={[
            ['forward', 'W', '前进', { forwardBack: speed }, 'up'],
            ['left', 'A', '左移', { leftRight: -speed }, 'left'],
            ['hover-a', 'SPACE', '悬停', {}, 'center'],
            ['right', 'D', '右移', { leftRight: speed }, 'right'],
            ['back', 'S', '后退', { forwardBack: -speed }, 'down']
          ]}
        />
        <ControlPad
          title="高度与偏航"
          active={activeControl}
          enabled={!controlsLocked}
          onStart={onStart}
          onStop={onStop}
          controls={[
            ['up', 'R', '上升', { upDown: speed }, 'up'],
            ['yaw-left', 'J', '左转', { yaw: -speed }, 'left'],
            ['hover-b', 'SPACE', '悬停', {}, 'center'],
            ['yaw-right', 'L', '右转', { yaw: speed }, 'right'],
            ['down', 'F', '下降', { upDown: -speed }, 'down']
          ]}
        />
      </div>
    </section>
  )
}

function ControlPad({ title, controls, active, enabled, onStart, onStop }: {
  title: string
  controls: Control[]
  active: string | null
  enabled: boolean
  onStart: (name: string, command: PartialRcCommand) => void
  onStop: () => void
}): ReactElement {
  return (
    <div className="control-pad">
      <span className="control-title">{title}</span>
      <div className="pad-grid">
        {controls.map(([name, key, label, command, position]) => (
          <button
            key={name}
            className={`control-key ${position} ${active === name ? 'active' : ''}`}
            disabled={!enabled}
            onPointerDown={(event) => {
              event.preventDefault()
              if (name.startsWith('hover')) onStop()
              else onStart(name, command)
            }}
            onPointerUp={() => onStop()}
            onPointerCancel={() => onStop()}
          >
            <kbd>{key}</kbd>
            <span>{label}</span>
          </button>
        ))}
      </div>
    </div>
  )
}
