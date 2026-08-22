import type { KeyboardEvent, ReactElement } from 'react'

export type SetupTab = 'profiles' | 'preflight' | 'events'

const TABS: Array<{ key: SetupTab; label: string }> = [
  { key: 'profiles', label: '人物档案' },
  { key: 'preflight', label: '起飞准备' },
  { key: 'events', label: '运行事件' }
]

type Props = {
  active: SetupTab
  onChange: (tab: SetupTab) => void
  profilesPanel: ReactElement
  preflightPanel: ReactElement
  eventsPanel: ReactElement
}

/**
 * Segmented tabs over the ground-setup column. All three panels stay mounted
 * (inactive ones hidden) so an in-progress enrollment survives tab switches.
 */
export function SetupTabs({ active, onChange, profilesPanel, preflightPanel, eventsPanel }: Props): ReactElement {
  const onKeyDown = (event: KeyboardEvent<HTMLButtonElement>): void => {
    const index = TABS.findIndex((tab) => tab.key === active)
    if (event.key === 'ArrowRight') {
      event.preventDefault()
      onChange(TABS[(index + 1) % TABS.length].key)
    } else if (event.key === 'ArrowLeft') {
      event.preventDefault()
      onChange(TABS[(index - 1 + TABS.length) % TABS.length].key)
    }
  }
  return (
    <div className="setup-tabs">
      <div className="setup-tab-list" role="tablist" aria-label="地面准备">
        {TABS.map((tab) => (
          <button
            key={tab.key}
            role="tab"
            id={`setup-tab-${tab.key}`}
            aria-selected={active === tab.key}
            aria-controls={`setup-panel-${tab.key}`}
            tabIndex={active === tab.key ? 0 : -1}
            className={active === tab.key ? 'active' : ''}
            onClick={() => onChange(tab.key)}
            onKeyDown={onKeyDown}
          >
            {tab.label}
          </button>
        ))}
      </div>
      <div className="setup-tab-body">
        <div role="tabpanel" id="setup-panel-profiles" aria-labelledby="setup-tab-profiles" hidden={active !== 'profiles'}>
          {profilesPanel}
        </div>
        <div role="tabpanel" id="setup-panel-preflight" aria-labelledby="setup-tab-preflight" hidden={active !== 'preflight'}>
          {preflightPanel}
        </div>
        <div role="tabpanel" id="setup-panel-events" aria-labelledby="setup-tab-events" hidden={active !== 'events'}>
          {eventsPanel}
        </div>
      </div>
    </div>
  )
}
