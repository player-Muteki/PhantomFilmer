import type { RcCommand } from '../../../preload/api'

export type MissionMode = 'manual' | 'normal' | 'side' | 'front'
export type PartialRcCommand = Partial<RcCommand>
export type Control = [string, string, string, PartialRcCommand, 'up' | 'left' | 'center' | 'right' | 'down']

export const emptyCommand: RcCommand = { leftRight: 0, forwardBack: 0, upDown: 0, yaw: 0 }

/** Operator keys forwarded to the sidecar's semantic input channel. */
export const missionKey: Record<MissionMode, string> = { manual: 'm', normal: '1', side: '2', front: '3' }

export function missionModeLabel(mode: string): string {
  return ({ manual: '手动接管', normal: '普通跟随', side: '侧向跟随', front: '前向跟随' } as Record<string, string>)[mode] ?? mode
}
