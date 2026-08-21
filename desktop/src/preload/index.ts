import { contextBridge, ipcRenderer } from 'electron'
import type { BackendState, PhantomFilmerApi, RcCommand } from './api'

const invoke = <Result>(channel: string, payload?: unknown): Promise<Result> =>
  ipcRenderer.invoke(channel, payload) as Promise<Result>

const api: PhantomFilmerApi = {
  connect: () => invoke('drone:connect'),
  status: () => invoke('drone:status'),
  takeoff: () => invoke('drone:takeoff'),
  land: () => invoke('drone:land'),
  hover: () => invoke('drone:hover'),
  moveRc: (command: RcCommand) => invoke('drone:move-rc', command),
  inputKey: (key: string) => invoke('drone:input-key', key),
  stop: () => invoke('drone:stop'),
  emergencyLand: () => invoke('drone:emergency-land'),
  getVideoUrl: () => invoke('drone:video-url'),
  getBackendState: () => invoke('backend:state'),
  restartBackend: () => invoke('backend:restart'),
  getRuntimeCapabilities: () => invoke('runtime:capabilities'),
  getRuntimeSnapshot: () => invoke('runtime:snapshot'),
  getRuntimeEvents: (since: number) => invoke('runtime:events', since),
  startMission: (options) => invoke('mission:start', options),
  stopMission: () => invoke('mission:stop'),
  emergencyStopMission: () => invoke('mission:emergency-stop'),
  selectControlMode: (mode) => invoke('mission:control-mode', mode),
  toggleMissionPause: () => invoke('mission:pause-toggle'),
  listProfiles: () => invoke('profiles:list'),
  enrollProfile: (name, overwrite) => invoke('profiles:enroll', { name, overwrite }),
  startPreview: (profileName) => invoke('preview:start', profileName),
  stopPreview: () => invoke('preview:stop'),
  onBackendState: (listener) => {
    const handler = (_event: Electron.IpcRendererEvent, state: BackendState): void => listener(state)
    ipcRenderer.on('backend:state-changed', handler)
    return () => ipcRenderer.removeListener('backend:state-changed', handler)
  }
}

contextBridge.exposeInMainWorld('phantomFilmer', api)
