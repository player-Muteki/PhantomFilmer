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
  stop: () => invoke('drone:stop'),
  emergencyLand: () => invoke('drone:emergency-land'),
  getVideoUrl: () => invoke('drone:video-url'),
  getBackendState: () => invoke('backend:state'),
  restartBackend: () => invoke('backend:restart'),
  onBackendState: (listener) => {
    const handler = (_event: Electron.IpcRendererEvent, state: BackendState): void => listener(state)
    ipcRenderer.on('backend:state-changed', handler)
    return () => ipcRenderer.removeListener('backend:state-changed', handler)
  }
}

contextBridge.exposeInMainWorld('phantomFilmer', api)
