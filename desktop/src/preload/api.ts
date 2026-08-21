export type FlightPhase = '连接' | '检查' | '起飞' | '手动飞行' | '降落'

export type RcCommand = {
  leftRight: number
  forwardBack: number
  upDown: number
  yaw: number
}

export type DroneStatus = {
  battery?: number
  heightCm?: number
  frontTofCm?: number | null
  frontTofState?: 'clear' | 'blocked' | 'out_of_range' | 'unavailable'
  controlHz?: number
  flightState?: string
  phase?: FlightPhase
  videoReady?: boolean
  airborne?: boolean
  canTakeoff?: boolean
  rcEnabled?: boolean
  preflight?: {
    sdk?: boolean
    video?: boolean
    battery?: boolean
    bottomTof?: boolean
    frontTof?: boolean
  }
}

export type BackendState = {
  status: 'starting' | 'ready' | 'offline' | 'stopping'
  version: string
  logDir: string
  error?: string
  airborne: boolean
  restartAllowed: boolean
}

export type PhantomFilmerApi = {
  connect: () => Promise<DroneStatus>
  status: () => Promise<DroneStatus>
  takeoff: () => Promise<DroneStatus>
  land: () => Promise<DroneStatus>
  hover: () => Promise<DroneStatus>
  moveRc: (command: RcCommand) => Promise<{ ok: boolean; flightState: string }>
  stop: () => Promise<{ ok: boolean }>
  emergencyLand: () => Promise<{ ok: boolean }>
  getVideoUrl: () => Promise<string>
  getBackendState: () => Promise<BackendState>
  restartBackend: () => Promise<BackendState>
  onBackendState: (listener: (state: BackendState) => void) => () => void
}
