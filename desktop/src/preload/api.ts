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
  paused?: boolean
  batteryFresh?: boolean
  heightFresh?: boolean
  safetyReason?: string | null
  preview?: GroundPreviewStatus
  preflight?: {
    sdk?: boolean
    video?: boolean
    battery?: boolean
    bottomTof?: boolean
    frontTof?: boolean
  }
}

export type GroundPreviewStatus = {
  active: boolean
  state: 'idle' | 'preparing' | 'running' | 'error'
  profileName?: string | null
  confirmed: boolean
  stableFrames: number
  requiredStableFrames: number
  found: boolean
  ambiguous: boolean
  similarity?: number | null
  similarityThreshold?: number | null
  candidateCount: number
  areaRatio?: number | null
  orientationDeg?: number | null
  fps: number
  lastFrameAgeMs?: number | null
  error?: string | null
}

export type BackendState = {
  status: 'starting' | 'ready' | 'offline' | 'stopping'
  version: string
  logDir: string
  error?: string
  airborne: boolean
  restartAllowed: boolean
}

export type RuntimeSnapshot = {
  sequence: number
  phase: 'disconnected' | 'connecting' | 'preflight' | 'taking_off' | 'airborne' | 'landing' | 'stopping' | 'error'
  mission: 'idle' | 'manual' | 'follow' | 'reid_follow' | 'fixed_demo' | 'dry_run'
  controlMode: 'none' | 'manual' | 'normal' | 'side' | 'front'
  connected: boolean
  airborne: boolean
  streaming: boolean
  flightState: string
  allowedActions: string[]
  telemetry: Record<string, unknown>
  error?: string | null
}

export type RuntimeEvent = {
  sequence: number
  occurredAt: number
  type: string
  payload: Record<string, unknown>
  snapshot?: RuntimeSnapshot | null
}

export type RuntimeCapabilities = {
  apiVersion: '1'
  commands: string[]
  missions: string[]
  eventReplay: boolean
  rcLease: { required: boolean; ttlMs: number }
  safety?: {
    minTakeoffBattery: number
    lowBatteryLand: number
    maxHeightCm: number
    maxRcSpeed: number
    minDescentHeightCm: number
    maxAscentHeightCm: number
    frontStopDistanceCm: number
    telemetryMaxAgeMs: number
  }
  preview?: {
    requiredForAutomaticMission: boolean
    stableFrames: number
    maxAgeMs: number
  }
  missionReadiness?: {
    available: boolean
    missingAssets: string[]
    profileRequired: boolean
  }
}

export type MissionStartOptions = {
  mission: 'follow' | 'reid_follow' | 'fixed_demo'
  profileName: string
  initialControlMode: 'manual' | 'normal' | 'side' | 'front'
  obstacleEnabled: boolean
}

export type ProfileSummary = {
  name: string
  createdAt?: string | null
  photoCount?: number | null
  embeddingDimension?: number | null
  modelName?: string | null
}

export type RuntimeEventsResponse = {
  apiVersion: '1'
  latestSequence: number
  resetRequired: boolean
  events: RuntimeEvent[]
}

export type PhantomFilmerApi = {
  connect: () => Promise<DroneStatus>
  status: () => Promise<DroneStatus>
  takeoff: () => Promise<DroneStatus>
  land: () => Promise<DroneStatus>
  hover: () => Promise<DroneStatus>
  moveRc: (command: RcCommand) => Promise<{ ok: boolean; flightState: string }>
  inputKey: (key: string) => Promise<{ ok: boolean; operatorSequence: number; key: string }>
  stop: () => Promise<{ ok: boolean }>
  emergencyLand: () => Promise<{ ok: boolean }>
  getVideoUrl: () => Promise<string>
  getBackendState: () => Promise<BackendState>
  restartBackend: () => Promise<BackendState>
  getRuntimeCapabilities: () => Promise<RuntimeCapabilities>
  getRuntimeSnapshot: () => Promise<RuntimeSnapshot>
  getRuntimeEvents: (since: number) => Promise<RuntimeEventsResponse>
  startMission: (options: MissionStartOptions) => Promise<{ ok: boolean; mission: string }>
  stopMission: () => Promise<{ ok: boolean }>
  emergencyStopMission: () => Promise<{ ok: boolean }>
  selectControlMode: (mode: MissionStartOptions['initialControlMode']) => Promise<{ ok: boolean; mode: string }>
  toggleMissionPause: () => Promise<{ ok: boolean }>
  listProfiles: () => Promise<ProfileSummary[]>
  enrollProfile: (name: string, overwrite: boolean) => Promise<ProfileSummary | null>
  startPreview: (profileName: string) => Promise<GroundPreviewStatus & { ok: boolean }>
  stopPreview: () => Promise<GroundPreviewStatus & { ok: boolean }>
  onBackendState: (listener: (state: BackendState) => void) => () => void
}
