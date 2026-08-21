import { spawn, type ChildProcessByStdio } from 'node:child_process'
import { createWriteStream, mkdirSync } from 'node:fs'
import { dirname, extname, join, resolve } from 'node:path'
import { randomBytes } from 'node:crypto'
import type { Readable } from 'node:stream'
import { app } from 'electron'
import type { BackendState, DroneStatus, RcCommand } from '../preload/api'

type ReadyMessage = {
  event: 'ready'
  host: string
  port: number
  pid: number
  logPath: string
}

type SidecarManagerOptions = {
  onStateChange: (state: BackendState) => void
}

type V1CommandEnvelope<Result> = {
  apiVersion: '1'
  commandId: string
  result: Result
  snapshot: unknown
}

type V1RcLease = {
  apiVersion: '1'
  leaseId: string
  lastSequence: number
  expiresAt: number
}

type ActiveRcLease = {
  leaseId: string
  sequence: number
  expiresAt: number
}

const START_TIMEOUT_MS = 15_000
const STOP_TIMEOUT_MS = 12_000

export class SidecarManager {
  private process: ChildProcessByStdio<null, Readable, Readable> | null = null
  private baseUrl: string | null = null
  private sessionToken: string | null = null
  private expectedExit = false
  private lastStatus: DroneStatus = {}
  private rcLease: ActiveRcLease | null = null
  private state: BackendState
  private readonly logDir: string

  constructor(private readonly options: SidecarManagerOptions) {
    this.logDir = join(app.getPath('userData'), 'logs')
    mkdirSync(this.logDir, { recursive: true })
    this.state = {
      status: 'offline',
      version: app.getVersion(),
      logDir: this.logDir,
      airborne: false,
      restartAllowed: true
    }
  }

  getState(): BackendState {
    return { ...this.state }
  }

  async start(): Promise<BackendState> {
    if (this.process && this.state.status === 'ready') return this.getState()
    if (this.state.airborne) {
      throw new Error('上次后端中断时真机仍在空中，已禁止自动重启。请先按紧急处置指引确认真机状态。')
    }

    this.expectedExit = false
    this.baseUrl = null
    this.rcLease = null
    this.sessionToken = randomBytes(32).toString('base64url')
    this.updateState({ status: 'starting', error: undefined, restartAllowed: false })

    const launch = this.resolveLaunchCommand()
    const args = [
      ...launch.prefixArgs,
      '--host',
      '127.0.0.1',
      '--port',
      '0',
      '--token',
      this.sessionToken,
      '--data-dir',
      app.getPath('userData')
    ]
    const child = spawn(launch.command, args, {
      cwd: launch.cwd,
      env: this.sidecarEnvironment(),
      stdio: ['ignore', 'pipe', 'pipe'],
      windowsHide: true
    })
    this.process = child

    const processLog = createWriteStream(join(this.logDir, 'sidecar-process.log'), { flags: 'a' })
    child.stderr.pipe(processLog, { end: false })
    child.once('exit', (code, signal) => {
      processLog.write(`\n[desktop] sidecar exited code=${String(code)} signal=${String(signal)}\n`)
      processLog.end()
      if (this.process === child) {
        this.process = null
        this.baseUrl = null
      }
      if (!this.expectedExit) {
        const airborne = this.lastStatus.airborne === true
        this.updateState({
          status: 'offline',
          error: airborne
            ? '后端意外退出，最后一次遥测显示真机在空中。请勿重启后端，立即目视确认并准备使用实体应急措施。'
            : `后端意外退出（${code ?? signal ?? '未知原因'}）。`,
          airborne,
          restartAllowed: !airborne
        })
      }
    })

    try {
      const ready = await this.waitForReady(child, processLog)
      this.baseUrl = `http://${ready.host}:${ready.port}`
      await this.request<{ apiVersion: '1'; ok: boolean }>('GET', '/api/v1/health')
      this.updateState({ status: 'ready', error: undefined, restartAllowed: true })
      return this.getState()
    } catch (error) {
      this.expectedExit = true
      child.kill()
      const message = error instanceof Error ? error.message : '后端启动失败。'
      this.updateState({ status: 'offline', error: message, restartAllowed: true })
      throw new Error(message)
    }
  }

  async restart(): Promise<BackendState> {
    if (!this.state.restartAllowed || this.state.airborne) {
      throw new Error('当前状态禁止重启后端。请先确认真机已安全落地。')
    }
    await this.shutdown().catch(() => undefined)
    this.lastStatus = {}
    this.updateState({ airborne: false, restartAllowed: true })
    return this.start()
  }

  async connect(): Promise<DroneStatus> {
    return this.commandStatusRequest('device.connect')
  }

  async status(): Promise<DroneStatus> {
    return this.commandStatusRequest('device.status.refresh')
  }

  async takeoff(): Promise<DroneStatus> {
    // Once the request leaves Electron, a transport failure cannot prove that
    // the aircraft stayed on the ground. Keep the last-known state conservative
    // until a successful landing/stop response explicitly clears it.
    this.lastStatus = { ...this.lastStatus, airborne: true }
    this.updateState({ airborne: true, restartAllowed: false, error: undefined })
    try {
      return await this.commandStatusRequest('flight.takeoff')
    } catch (error) {
      const detail = error instanceof Error ? error.message : '未知错误'
      const message = `起飞请求结果未知，真机可能已在空中。请目视确认并优先执行降落；禁止重启后端。${detail}`
      this.updateState({ airborne: true, restartAllowed: false, error: message })
      throw new Error(message)
    }
  }

  async land(): Promise<DroneStatus> {
    await this.releaseRcLease().catch(() => undefined)
    return this.commandStatusRequest('flight.land')
  }

  async hover(): Promise<DroneStatus> {
    await this.releaseRcLease().catch(() => undefined)
    return this.commandStatusRequest('flight.hover')
  }

  async moveRc(command: RcCommand): Promise<{ ok: boolean; flightState: string }> {
    const lease = await this.ensureRcLease()
    const sequence = lease.sequence + 1
    let result: {
      apiVersion: '1'
      ok: boolean
      flightState: string
      leaseId: string
      lastSequence: number
      expiresAt: number
    }
    try {
      result = await this.request('POST', '/api/v1/rc', {
        leaseId: lease.leaseId,
        sequence,
        issuedAt: Date.now(),
        ...command
      })
    } catch (error) {
      this.rcLease = null
      throw error
    }
    this.rcLease = {
      leaseId: result.leaseId,
      sequence: result.lastSequence,
      expiresAt: result.expiresAt
    }
    return { ok: result.ok, flightState: result.flightState }
  }

  async stopDrone(): Promise<{ ok: boolean }> {
    await this.releaseRcLease().catch(() => undefined)
    const result = await this.commandRequest<{ ok: boolean }>('device.stop')
    this.rememberStatus({})
    return result
  }

  async emergencyLand(): Promise<{ ok: boolean }> {
    await this.releaseRcLease().catch(() => undefined)
    const result = await this.commandRequest<{ ok: boolean }>('flight.emergency_land')
    this.rememberStatus({})
    return result
  }

  async getVideoUrl(): Promise<string> {
    const result = await this.request<{ token: string }>('POST', '/api/drone/video-token')
    return `${this.requireBaseUrl()}/api/drone/video/stream?token=${encodeURIComponent(result.token)}`
  }

  async shutdown(timeoutMs = STOP_TIMEOUT_MS): Promise<void> {
    const child = this.process
    if (!child) return
    this.expectedExit = true
    this.updateState({ status: 'stopping', restartAllowed: false })
    let shutdownError: unknown
    try {
      await this.request('POST', '/api/sidecar/shutdown')
    } catch (error) {
      shutdownError = error
    }
    try {
      await this.waitForExit(child, timeoutMs)
    } catch (error) {
      throw shutdownError ?? error
    }
    this.process = null
    this.baseUrl = null
    this.rcLease = null
    if (shutdownError) {
      const airborne = this.lastStatus.airborne === true
      this.updateState({
        status: 'offline',
        airborne,
        restartAllowed: !airborne,
        error: airborne
          ? '安全退出期间降落或清理失败，最后一次遥测仍为空中。'
          : '后端已退出，但安全清理返回错误。'
      })
      throw shutdownError
    }
    this.lastStatus = {}
    this.updateState({ status: 'offline', airborne: false, restartAllowed: true, error: undefined })
  }

  forceTerminate(): void {
    this.expectedExit = true
    this.process?.kill('SIGKILL')
    this.process = null
    this.baseUrl = null
    this.rcLease = null
  }

  private async commandStatusRequest(type: string): Promise<DroneStatus> {
    const status = await this.commandRequest<DroneStatus>(type)
    this.rememberStatus(status)
    return status
  }

  private async commandRequest<Result>(type: string): Promise<Result> {
    const commandId = randomBytes(16).toString('hex')
    const envelope = await this.request<V1CommandEnvelope<Result>>('POST', '/api/v1/commands', {
      type,
      commandId,
      issuedAt: Date.now()
    })
    if (envelope.apiVersion !== '1' || envelope.commandId !== commandId) {
      throw new Error('本地后端返回了不兼容的命令响应。')
    }
    return envelope.result
  }

  private async ensureRcLease(): Promise<ActiveRcLease> {
    const current = this.rcLease
    if (current && current.expiresAt > Date.now() + 100) return current
    const lease = await this.request<V1RcLease>('POST', '/api/v1/rc/lease')
    if (lease.apiVersion !== '1' || !lease.leaseId) {
      throw new Error('本地后端未返回有效的 RC 控制租约。')
    }
    this.rcLease = {
      leaseId: lease.leaseId,
      sequence: lease.lastSequence,
      expiresAt: lease.expiresAt
    }
    return this.rcLease
  }

  private async releaseRcLease(): Promise<void> {
    const lease = this.rcLease
    this.rcLease = null
    if (!lease) return
    await this.request<{ apiVersion: '1'; ok: boolean }>('POST', '/api/v1/rc/release', {
      leaseId: lease.leaseId
    })
  }

  private rememberStatus(status: DroneStatus): void {
    this.lastStatus = status
    const airborne = status.airborne === true
    this.updateState({ airborne, restartAllowed: this.state.status === 'ready' || !airborne })
  }

  private updateState(changes: Partial<BackendState>): void {
    this.state = { ...this.state, ...changes }
    this.options.onStateChange(this.getState())
  }

  private async request<Result>(method: 'GET' | 'POST', path: string, body?: unknown): Promise<Result> {
    const token = this.sessionToken
    if (!token) throw new Error('后端会话尚未启动。')
    let response: Response
    try {
      response = await fetch(`${this.requireBaseUrl()}${path}`, {
        method,
        headers: {
          'X-Phantom-Token': token,
          ...(body === undefined ? {} : { 'Content-Type': 'application/json' })
        },
        body: body === undefined ? undefined : JSON.stringify(body),
        signal: AbortSignal.timeout(path === '/api/sidecar/shutdown' ? STOP_TIMEOUT_MS : START_TIMEOUT_MS)
      })
    } catch (error) {
      throw new Error(`无法连接本地后端：${error instanceof Error ? error.message : '未知错误'}`)
    }
    if (!response.ok) {
      const payload = (await response.json().catch(() => ({}))) as {
        error?: string | { message?: string }
      }
      const message = typeof payload.error === 'string' ? payload.error : payload.error?.message
      throw new Error(message || `后端请求失败（HTTP ${response.status}）。`)
    }
    return (await response.json()) as Result
  }

  private requireBaseUrl(): string {
    if (!this.baseUrl) throw new Error('本地后端尚未就绪。')
    return this.baseUrl
  }

  private waitForReady(
    child: ChildProcessByStdio<null, Readable, Readable>,
    processLog: NodeJS.WritableStream
  ): Promise<ReadyMessage> {
    return new Promise((resolveReady, rejectReady) => {
      let buffer = ''
      const timeout = setTimeout(() => rejectReady(new Error('等待后端就绪超时。')), START_TIMEOUT_MS)

      const fail = (error: Error): void => {
        clearTimeout(timeout)
        rejectReady(error)
      }
      child.once('error', fail)
      child.once('exit', (code) => fail(new Error(`后端在就绪前退出（${String(code)}）。`)))
      child.stdout.on('data', (chunk: Buffer) => {
        processLog.write(chunk)
        buffer += chunk.toString('utf8')
        const lines = buffer.split(/\r?\n/)
        buffer = lines.pop() ?? ''
        for (const line of lines) {
          try {
            const candidate = JSON.parse(line) as ReadyMessage
            if (candidate.event === 'ready' && candidate.host === '127.0.0.1' && candidate.port > 0) {
              clearTimeout(timeout)
              resolveReady(candidate)
              return
            }
          } catch {
            // Non-JSON output is retained in the process log and ignored here.
          }
        }
      })
    })
  }

  private waitForExit(child: ChildProcessByStdio<null, Readable, Readable>, timeoutMs: number): Promise<void> {
    if (child.exitCode != null) return Promise.resolve()
    return new Promise((resolveExit, rejectExit) => {
      const timeout = setTimeout(
        () => rejectExit(new Error('后端清理超时，尚未确认视频、SDK 与飞行线程已停止。')),
        timeoutMs
      )
      child.once('exit', () => {
        clearTimeout(timeout)
        resolveExit()
      })
    })
  }

  private resolveLaunchCommand(): { command: string; prefixArgs: string[]; cwd: string } {
    const injectedPath = process.env.PHANTOMFILMER_SIDECAR_PATH
    if (injectedPath) {
      const resolvedPath = resolve(injectedPath)
      if (extname(resolvedPath) === '.py') {
        return {
          command: process.platform === 'win32' ? 'python' : 'python3',
          prefixArgs: [resolvedPath],
          cwd: dirname(resolvedPath)
        }
      }
      return { command: resolvedPath, prefixArgs: [], cwd: dirname(resolvedPath) }
    }
    if (app.isPackaged) {
      const executableName = process.platform === 'win32' ? 'phantomfilmer-sidecar.exe' : 'phantomfilmer-sidecar'
      const command = join(process.resourcesPath, 'sidecar', executableName)
      return { command, prefixArgs: [], cwd: dirname(command) }
    }
    const repositoryRoot = resolve(app.getAppPath(), '..')
    return {
      command: process.platform === 'win32' ? 'python' : 'python3',
      prefixArgs: ['-m', 'web_api.server'],
      cwd: repositoryRoot
    }
  }

  private sidecarEnvironment(): NodeJS.ProcessEnv {
    const environment: NodeJS.ProcessEnv = {
      PATH: process.env.PATH,
      PYTHONIOENCODING: 'utf-8',
      PYTHONUNBUFFERED: '1'
    }
    if (process.platform === 'win32') environment.SYSTEMROOT = process.env.SYSTEMROOT
    if (!app.isPackaged && process.env.PHANTOMFILMER_SIDECAR_PATH) {
      for (const [key, value] of Object.entries(process.env)) {
        if (key.startsWith('PHANTOMFILMER_TEST_')) environment[key] = value
      }
    }
    return environment
  }
}
