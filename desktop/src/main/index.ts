import { join } from 'node:path'
import { app, BrowserWindow, dialog, ipcMain, session } from 'electron'
import { is } from '@electron-toolkit/utils'
import type { RcCommand } from '../preload/api'
import { SidecarManager } from './sidecar'

let mainWindow: BrowserWindow | null = null
let allowQuit = false
let closeInProgress = false
let sidecar: SidecarManager

app.commandLine.appendSwitch('disable-features', 'SpellcheckService')
app.commandLine.appendSwitch('disable-component-update')

const hasSingleInstanceLock = app.requestSingleInstanceLock()
if (hasSingleInstanceLock) {
  app.on('second-instance', () => {
    if (!mainWindow) return
    if (mainWindow.isMinimized()) mainWindow.restore()
    mainWindow.focus()
  })
}

function createWindow(): void {
  mainWindow = new BrowserWindow({
    width: 1480,
    height: 980,
    minWidth: 1040,
    minHeight: 760,
    show: false,
    title: 'PhantomFilmer',
    backgroundColor: '#edf3f8',
    icon: join(__dirname, '../../build/icon.png'),
    webPreferences: {
      preload: join(__dirname, '../preload/index.js'),
      contextIsolation: true,
      sandbox: true,
      nodeIntegration: false,
      webSecurity: true,
      devTools: is.dev
    }
  })

  mainWindow.on('ready-to-show', () => mainWindow?.show())
  mainWindow.on('closed', () => {
    mainWindow = null
  })
  mainWindow.webContents.setWindowOpenHandler(() => ({ action: 'deny' }))
  mainWindow.webContents.on('will-navigate', (event) => event.preventDefault())
  mainWindow.webContents.on('will-attach-webview', (event) => event.preventDefault())
  mainWindow.on('close', (event) => {
    if (allowQuit) return
    event.preventDefault()
    if (!closeInProgress) void requestSafeQuit()
  })

  if (is.dev && process.env.ELECTRON_RENDERER_URL) {
    void mainWindow.loadURL(process.env.ELECTRON_RENDERER_URL)
  } else {
    void mainWindow.loadFile(join(__dirname, '../renderer/index.html'))
  }
}

async function requestSafeQuit(): Promise<void> {
  if (!mainWindow || closeInProgress) return
  closeInProgress = true
  const backendState = sidecar.getState()
  if (backendState.airborne) {
    const choice = await dialog.showMessageBox(mainWindow, {
      type: 'warning',
      title: '真机仍在空中',
      message: '退出前必须先悬停并安全降落。',
      detail: 'PhantomFilmer 将发送悬停、降落并关闭视频与 SDK。请持续目视真机，确认落地前不要关闭电脑。',
      buttons: ['安全降落并退出', '取消'],
      defaultId: 1,
      cancelId: 1,
      noLink: true
    })
    if (choice.response !== 0) {
      closeInProgress = false
      return
    }
  }

  try {
    await sidecar.shutdown()
  } catch (error) {
    const risk = await dialog.showMessageBox(mainWindow, {
      type: 'error',
      title: '安全清理未完成',
      message: '无法确认真机连接已安全关闭。',
      detail: `${error instanceof Error ? error.message : '未知错误'}\n\n强制退出可能使真机失去软件控制。只有在已目视确认落地或准备采用实体应急措施时才能继续。`,
      buttons: ['返回控制台', '强制退出'],
      defaultId: 0,
      cancelId: 0,
      noLink: true
    })
    if (risk.response !== 1) {
      closeInProgress = false
      return
    }
    sidecar.forceTerminate()
  }

  allowQuit = true
  app.quit()
}

function registerIpc(): void {
  ipcMain.handle('drone:connect', () => sidecar.connect())
  ipcMain.handle('drone:status', () => sidecar.status())
  ipcMain.handle('drone:takeoff', () => sidecar.takeoff())
  ipcMain.handle('drone:land', () => sidecar.land())
  ipcMain.handle('drone:hover', () => sidecar.hover())
  ipcMain.handle('drone:stop', () => sidecar.stopDrone())
  ipcMain.handle('drone:emergency-land', () => sidecar.emergencyLand())
  ipcMain.handle('drone:video-url', () => sidecar.getVideoUrl())
  ipcMain.handle('backend:state', () => sidecar.getState())
  ipcMain.handle('backend:restart', () => sidecar.restart())
  ipcMain.handle('drone:move-rc', (_event, command: RcCommand) => {
    const channels = ['leftRight', 'forwardBack', 'upDown', 'yaw'] as const
    if (
      !command ||
      channels.some((channel) =>
        typeof command[channel] !== 'number' || !Number.isFinite(command[channel])
      )
    ) {
      throw new Error('手动控制指令格式无效。')
    }
    return sidecar.moveRc(command)
  })
}

if (!hasSingleInstanceLock) {
  app.quit()
} else {
  sidecar = new SidecarManager({
    onStateChange: (state) => {
      const window = mainWindow
      if (closeInProgress || !window || window.isDestroyed() || window.webContents.isDestroyed()) return
      try {
        window.webContents.send('backend:state-changed', state)
      } catch {
        return
      }
    }
  })
  app.whenReady().then(async () => {
    session.defaultSession.setSpellCheckerEnabled(false)
    session.defaultSession.webRequest.onBeforeRequest((details, callback) => {
      const target = new URL(details.url)
      const isNetworkRequest = ['http:', 'https:', 'ws:', 'wss:'].includes(target.protocol)
      const isLoopback = ['127.0.0.1', 'localhost', '::1'].includes(target.hostname)
      callback({ cancel: isNetworkRequest && !isLoopback })
    })
    session.defaultSession.setPermissionRequestHandler((_webContents, _permission, callback) => callback(false))
    session.defaultSession.webRequest.onHeadersReceived((details, callback) => {
      if (details.resourceType !== 'mainFrame') {
        callback({ responseHeaders: details.responseHeaders })
        return
      }
      callback({
        responseHeaders: {
          ...details.responseHeaders,
          'Content-Security-Policy': [
            "default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data: http://127.0.0.1:*; connect-src 'none'; object-src 'none'; frame-src 'none'; base-uri 'none'; form-action 'none'"
          ]
        }
      })
    })
    registerIpc()
    createWindow()
    await sidecar.start().catch(() => undefined)
  })

  app.on('activate', () => {
    if (BrowserWindow.getAllWindows().length === 0) createWindow()
  })

  app.on('window-all-closed', () => {
    if (process.platform !== 'darwin') app.quit()
  })
}
