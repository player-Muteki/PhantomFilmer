import { join } from 'node:path'
import { _electron as electron, expect, test, type ElectronApplication } from '@playwright/test'

const appEntry = join(__dirname, '..', 'out', 'main', 'index.js')
const sidecar = join(__dirname, 'fixtures', 'test_sidecar.py')
const { ELECTRON_RUN_AS_NODE: _ignoredElectronRunAsNode, ...electronEnv } = process.env

function launchTestApp(extraEnv: NodeJS.ProcessEnv = {}): Promise<ElectronApplication> {
  // Some shells set this while invoking Electron tooling. The actual desktop
  // process must never inherit it: Electron would start as Node.js and reject
  // Playwright's debugging arguments before the app can load.
  return electron.launch({
    args: [appEntry],
    env: { ...electronEnv, PHANTOMFILMER_SIDECAR_PATH: sidecar, ...extraEnv }
  })
}

async function forceClose(application: ElectronApplication): Promise<void> {
  const electronProcess = application.process()
  const exited = new Promise<void>((resolve) => electronProcess.once('exit', () => resolve()))
  electronProcess.kill('SIGKILL')
  await exited
}

test('connects, streams, refreshes RC, lands, and cleans up', async () => {
  const application = await launchTestApp()
  const window = await application.firstWindow()
  await expect(window.getByText('后端在线')).toBeVisible()
  await window.getByRole('button', { name: '连接真机' }).click()
  await expect(window.getByText('真机已连接')).toBeVisible()
  await expect(window.getByAltText('无人机实时视频流')).toBeVisible()

  await window.getByRole('button', { name: /^起飞/ }).click()
  await window.getByRole('button', { name: /^确认起飞/ }).click()
  await expect(window.getByText('正在起飞并上升至 150 cm；到达后请选择跟随模式。')).toBeVisible()
  await expect(window.getByRole('button', { name: '普通跟随' })).toBeEnabled()

  await window.getByRole('button', { name: '停止并降落' }).click()
  await window.getByRole('button', { name: '确认停止并降落' }).click()
  await expect(window.getByText('任务已停止并降落。')).toBeVisible()
  await application.close()
})

test('locks controls and refuses restart after airborne backend crash', async () => {
  const application = await launchTestApp({
    PHANTOMFILMER_TEST_AIRBORNE: '1',
    PHANTOMFILMER_TEST_CRASH_AFTER_CONNECT: '1'
  })
  const window = await application.firstWindow()
  await window.getByRole('button', { name: '连接真机' }).click()
  await expect(window.getByText('后端中断 · 最后状态为空中')).toBeVisible()
  await expect(window.getByRole('button', { name: /禁止自动重启/ })).toBeDisabled()
  await expect(window.getByRole('button', { name: '停止并降落' })).toBeDisabled()
  await forceClose(application)
})

test('treats a backend crash during takeoff as possibly airborne', async () => {
  const application = await launchTestApp({
    PHANTOMFILMER_TEST_CRASH_DURING_TAKEOFF: '1'
  })
  const window = await application.firstWindow()
  await window.getByRole('button', { name: '连接真机' }).click()
  await window.getByRole('button', { name: /^起飞/ }).click()
  await window.getByRole('button', { name: /^确认起飞/ }).click()

  await expect(window.getByText('后端中断 · 最后状态为空中')).toBeVisible()
  await expect(window.getByRole('button', { name: /禁止自动重启/ })).toBeDisabled()
  await forceClose(application)
})

test('starts without ground preview and selects an automatic mode through every process boundary', async () => {
  const application = await launchTestApp()
  const window = await application.firstWindow()
  await window.getByRole('button', { name: '连接真机' }).click()
  await expect(window.getByText('真机已连接')).toBeVisible()
  await expect(window.getByRole('combobox', { name: '人物档案' })).toHaveValue('operator-a')
  await window.getByRole('button', { name: '起飞' }).click()
  await window.getByRole('button', { name: '确认起飞' }).click()
  await expect(window.getByRole('button', { name: '普通跟随' })).toBeEnabled()
  await window.getByRole('button', { name: '普通跟随' }).click()
  await expect(window.getByText('正在切换到普通跟随…')).toBeVisible()

  await window.getByRole('button', { name: '停止并降落' }).click()
  await window.getByRole('button', { name: '确认停止并降落' }).click()
  await expect(window.getByText('任务已停止并降落。')).toBeVisible()
  await application.close()
})
