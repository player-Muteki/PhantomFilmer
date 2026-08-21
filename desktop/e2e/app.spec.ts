import { join } from 'node:path'
import { _electron as electron, expect, test, type ElectronApplication } from '@playwright/test'

const appEntry = join(__dirname, '..', 'out', 'main', 'index.js')
const sidecar = join(__dirname, 'fixtures', 'test_sidecar.py')

async function forceClose(application: ElectronApplication): Promise<void> {
  const electronProcess = application.process()
  const exited = new Promise<void>((resolve) => electronProcess.once('exit', () => resolve()))
  electronProcess.kill('SIGKILL')
  await exited
}

test('connects, streams, refreshes RC, lands, and cleans up', async () => {
  const application = await electron.launch({
    args: [appEntry],
    env: { ...process.env, PHANTOMFILMER_SIDECAR_PATH: sidecar }
  })
  const window = await application.firstWindow()
  await expect(window.getByText('后端在线')).toBeVisible()
  await window.getByRole('button', { name: '连接真机' }).click()
  await expect(window.getByText('真机已连接')).toBeVisible()
  await expect(window.getByAltText('无人机实时视频流')).toBeVisible()

  await window.getByRole('button', { name: /^起飞/ }).click()
  await window.getByRole('button', { name: /^确认起飞/ }).click()
  await expect(window.getByText('空中')).toBeVisible()
  await window.keyboard.down('w')
  await window.waitForTimeout(380)
  await window.keyboard.up('w')
  await expect(window.getByText('已悬停')).toBeVisible()

  await window.getByRole('button', { name: /^正常降落/ }).click()
  await window.getByRole('button', { name: /^确认降落/ }).click()
  await expect(window.getByText('降落完成，视频保持连接')).toBeVisible()
  await application.close()
})

test('locks controls and refuses restart after airborne backend crash', async () => {
  const application = await electron.launch({
    args: [appEntry],
    env: {
      ...process.env,
      PHANTOMFILMER_SIDECAR_PATH: sidecar,
      PHANTOMFILMER_TEST_AIRBORNE: '1',
      PHANTOMFILMER_TEST_CRASH_AFTER_CONNECT: '1'
    }
  })
  const window = await application.firstWindow()
  await window.getByRole('button', { name: '连接真机' }).click()
  await expect(window.getByText('后端中断 · 最后状态为空中')).toBeVisible()
  await expect(window.getByRole('button', { name: /禁止自动重启/ })).toBeDisabled()
  await expect(window.getByRole('button', { name: /正常降落/ })).toBeDisabled()
  await forceClose(application)
})

test('treats a backend crash during takeoff as possibly airborne', async () => {
  const application = await electron.launch({
    args: [appEntry],
    env: {
      ...process.env,
      PHANTOMFILMER_SIDECAR_PATH: sidecar,
      PHANTOMFILMER_TEST_CRASH_DURING_TAKEOFF: '1'
    }
  })
  const window = await application.firstWindow()
  await window.getByRole('button', { name: '连接真机' }).click()
  await window.getByRole('button', { name: /^起飞/ }).click()
  await window.getByRole('button', { name: /^确认起飞/ }).click()

  await expect(window.getByText('后端中断 · 最后状态为空中')).toBeVisible()
  await expect(window.getByRole('button', { name: /禁止自动重启/ })).toBeDisabled()
  await forceClose(application)
})

test('previews a profile and starts an automatic mission through every process boundary', async () => {
  const application = await electron.launch({
    args: [appEntry],
    env: { ...process.env, PHANTOMFILMER_SIDECAR_PATH: sidecar }
  })
  const window = await application.firstWindow()
  await window.getByRole('button', { name: '连接真机' }).click()
  await expect(window.getByText('真机已连接')).toBeVisible()
  await window.getByRole('button', { name: '任务与人物' }).click()

  const preview = window.getByRole('region', { name: '地面人物识别预览' })
  await expect(preview.getByRole('button', { name: '启动识别预览' })).toBeEnabled()
  await preview.getByRole('button', { name: '启动识别预览' }).click()
  await expect(window.getByText('目标人物已确认')).toBeVisible()

  const card = window.getByRole('heading', { name: '普通自动跟随' }).locator('..')
  await expect(card.getByRole('button', { name: '启动任务' })).toBeEnabled()
  await card.getByRole('button', { name: '启动任务' }).click()
  await card.getByRole('button', { name: '再次点击确认起飞' }).click()
  await expect(window.getByText(/普通自动跟随 · FOLLOWING/)).toBeVisible()

  await window.getByRole('button', { name: '停止并降落' }).click()
  await window.getByRole('button', { name: '再次确认停止并降落' }).click()
  await expect(window.getByText('任务已停止并降落。')).toBeVisible()
  await application.close()
})
