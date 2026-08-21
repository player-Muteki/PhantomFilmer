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
