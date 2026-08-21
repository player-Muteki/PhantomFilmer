import { defineConfig } from '@playwright/test'

export default defineConfig({
  testDir: './e2e',
  timeout: 30_000,
  workers: 1,
  use: {
    trace: 'retain-on-failure'
  }
})
