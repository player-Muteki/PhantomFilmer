import type { PhantomFilmerApi } from '../../preload/api'

declare global {
  interface Window {
    phantomFilmer: PhantomFilmerApi
  }
}

export {}
