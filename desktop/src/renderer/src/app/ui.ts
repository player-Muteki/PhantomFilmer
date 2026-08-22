export type ConnectionState = 'disconnected' | 'connecting' | 'connected' | 'error'
export type ArmedAction = 'start' | 'stop' | 'emergency' | null

export function errorMessage(error: unknown, fallback: string): string {
  return error instanceof Error ? error.message : fallback
}
