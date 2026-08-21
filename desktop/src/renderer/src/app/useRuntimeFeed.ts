import { useEffect, useRef, useState } from 'react'
import type {
  RuntimeCapabilities,
  RuntimeEvent,
  RuntimeSnapshot
} from '../../../preload/api'

export type RuntimeFeed = {
  capabilities: RuntimeCapabilities | null
  snapshot: RuntimeSnapshot | null
  events: RuntimeEvent[]
  error: string | null
}

const initialFeed: RuntimeFeed = {
  capabilities: null,
  snapshot: null,
  events: [],
  error: null
}

export function useRuntimeFeed(enabled: boolean): RuntimeFeed {
  const [feed, setFeed] = useState<RuntimeFeed>(initialFeed)
  const cursor = useRef(0)

  useEffect(() => {
    if (!enabled) {
      cursor.current = 0
      setFeed(initialFeed)
      return
    }
    let cancelled = false

    const initialize = async (): Promise<void> => {
      try {
        const [capabilities, snapshot] = await Promise.all([
          window.phantomFilmer.getRuntimeCapabilities(),
          window.phantomFilmer.getRuntimeSnapshot()
        ])
        if (cancelled) return
        cursor.current = snapshot.sequence
        setFeed({ capabilities, snapshot, events: [], error: null })
      } catch (error) {
        if (!cancelled) {
          setFeed((current) => ({
            ...current,
            error: error instanceof Error ? error.message : '无法读取运行时契约'
          }))
        }
      }
    }

    const poll = async (): Promise<void> => {
      try {
        const response = await window.phantomFilmer.getRuntimeEvents(cursor.current)
        if (cancelled) return
        let snapshot: RuntimeSnapshot | null | undefined
        if (response.resetRequired) {
          snapshot = await window.phantomFilmer.getRuntimeSnapshot()
          if (cancelled) return
        } else {
          snapshot = [...response.events].reverse().find((event) => event.snapshot)?.snapshot
        }
        cursor.current = response.latestSequence
        setFeed((current) => ({
          ...current,
          snapshot: snapshot ?? current.snapshot,
          events: [...current.events, ...response.events].slice(-50),
          error: null
        }))
      } catch (error) {
        if (!cancelled) {
          setFeed((current) => ({
            ...current,
            error: error instanceof Error ? error.message : '运行时事件同步失败'
          }))
        }
      }
    }

    void initialize()
    const timer = window.setInterval(() => void poll(), 750)
    return () => {
      cancelled = true
      window.clearInterval(timer)
    }
  }, [enabled])

  return feed
}
