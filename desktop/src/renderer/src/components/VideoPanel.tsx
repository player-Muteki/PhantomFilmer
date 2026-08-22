import { useLayoutEffect, useRef, useState, type CSSProperties, type ReactElement, type SyntheticEvent } from 'react'
import { Icon } from '../Icons'
import type { ConnectionState } from '../app/ui'

type Props = {
  videoUrl: string | null
  connection: ConnectionState
  backendReady: boolean
  connected: boolean
  awaitingModeSelection: boolean
  canReconnectVideo: boolean
  onConnect: () => void
  onRetryVideo: () => void
  onVideoError: () => void
  overlay?: ReactElement | null
}

export function VideoPanel({
  videoUrl, connection, backendReady, connected, awaitingModeSelection, canReconnectVideo, onConnect, onRetryVideo, onVideoError, overlay
}: Props): ReactElement {
  const viewportRef = useRef<HTMLDivElement>(null)
  const [aspectRatio, setAspectRatio] = useState(4 / 3)
  const [frameSize, setFrameSize] = useState<{ width: number; height: number } | null>(null)

  useLayoutEffect(() => {
    const viewport = viewportRef.current
    if (!viewport) return
    const update = (): void => {
      const { width, height } = viewport.getBoundingClientRect()
      if (width <= 0 || height <= 0) return
      const frameWidth = Math.min(width, height * aspectRatio)
      setFrameSize({ width: Math.floor(frameWidth), height: Math.floor(frameWidth / aspectRatio) })
    }
    update()
    if (typeof ResizeObserver === 'undefined') return
    const observer = new ResizeObserver(update)
    observer.observe(viewport)
    return () => observer.disconnect()
  }, [aspectRatio])

  const onVideoLoad = (event: SyntheticEvent<HTMLImageElement>): void => {
    const image = event.currentTarget
    if (image.naturalWidth > 0 && image.naturalHeight > 0) {
      const ratio = image.naturalWidth / image.naturalHeight
      if (Number.isFinite(ratio) && ratio >= 0.5 && ratio <= 3) setAspectRatio(ratio)
    }
  }

  const frameStyle: CSSProperties = frameSize
    ? { width: `${frameSize.width}px`, height: `${frameSize.height}px`, aspectRatio: String(aspectRatio) }
    : { width: '100%', aspectRatio: String(aspectRatio) }
  return (
    <div className={`video-viewport ${videoUrl ? 'streaming' : ''}`} ref={viewportRef}>
    <section className={`video-panel core-video ${videoUrl ? 'streaming' : ''}`} style={frameStyle} data-aspect-ratio={aspectRatio.toFixed(6)}>
      {videoUrl ? (
        <img src={videoUrl} alt="无人机实时视频流" onLoad={onVideoLoad} onError={onVideoError} />
      ) : (
        <div className="video-empty">
          <div className="reticle" aria-hidden="true">
            <span /><span /><span /><span /><i />
          </div>
          <h2>{connection === 'connecting' ? '正在建立链路' : backendReady ? (canReconnectVideo ? '视频会话中断' : '等待真机视频') : '后端离线'}</h2>
          <p>连接无人机后即可显示实时视频与人物识别框。</p>
          {canReconnectVideo ? (
            <button className="button primary" onClick={onRetryVideo}>
              <Icon name="link" />重建视频会话
            </button>
          ) : (
            <button
              className="button primary"
              disabled={!backendReady || connection === 'connecting' || connected}
              onClick={onConnect}
            >
              <Icon name="link" />
              {connection === 'connecting' ? '连接中…' : connection === 'error' ? '重新连接真机' : '连接真机'}
            </button>
          )}
        </div>
      )}
      {videoUrl && awaitingModeSelection && (
        <div className="control-ready-banner" role="status">
          已到达悬停高度，请选择跟随模式（普通 / 侧向 / 前向 / 手动）
        </div>
      )}
      {overlay}
    </section>
    </div>
  )
}
