import type { ReactElement } from 'react'
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
  children?: ReactElement
}

export function VideoPanel({
  videoUrl, connection, backendReady, connected, awaitingModeSelection, canReconnectVideo, onConnect, onRetryVideo, onVideoError, overlay, children
}: Props): ReactElement {
  return (
    <section className={`video-panel core-video ${videoUrl ? 'streaming' : ''}`}>
      {videoUrl ? (
        <img src={videoUrl} alt="无人机实时视频流" onError={onVideoError} />
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
      {children}
    </section>
  )
}
