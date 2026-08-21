'use client';

import { useEffect, useMemo, useState } from 'react';

type ConnectionState = 'disconnected' | 'connecting' | 'connected' | 'error';
type FlightPhase = '连接' | '起飞' | '爬升' | '跟随' | '搜索' | '降落';

type DroneStatus = {
  battery?: number;
  heightCm?: number;
  frontTofCm?: number | null;
  controlHz?: number;
  flightState?: string;
  targetConfirmed?: boolean;
  phase?: FlightPhase;
  videoReady?: boolean;
};

const phases: FlightPhase[] = ['连接', '起飞', '爬升', '跟随', '搜索', '降落'];
const statusCards = [
  { key: 'height', label: '高度', unit: 'cm', icon: '↕' },
  { key: 'tof', label: '前向 ToF', unit: 'cm', icon: '◎' },
  { key: 'rate', label: '视频率', unit: 'Hz', icon: '∿' },
] as const;

async function responseError(response: Response, fallback: string) {
  try {
    const body = (await response.json()) as { error?: string };
    return body.error || fallback;
  } catch {
    return fallback;
  }
}

export default function Home() {
  const [connection, setConnection] = useState<ConnectionState>('disconnected');
  const [status, setStatus] = useState<DroneStatus>({});
  const [notice, setNotice] = useState('连接真机后才能开启视频流');
  const [emergencyArmed, setEmergencyArmed] = useState(false);

  const connected = connection === 'connected';
  const videoReady = connected && status.videoReady === true;
  const currentPhase = status.phase ?? '连接';
  const currentPhaseIndex = connected ? phases.indexOf(currentPhase) : -1;
  const batteryLabel = useMemo(
    () => (!connected || status.battery == null ? '--' : `${status.battery}%`),
    [connected, status.battery],
  );

  useEffect(() => {
    if (!emergencyArmed) return;
    const timer = window.setTimeout(() => setEmergencyArmed(false), 4000);
    return () => window.clearTimeout(timer);
  }, [emergencyArmed]);

  useEffect(() => {
    if (!connected) return;
    let cancelled = false;
    const refresh = async () => {
      try {
        const response = await fetch('/api/drone/status', { cache: 'no-store' });
        if (!response.ok) throw new Error(await responseError(response, '遥测不可用'));
        const data = (await response.json()) as DroneStatus;
        if (!cancelled) {
          setStatus((previous) => ({ ...previous, ...data }));
          setNotice(data.videoReady ? '真机与视频流均已就绪' : '真机已连接，正在等待视频流');
        }
      } catch (error) {
        if (!cancelled) setNotice(error instanceof Error ? error.message : '真机遥测暂时不可用');
      }
    };
    void refresh();
    const timer = window.setInterval(refresh, 1000);
    return () => {
      cancelled = true;
      window.clearInterval(timer);
    };
  }, [connected]);

  const connectDrone = async () => {
    setConnection('connecting');
    setNotice('正在连接真机…');
    try {
      const response = await fetch('/api/drone/connect', { method: 'POST' });
      if (!response.ok) throw new Error(await responseError(response, '连接失败'));
      const data = (await response.json()) as DroneStatus;
      setStatus(data);
      setConnection('connected');
      setNotice(data.videoReady ? '真机与视频流均已就绪' : '真机已连接，正在等待视频流');
    } catch (error) {
      setConnection('error');
      setStatus({});
      setNotice(error instanceof Error ? error.message : '未连接到真机控制服务');
    }
  };

  const sendAction = async (action: 'stop' | 'emergency-land') => {
    if (!connected) return;
    if (action === 'emergency-land' && !emergencyArmed) {
      setEmergencyArmed(true);
      setNotice('请再次点击确认紧急降落');
      return;
    }
    setEmergencyArmed(false);
    setNotice(action === 'stop' ? '正在停止任务并关闭真机连接…' : '紧急降落指令已发送');
    try {
      const response = await fetch(`/api/drone/${action}`, { method: 'POST' });
      if (!response.ok) throw new Error(await responseError(response, '控制指令发送失败'));
      setConnection('disconnected');
      setStatus({});
      setNotice(action === 'stop' ? '任务已停止，真机连接已关闭' : '降落指令已执行，真机连接已关闭');
    } catch (error) {
      setNotice(error instanceof Error ? error.message : '控制指令发送失败，请立即检查真机状态');
    }
  };

  const metricValue = (key: 'height' | 'tof' | 'rate') => {
    if (!connected) return '--';
    if (key === 'height') return status.heightCm ?? '--';
    if (key === 'tof') return status.frontTofCm == null ? '超量程/不可用' : status.frontTofCm;
    return status.controlHz?.toFixed(1) ?? '--';
  };

  return (
    <main className="app-shell">
      <header className="topbar">
        <a className="brand" href="#main-console" aria-label="返回 PhantomFilmer 主控制台">
          <span className="brand-mark" aria-hidden="true"><i /><i /><i /></span>
          <span>PhantomFilmer</span>
        </a>
        <div className="system-summary" aria-label="系统状态">
          <StatusPill
            tone={connected ? 'good' : connection === 'connecting' ? 'warn' : 'idle'}
            label={connected ? '真机已连接' : connection === 'connecting' ? '正在连接' : '真机未连接'}
          />
          <StatusPill tone={videoReady ? 'good' : 'idle'} label={videoReady ? '视频流正常' : '视频未开启'} />
          <div className="battery-status" aria-label={`电量 ${batteryLabel}`}>
            <span>电量</span><strong>{batteryLabel}</strong>
            <span className={`battery-icon ${connected ? '' : 'muted'}`}><i /></span>
          </div>
        </div>
        <div className="top-actions">
          <button className="pill-button stop" disabled={!connected} onClick={() => void sendAction('stop')}>停止任务</button>
          <button
            className={`pill-button emergency ${emergencyArmed ? 'armed' : ''}`}
            disabled={!connected}
            onClick={() => void sendAction('emergency-land')}
          >
            {emergencyArmed ? '再次确认降落' : '紧急降落'}
          </button>
        </div>
      </header>

      <section id="main-console" className="console-layout" aria-label="真机飞行控制台">
        <section className={`video-panel ${videoReady ? 'streaming' : ''}`} aria-label="真机内嵌视频">
          {videoReady ? (
            <>
              {/* eslint-disable-next-line @next/next/no-img-element */}
              <img className="video-stream" src="/api/drone/video/stream" alt="无人机实时视频流" />
              <div className="video-overlay">
                <span>实时画面</span>
                <span>{status.targetConfirmed ? '目标已确认' : '等待目标确认'}</span>
              </div>
            </>
          ) : (
            <div className="video-empty">
              <span className="viewfinder" aria-hidden="true"><i /><i /><i /><i /></span>
              <p className="eyebrow">REAL DEVICE VIDEO</p>
              <h1>{connection === 'connecting' ? '正在连接真机' : '视频流未开启'}</h1>
              <p>只有确认真机连接成功后，实时画面才会在这里显示。</p>
              <button className="pill-button connect" disabled={connection === 'connecting'} onClick={() => void connectDrone()}>
                {connection === 'connecting' ? '连接中…' : connection === 'error' ? '重新连接真机' : '连接真机'}
              </button>
              <span className={`inline-notice ${connection === 'error' ? 'error' : ''}`} role="status">{notice}</span>
            </div>
          )}
        </section>

        <aside className="monitor-panel" aria-label="实时监控">
          <div className="monitor-heading">
            <div><p className="eyebrow">FLIGHT STATUS</p><h2>当前状态</h2></div>
            <span className={`live-dot ${connected ? 'active' : ''}`} aria-label={connected ? '遥测在线' : '遥测离线'} />
          </div>
          <article className={`state-card primary ${connected ? '' : 'disabled-card'}`}>
            <span className="target-icon" aria-hidden="true"><i /></span>
            <div><span className="card-label">飞行模式</span><strong>{connected ? status.flightState ?? '待机' : '等待连接'}</strong></div>
            <span className={`check ${connected ? 'done' : ''}`}>{connected ? '✓' : '—'}</span>
          </article>
          <div className="metrics-list">
            {statusCards.map((card) => {
              const value = metricValue(card.key);
              return (
                <article className={`state-card ${connected ? '' : 'disabled-card'}`} key={card.key}>
                  <span className="metric-icon" aria-hidden="true">{card.icon}</span>
                  <div>
                    <span className="card-label">{card.label}</span>
                    <strong>{value} <small>{value === '--' || value === '超量程/不可用' ? '' : card.unit}</small></strong>
                  </div>
                </article>
              );
            })}
          </div>
          <article className={`state-card identity-card ${connected ? '' : 'disabled-card'}`}>
            <span className="person-icon" aria-hidden="true">人</span>
            <div><span className="card-label">人物识别</span><strong>{connected ? status.targetConfirmed ? '目标已确认' : '未启动识别任务' : '不可用'}</strong></div>
            <span className={`check ${status.targetConfirmed ? 'done' : ''}`}>{status.targetConfirmed ? '✓' : '—'}</span>
          </article>
        </aside>
      </section>

      <footer className="phase-bar" aria-label="飞行任务阶段">
        {phases.map((phase, index) => {
          const complete = currentPhaseIndex > index;
          const active = connected && currentPhase === phase;
          return (
            <div className="phase-wrap" key={phase}>
              <div className={`phase-item ${complete ? 'complete' : ''} ${active ? 'active' : ''}`}>
                <span className="phase-symbol">{complete ? '✓' : index + 1}</span><span>{phase}</span><i />
              </div>
              {index < phases.length - 1 && <span className={`phase-line ${complete ? 'complete' : ''}`} />}
            </div>
          );
        })}
      </footer>
    </main>
  );
}

function StatusPill({ tone, label }: { tone: 'good' | 'warn' | 'idle'; label: string }) {
  return <div className="status-pill"><span className={`status-dot ${tone}`} /><span>{label}</span></div>;
}
