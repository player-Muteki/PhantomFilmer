'use client';

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';

type ConnectionState = 'disconnected' | 'connecting' | 'connected' | 'error';
type FlightPhase = '连接' | '检查' | '起飞' | '手动飞行' | '降落';
type ArmedAction = 'takeoff' | 'land' | 'emergency' | null;
type RcCommand = { leftRight?: number; forwardBack?: number; upDown?: number; yaw?: number };

type DroneStatus = {
  battery?: number;
  heightCm?: number;
  frontTofCm?: number | null;
  frontTofState?: 'clear' | 'blocked' | 'out_of_range' | 'unavailable';
  controlHz?: number;
  flightState?: string;
  phase?: FlightPhase;
  videoReady?: boolean;
  airborne?: boolean;
  canTakeoff?: boolean;
  rcEnabled?: boolean;
  preflight?: {
    sdk?: boolean;
    video?: boolean;
    battery?: boolean;
    bottomTof?: boolean;
    frontTof?: boolean;
  };
};

const phases: FlightPhase[] = ['连接', '检查', '起飞', '手动飞行', '降落'];
const emptyCommand = { leftRight: 0, forwardBack: 0, upDown: 0, yaw: 0 };

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
  const [armedAction, setArmedAction] = useState<ArmedAction>(null);
  const [actionBusy, setActionBusy] = useState<string | null>(null);
  const [speed, setSpeed] = useState(20);
  const [activeControl, setActiveControl] = useState<string | null>(null);
  const controlTimer = useRef<number | null>(null);
  const rcInFlight = useRef(false);

  const connected = connection === 'connected';
  const videoReady = connected && status.videoReady === true;
  const airborne = connected && status.airborne === true;
  const currentPhase = status.phase ?? '连接';
  const currentPhaseIndex = connected ? phases.indexOf(currentPhase) : -1;
  const batteryLabel = useMemo(
    () => (!connected || status.battery == null ? '--' : `${status.battery}%`),
    [connected, status.battery],
  );

  useEffect(() => {
    if (!armedAction) return;
    const timer = window.setTimeout(() => setArmedAction(null), 4000);
    return () => window.clearTimeout(timer);
  }, [armedAction]);

  useEffect(() => {
    if (!connected) return;
    let cancelled = false;
    const refresh = async () => {
      try {
        const response = await fetch('/api/drone/status', { cache: 'no-store' });
        if (!response.ok) throw new Error(await responseError(response, '遥测不可用'));
        const data = (await response.json()) as DroneStatus;
        if (!cancelled) setStatus(data);
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
    setActionBusy('connect');
    setNotice('正在连接真机并检查视频…');
    try {
      const response = await fetch('/api/drone/connect', { method: 'POST' });
      if (!response.ok) throw new Error(await responseError(response, '连接失败'));
      const data = (await response.json()) as DroneStatus;
      setStatus(data);
      setConnection('connected');
      setNotice(data.videoReady ? '真机与视频流均已就绪' : '真机已连接，正在等待有效视频帧');
    } catch (error) {
      setConnection('error');
      setStatus({});
      setNotice(error instanceof Error ? error.message : '未连接到真机控制服务');
    } finally {
      setActionBusy(null);
    }
  };

  const postStatusAction = async (endpoint: 'takeoff' | 'land' | 'hover') => {
    setActionBusy(endpoint);
    try {
      const response = await fetch(`/api/drone/${endpoint}`, { method: 'POST' });
      if (!response.ok) throw new Error(await responseError(response, '飞行指令执行失败'));
      const data = (await response.json()) as DroneStatus;
      setStatus(data);
      setNotice(endpoint === 'takeoff' ? '起飞完成，手动控制已启用' : endpoint === 'land' ? '降落完成，视频保持连接' : '已悬停');
    } catch (error) {
      setNotice(error instanceof Error ? error.message : '飞行指令执行失败');
    } finally {
      setActionBusy(null);
      setArmedAction(null);
    }
  };

  const confirmAction = (action: Exclude<ArmedAction, null>) => {
    if (armedAction !== action) {
      setArmedAction(action);
      setNotice(action === 'takeoff' ? '请再次点击确认起飞' : action === 'land' ? '请再次点击确认正常降落' : '请再次点击确认紧急降落');
      return;
    }
    if (action === 'takeoff') void postStatusAction('takeoff');
    if (action === 'land') void postStatusAction('land');
    if (action === 'emergency') void disconnectAction('emergency-land');
  };

  const disconnectAction = async (endpoint: 'stop' | 'emergency-land') => {
    setActionBusy(endpoint);
    setArmedAction(null);
    setNotice(endpoint === 'stop' ? '正在安全停止；如已起飞将先降落…' : '紧急降落指令已发送');
    try {
      const response = await fetch(`/api/drone/${endpoint}`, { method: 'POST' });
      if (!response.ok) throw new Error(await responseError(response, '指令发送失败'));
      setConnection('disconnected');
      setStatus({});
      setNotice(endpoint === 'stop' ? '任务已停止，真机连接已关闭' : '紧急降落完成，连接已关闭');
    } catch (error) {
      setNotice(error instanceof Error ? error.message : '指令发送失败，请立即检查真机');
    } finally {
      setActionBusy(null);
    }
  };

  const sendRc = useCallback(async (command: RcCommand) => {
    if (rcInFlight.current) return;
    rcInFlight.current = true;
    try {
      const response = await fetch('/api/drone/rc', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ ...emptyCommand, ...command }),
      });
      if (!response.ok) throw new Error(await responseError(response, '手动控制被安全系统拒绝'));
    } catch (error) {
      setNotice(error instanceof Error ? error.message : '手动控制发送失败');
    } finally {
      rcInFlight.current = false;
    }
  }, []);

  const stopControl = useCallback(async () => {
    if (controlTimer.current != null) {
      window.clearInterval(controlTimer.current);
      controlTimer.current = null;
    }
    setActiveControl(null);
    if (!airborne) return;
    try {
      const response = await fetch('/api/drone/hover', { method: 'POST' });
      if (!response.ok) throw new Error(await responseError(response, '悬停指令失败'));
      const data = (await response.json()) as DroneStatus;
      setStatus(data);
      setNotice('已悬停');
    } catch (error) {
      setNotice(error instanceof Error ? error.message : '悬停指令失败');
    }
  }, [airborne]);

  const startControl = useCallback((name: string, command: RcCommand) => {
    if (!airborne || actionBusy) return;
    if (controlTimer.current != null) window.clearInterval(controlTimer.current);
    setActiveControl(name);
    void sendRc(command);
    controlTimer.current = window.setInterval(() => void sendRc(command), 180);
  }, [actionBusy, airborne, sendRc]);

  useEffect(() => {
    const keyCommands: Record<string, [string, RcCommand]> = {
      KeyW: ['forward', { forwardBack: speed }],
      KeyS: ['back', { forwardBack: -speed }],
      KeyA: ['left', { leftRight: -speed }],
      KeyD: ['right', { leftRight: speed }],
      KeyR: ['up', { upDown: speed }],
      KeyF: ['down', { upDown: -speed }],
      KeyJ: ['yaw-left', { yaw: -speed }],
      KeyL: ['yaw-right', { yaw: speed }],
    };
    const down = (event: KeyboardEvent) => {
      if (event.code === 'Space') {
        event.preventDefault();
        if (!event.repeat) void stopControl();
        return;
      }
      const entry = keyCommands[event.code];
      if (!entry || event.repeat) return;
      event.preventDefault();
      startControl(entry[0], entry[1]);
    };
    const up = (event: KeyboardEvent) => {
      if (keyCommands[event.code]) {
        event.preventDefault();
        void stopControl();
      }
    };
    window.addEventListener('keydown', down);
    window.addEventListener('keyup', up);
    window.addEventListener('blur', stopControl);
    return () => {
      window.removeEventListener('keydown', down);
      window.removeEventListener('keyup', up);
      window.removeEventListener('blur', stopControl);
    };
  }, [speed, startControl, stopControl]);

  const metricValue = (key: 'height' | 'tof' | 'rate') => {
    if (!connected) return '--';
    if (key === 'height') return status.heightCm ?? '--';
    if (key === 'tof') return status.frontTofCm == null ? (status.frontTofState === 'out_of_range' ? '超量程' : '不可用') : status.frontTofCm;
    return status.controlHz?.toFixed(1) ?? '--';
  };

  const preflightItems = [
    ['SDK 连接', status.preflight?.sdk],
    ['有效视频帧', status.preflight?.video],
    ['电量 ≥ 20%', status.preflight?.battery],
    ['底部 ToF', status.preflight?.bottomTof],
    ['前向 ToF', status.preflight?.frontTof],
  ] as const;

  return (
    <main className="app-shell">
      <header className="topbar">
        <a className="brand" href="#main-console" aria-label="返回 PhantomFilmer 主控制台">
          <span className="brand-mark" aria-hidden="true"><i /><i /><i /></span><span>PhantomFilmer</span>
        </a>
        <div className="system-summary" aria-label="系统状态">
          <StatusPill tone={connected ? 'good' : connection === 'connecting' ? 'warn' : 'idle'} label={connected ? '真机已连接' : connection === 'connecting' ? '正在连接' : '真机未连接'} />
          <StatusPill tone={videoReady ? 'good' : 'idle'} label={videoReady ? '视频流正常' : '视频未开启'} />
          <StatusPill tone={airborne ? 'warn' : 'idle'} label={airborne ? '飞行中' : connected ? '地面' : '未就绪'} />
          <div className="battery-status"><span>电量</span><strong>{batteryLabel}</strong><span className={`battery-icon ${connected ? '' : 'muted'}`}><i /></span></div>
        </div>
        <div className="top-actions">
          <button className="pill-button stop" disabled={!connected || actionBusy != null} onClick={() => void disconnectAction('stop')}>停止并断开</button>
          <button className={`pill-button emergency ${armedAction === 'emergency' ? 'armed' : ''}`} disabled={!airborne || actionBusy != null} onClick={() => confirmAction('emergency')}>
            {armedAction === 'emergency' ? '再次确认降落' : '紧急降落'}
          </button>
        </div>
      </header>

      <section id="main-console" className="console-layout" aria-label="真机飞行控制台">
        <section className={`video-panel ${videoReady ? 'streaming' : ''}`} aria-label="真机内嵌视频">
          {videoReady ? (
            <>
              {/* eslint-disable-next-line @next/next/no-img-element */}
              <img className="video-stream" src="/api/drone/video/stream" alt="无人机实时视频流" />
              <div className="video-overlay"><span>实时画面</span><span>{status.flightState ?? '地面待机'}</span></div>
            </>
          ) : (
            <div className="video-empty">
              <span className="viewfinder" aria-hidden="true"><i /><i /><i /><i /></span>
              <p className="eyebrow">REAL DEVICE VIDEO</p>
              <h1>{connection === 'connecting' ? '正在连接真机' : '视频流未开启'}</h1>
              <p>只有确认真机连接成功后，实时画面才会在这里显示。</p>
              <button className="pill-button connect" disabled={connection === 'connecting'} onClick={() => void connectDrone()}>{connection === 'connecting' ? '连接中…' : connection === 'error' ? '重新连接真机' : '连接真机'}</button>
            </div>
          )}
          <div className={`notice-bar ${connection === 'error' ? 'error' : ''}`} role="status"><span>{actionBusy ? '处理中' : '状态'}</span>{notice}</div>
        </section>

        <aside className="monitor-panel" aria-label="实时监控">
          <div className="monitor-heading"><div><p className="eyebrow">FLIGHT STATUS</p><h2>当前状态</h2></div><span className={`live-dot ${connected ? 'active' : ''}`} /></div>
          <article className={`state-card primary ${connected ? '' : 'disabled-card'}`}><span className="target-icon"><i /></span><div><span className="card-label">飞行状态</span><strong>{connected ? status.flightState ?? '待机' : '等待连接'}</strong></div><span className={`check ${connected ? 'done' : ''}`}>{connected ? '✓' : '—'}</span></article>
          <div className="metrics-list">
            {[
              ['height', '高度', 'cm', '↕'], ['tof', '前向 ToF', 'cm', '◎'], ['rate', '视频率', 'Hz', '∿'],
            ].map(([key, label, unit, icon]) => {
              const value = metricValue(key as 'height' | 'tof' | 'rate');
              return <article className={`state-card compact ${connected ? '' : 'disabled-card'}`} key={key}><span className="metric-icon">{icon}</span><div><span className="card-label">{label}</span><strong>{value} <small>{typeof value === 'number' || /^\d/.test(String(value)) ? unit : ''}</small></strong></div></article>;
            })}
          </div>
          <section className="preflight-card">
            <div className="section-title"><div><p className="eyebrow">PRE-FLIGHT</p><h3>起飞检查</h3></div><span>{status.canTakeoff ? '可以起飞' : '等待通过'}</span></div>
            <div className="checklist">{preflightItems.map(([label, ok]) => <div key={label}><span className={`mini-check ${ok ? 'ok' : ''}`}>{ok ? '✓' : '—'}</span><span>{label}</span></div>)}</div>
          </section>
        </aside>
      </section>

      <section className="flight-deck" aria-label="飞行操作">
        <div className="flight-actions-panel">
          <div className="section-title"><div><p className="eyebrow">FLIGHT ACTIONS</p><h2>飞行操作</h2></div><span className={`mode-badge ${airborne ? 'air' : ''}`}>{airborne ? '空中' : '地面'}</span></div>
          <p className="panel-help">危险动作需要连续点击两次确认。停止并断开会在空中时先执行降落。</p>
          <div className="flight-action-grid">
            <button className={`action-button takeoff ${armedAction === 'takeoff' ? 'armed' : ''}`} disabled={!status.canTakeoff || actionBusy != null} onClick={() => confirmAction('takeoff')}><span>↑</span><strong>{armedAction === 'takeoff' ? '确认起飞' : '起飞'}</strong><small>检查全部通过后启用</small></button>
            <button className={`action-button land ${armedAction === 'land' ? 'armed' : ''}`} disabled={!airborne || actionBusy != null} onClick={() => confirmAction('land')}><span>↓</span><strong>{armedAction === 'land' ? '确认降落' : '正常降落'}</strong><small>保留视频与连接</small></button>
            <button className="action-button hover" disabled={!airborne || actionBusy != null} onClick={() => void postStatusAction('hover')}><span>•</span><strong>立即悬停</strong><small>清零所有 RC 通道</small></button>
          </div>
        </div>

        <div className={`manual-panel ${airborne ? '' : 'disabled-manual'}`}>
          <div className="manual-heading"><div><p className="eyebrow">MANUAL CONTROL</p><h2>手动控制</h2></div><div className="speed-select" aria-label="控制速度">{[15, 20, 30].map((value) => <button key={value} className={speed === value ? 'active' : ''} onClick={() => setSpeed(value)} disabled={actionBusy != null}>{value}</button>)}</div></div>
          <div className="control-layout">
            <ControlPad title="平面移动" controls={[
              ['forward', 'W', '前进', { forwardBack: speed }, 'up'], ['left', 'A', '左移', { leftRight: -speed }, 'left'], ['hover', '空格', '悬停', {}, 'center'], ['right', 'D', '右移', { leftRight: speed }, 'right'], ['back', 'S', '后退', { forwardBack: -speed }, 'down'],
            ]} active={activeControl} enabled={airborne && !actionBusy} onStart={startControl} onStop={stopControl} />
            <ControlPad title="高度与偏航" controls={[
              ['up', 'R', '上升', { upDown: speed }, 'up'], ['yaw-left', 'J', '左转', { yaw: -speed }, 'left'], ['hover-2', '空格', '悬停', {}, 'center'], ['yaw-right', 'L', '右转', { yaw: speed }, 'right'], ['down', 'F', '下降', { upDown: -speed }, 'down'],
            ]} active={activeControl} enabled={airborne && !actionBusy} onStart={startControl} onStop={stopControl} />
          </div>
          <p className="keyboard-help">按住按钮或键盘持续运动，松开立即悬停；后端 0.4 秒看门狗会自动清零失联指令。</p>
        </div>
      </section>

      <footer className="phase-bar" aria-label="飞行任务阶段">
        {phases.map((phase, index) => {
          const complete = currentPhaseIndex > index;
          const active = connected && currentPhase === phase;
          return <div className="phase-wrap" key={phase}><div className={`phase-item ${complete ? 'complete' : ''} ${active ? 'active' : ''}`}><span className="phase-symbol">{complete ? '✓' : index + 1}</span><span>{phase}</span><i /></div>{index < phases.length - 1 && <span className={`phase-line ${complete ? 'complete' : ''}`} />}</div>;
        })}
      </footer>
    </main>
  );
}

type Control = [string, string, string, RcCommand, 'up' | 'left' | 'center' | 'right' | 'down'];

function ControlPad({ title, controls, active, enabled, onStart, onStop }: { title: string; controls: Control[]; active: string | null; enabled: boolean; onStart: (name: string, command: RcCommand) => void; onStop: () => Promise<void> }) {
  return <div className="control-pad"><span className="control-title">{title}</span><div className="pad-grid">{controls.map(([name, key, label, command, position]) => <button key={name} className={`control-key ${position} ${active === name ? 'active' : ''}`} disabled={!enabled} onPointerDown={(event) => { event.preventDefault(); event.currentTarget.setPointerCapture(event.pointerId); if (name.startsWith('hover')) void onStop(); else onStart(name, command); }} onPointerUp={() => void onStop()} onPointerCancel={() => void onStop()}><kbd>{key}</kbd><span>{label}</span></button>)}</div></div>;
}

function StatusPill({ tone, label }: { tone: 'good' | 'warn' | 'idle'; label: string }) {
  return <div className="status-pill"><span className={`status-dot ${tone}`} /><span>{label}</span></div>;
}
