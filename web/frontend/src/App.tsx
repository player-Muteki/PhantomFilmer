import { useEffect, useRef, useState } from "react";
import { AlertOutlined, CameraOutlined, CompassOutlined, DashboardOutlined, LinkOutlined, LoadingOutlined, SafetyCertificateOutlined, ThunderboltOutlined, WifiOutlined } from "@ant-design/icons";
import { Modal, message } from "antd";
import { apiClient } from "./api/client";
import { useTaskStore } from "./stores/task";
import { useTelemetryStore } from "./stores/telemetry";

const displayNumber = (value: number | null) => value === null ? "--" : String(Math.round(value));

function App() {
  const telemetry = useTelemetryStore();
  const { busy, logs, setBusy, addLog, clearLogs } = useTaskStore();
  const [videoError, setVideoError] = useState(false);
  const [videoKey, setVideoKey] = useState(0);
  const emergencyTimer = useRef<number>();

  useEffect(() => apiClient.subscribeTelemetry(telemetry.update, telemetry.setWebsocketOnline), []);
  useEffect(() => { if (telemetry.connectionVerified) { setVideoError(false); setVideoKey((key) => key + 1); } }, [telemetry.connectionVerified]);

  const run = async (work: () => Promise<unknown>, success: [string, string], tone: "ok" | "warn" | "danger" = "ok") => {
    setBusy(true);
    try { await work(); addLog(success[0], success[1], tone); message.success(success[0]); }
    catch (error) { const detail = error instanceof Error ? error.message : "未知错误"; addLog("操作未完成", detail, "danger"); message.error(detail); }
    finally { setBusy(false); }
  };
  const connect = () => run(() => apiClient.connect(), ["真机连接已验证", "SDK 响应、实时状态与电量检查均已通过"]);
  const startFollow = async () => {
    setBusy(true);
    try {
      const check = await apiClient.canStartTask();
      if (!check.allowed) throw new Error(check.message);
      Modal.confirm({ title: "确认启动自主跟随？", icon: <SafetyCertificateOutlined />, content: <div className="confirm-copy"><p>{check.message}</p><p>请确认航线净空、已安装保护罩，人员远离无人机。</p></div>, okText: "确认起飞并跟随", cancelText: "取消", okButtonProps: { danger: true }, onOk: () => run(() => apiClient.startFollowTask(true), ["跟随任务已启动", "飞行控制已交给安全任务层"]) });
    } catch (error) { const detail = error instanceof Error ? error.message : "起飞检查失败"; addLog("起飞检查未通过", detail, "danger"); message.error(detail); }
    finally { setBusy(false); }
  };
  const stopTask = () => run(() => apiClient.stopTask(), ["任务已停止", "无人机已进入安全降落流程"], "warn");
  const emergencyStop = () => run(() => apiClient.emergencyStop(), ["急停已执行", "控制输出已清零并强制降落"], "danger");
  const beginEmergency = () => { window.clearTimeout(emergencyTimer.current); emergencyTimer.current = window.setTimeout(emergencyStop, 500); };
  const cancelEmergency = () => window.clearTimeout(emergencyTimer.current);

  const dangerBattery = telemetry.connectionVerified && telemetry.battery < 20;
  const dangerHeight = telemetry.height > 220;
  const dangerDistance = telemetry.frontDistance !== null && telemetry.frontDistance < 60;
  const metrics = [
    { icon: <ThunderboltOutlined />, label: "剩余电量", value: telemetry.connectionVerified ? telemetry.battery : "--", unit: "%", tone: dangerBattery ? "danger" : "cyan", ratio: telemetry.battery },
    { icon: <DashboardOutlined />, label: "下向 TOF 高度", value: telemetry.connectionVerified ? telemetry.height : "--", unit: "cm", tone: dangerHeight ? "danger" : "blue", ratio: Math.min(100, telemetry.height / 2.5) },
    { icon: <CompassOutlined />, label: "当前航向", value: displayNumber(telemetry.yaw), unit: "°", tone: "violet", ratio: telemetry.yaw === null ? 0 : ((telemetry.yaw + 360) % 360) / 3.6 },
    { icon: <SafetyCertificateOutlined />, label: "前向 TOF（可选）", value: telemetry.frontTofSupported === false ? "未安装" : displayNumber(telemetry.frontDistance), unit: telemetry.frontTofSupported === false ? "" : "cm", tone: dangerDistance ? "danger" : "green", ratio: telemetry.frontDistance === null ? 0 : Math.min(100, telemetry.frontDistance / 2.4) },
  ];
  const checks = [["真机连接已验证", telemetry.connectionVerified], ["状态数据未过期", telemetry.connectionVerified && telemetry.statusAgeSeconds !== null && telemetry.statusAgeSeconds <= 5], ["电量安全阈值", telemetry.connectionVerified && telemetry.battery >= 30]] as const;
  const readyCount = checks.filter(([, ready]) => ready).length;

  return <main className={`console-shell ${dangerBattery && telemetry.battery < 5 ? "critical-flash" : ""}`}>
    <header className="topbar"><div className="brand-lockup"><div className="brand-mark"><span /></div><div><h1>PHANTOM<span>FILMER</span></h1><p>自主跟随飞行控制台</p></div></div><div className="system-state"><span className={telemetry.websocketOnline ? "pulse-dot" : "pulse-dot offline"} /><div><b>{telemetry.websocketOnline ? "控制台在线" : "控制台通信断开"}</b><small>TELEMETRY {telemetry.websocketOnline ? "CACHE 5 HZ" : "RECONNECTING"}</small></div></div><button className="emergency-mini" disabled={!telemetry.connectionVerified || busy} aria-label="长按 500 毫秒急停" onPointerDown={beginEmergency} onPointerUp={cancelEmergency} onPointerLeave={cancelEmergency} onKeyDown={(e) => { if (e.key === " " || e.key === "Enter") beginEmergency(); }} onKeyUp={cancelEmergency}><AlertOutlined /> 长按急停</button></header>
    <section className="mission-strip"><div><span className="eyebrow">任务</span><strong>PF-082 · 目标跟随</strong></div><div className="mission-tags"><span><WifiOutlined /> {telemetry.connectionVerified ? "真机连接已验证" : telemetry.connectionState === "DEGRADED" ? "真机连接异常" : "等待真机连接"}</span><span><SafetyCertificateOutlined /> {telemetry.connectionVerified && telemetry.battery >= 30 ? "电量检查通过" : "禁止启动飞行"}</span></div><div className="flight-mode"><small>当前模式</small><b>{telemetry.mode}</b></div></section>
    <section className="workspace-grid">
      <article className="panel video-panel"><div className="panel-head"><div><span className="kicker">LIVE FEED</span><h2>真机视频</h2></div><span className={`live-badge ${telemetry.streaming && !videoError ? "is-live" : ""}`}><i /> {telemetry.streaming && !videoError ? "LIVE" : "STANDBY"}</span></div><div className="video-stage">{telemetry.connectionVerified && !videoError && <img key={videoKey} className="video-stream" src={`${apiClient.getVideoStreamUrl()}?v=${videoKey}`} alt="无人机实时视频" onError={() => setVideoError(true)} />}{(!telemetry.connectionVerified || videoError) && <div className="video-empty"><CameraOutlined /><b>{videoError ? "真机视频流中断" : "等待真机验证"}</b><span>{videoError ? "检查图传与 Wi-Fi 后点击重连" : "验证真实无人机连接后才会启动图传"}</span>{videoError && telemetry.connectionVerified && <button className="retry-video" onClick={() => { setVideoError(false); setVideoKey((key) => key + 1); }}>重连视频</button>}</div>}</div><div className="video-footer"><span>画面来自无人机实时视频流</span><span className="latency">STREAM <b>{telemetry.streaming ? "MJPEG" : "OFFLINE"}</b></span></div></article>
      <aside className="telemetry-column"><div className="section-title"><span className="kicker">FLIGHT DATA</span><h2>实时遥测</h2></div><div className="metric-grid">{metrics.map((item) => <div className={`metric-card ${item.tone}`} key={item.label}><div className="metric-icon">{item.icon}</div><span>{item.label}</span><strong>{item.value}<small>{item.unit}</small></strong><div className="metric-line"><i style={{ width: `${item.ratio}%` }} /></div></div>)}</div><div className="sensor-note"><SafetyCertificateOutlined /><div><b>前向 TOF 为可选扩展</b><span>未安装或暂时无数据不会阻塞真机连接与任务启动；核心避障配置开启时仍执行原有安全策略。</span></div></div></aside>
      <aside className="control-column panel"><div className="panel-head"><div><span className="kicker">MISSION CONTROL</span><h2>任务控制</h2></div><span className={`status-pill ${telemetry.taskActive ? "active" : ""}`}>{telemetry.mode}</span></div><button className="connect-button" onClick={connect} disabled={telemetry.connectionVerified || telemetry.connectionState === "CONNECTING" || busy}>{busy ? <LoadingOutlined /> : <LinkOutlined />}<span><b>{telemetry.connectionVerified ? "真机连接已验证" : telemetry.connectionState === "DEGRADED" ? "重新验证真机" : "连接并验证真机"}</b><small>{telemetry.connectionVerified ? "AIRCRAFT VERIFIED" : "VERIFY REAL AIRCRAFT"}</small></span><i>{telemetry.connectionVerified ? "✓" : "→"}</i></button><div className="readiness"><div className="readiness-head"><span>起飞就绪检查</span><b>{readyCount} / {checks.length}</b></div>{checks.map(([label, ready]) => <div className="check-row" key={label}><i className={ready ? "done" : "pending"}>{ready ? "✓" : ""}</i><span>{label}</span><small>{ready ? "通过" : "待检查"}</small></div>)}</div><div className="action-row"><button className="primary-action" disabled={!telemetry.connectionVerified || telemetry.battery < 30 || telemetry.taskActive || busy} onClick={startFollow}>开始跟随</button><button className="stop-action" disabled={!telemetry.taskActive || busy} onClick={stopTask}>停止任务</button></div><div className="safety-note"><SafetyCertificateOutlined /><span><b>硬门禁</b>只有真机连接已验证且即时电量检查通过，服务端才允许启动任务。</span></div><div className="timeline"><div className="timeline-head"><span>任务日志</span><button onClick={clearLogs}>清空</button></div>{logs.length === 0 ? <div className="empty-logs">暂无日志</div> : logs.map((entry, index) => <div className={`log-entry ${index === 0 ? "active" : ""} ${entry.tone || ""}`} key={entry.id}><time>{entry.time}</time><div><b>{entry.title}</b><span>{entry.detail}</span></div></div>)}</div></aside>
    </section>
    <footer className="statusbar"><span><i className={telemetry.websocketOnline ? "ok" : ""} /> CORE {telemetry.websocketOnline ? "ONLINE" : "RECONNECTING"}</span><span><i className={telemetry.connectionVerified ? "ok" : ""} /> AIRCRAFT {telemetry.connectionState}</span><span><i className={telemetry.streaming ? "ok" : ""} /> VIDEO {telemetry.streaming ? "ACTIVE" : "STANDBY"}</span><div>LOCAL CONTROL <b>{location.host}</b></div></footer>
  </main>;
}

export default App;
