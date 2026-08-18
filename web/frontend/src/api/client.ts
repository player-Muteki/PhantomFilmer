export interface Telemetry {
  battery: number;
  height: number;
  yaw: number | null;
  frontDistance: number | null;
  mode: string;
  connected: boolean;
  airborne: boolean;
  streaming: boolean;
  taskActive: boolean;
  connectionState: "DISCONNECTED" | "CONNECTING" | "VERIFIED" | "DEGRADED" | "CLOSING";
  connectionVerified: boolean;
  statusAgeSeconds: number | null;
  connectionError: string | null;
  frontTofSupported: boolean | null;
  videoOwner: "NONE" | "WEB_PREVIEW" | "FOLLOW_SESSION";
  videoError: string | null;
}

type ApiTelemetry = Omit<Telemetry, "frontDistance" | "taskActive" | "connectionState" | "connectionVerified" | "statusAgeSeconds" | "connectionError" | "frontTofSupported" | "videoOwner" | "videoError"> & {
  front_distance: number | null;
  task_active: boolean;
  connection_state: Telemetry["connectionState"];
  connection_verified: boolean;
  status_age_seconds: number | null;
  connection_error: string | null;
  front_tof_supported: boolean | null;
  video_owner: Telemetry["videoOwner"];
  video_error: string | null;
};

export interface ApiClient {
  connect(): Promise<{ ok: boolean; mode: string; connection_state: string; connection_verified: boolean; battery: number }>;
  canStartTask(): Promise<{ allowed: boolean; message: string }>;
  startFollowTask(confirmed: boolean): Promise<{ ok: boolean; mode: string }>;
  stopTask(): Promise<{ ok: boolean; mode: string }>;
  emergencyStop(): Promise<{ ok: boolean; mode: string }>;
  subscribeTelemetry(handler: (data: Telemetry) => void, onConnection: (online: boolean) => void): () => void;
  getVideoStreamUrl(): string;
}

async function request<T>(path: string, init?: RequestInit): Promise<T> {
  const response = await fetch(path, init);
  if (!response.ok) {
    const payload = await response.json().catch(() => ({}));
    throw new Error(payload.detail || `请求失败 (${response.status})`);
  }
  return response.json();
}

class HttpApiClient implements ApiClient {
  connect() { return request<{ ok: boolean; mode: string; connection_state: string; connection_verified: boolean; battery: number }>("/api/connect", { method: "POST" }); }
  canStartTask() { return request<{ allowed: boolean; message: string }>("/api/task/can-start"); }
  startFollowTask(confirmed: boolean) { return request<{ ok: boolean; mode: string }>("/api/task/start", { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ confirmed }) }); }
  stopTask() { return request<{ ok: boolean; mode: string }>("/api/task/stop", { method: "POST" }); }
  emergencyStop() { return request<{ ok: boolean; mode: string }>("/api/emergency-stop", { method: "POST" }); }
  subscribeTelemetry(handler: (data: Telemetry) => void, onConnection: (online: boolean) => void) {
    let socket: WebSocket | null = null;
    let timer: number | undefined;
    let closed = false;
    const open = () => {
      const protocol = location.protocol === "https:" ? "wss" : "ws";
      socket = new WebSocket(`${protocol}://${location.host}/ws/telemetry`);
      socket.onopen = () => onConnection(true);
      socket.onmessage = (event) => {
        const data = JSON.parse(event.data) as ApiTelemetry;
        handler({
          ...data,
          frontDistance: data.front_distance,
          taskActive: data.task_active,
          connectionState: data.connection_state,
          connectionVerified: data.connection_verified,
          statusAgeSeconds: data.status_age_seconds,
          connectionError: data.connection_error,
          frontTofSupported: data.front_tof_supported,
          videoOwner: data.video_owner,
          videoError: data.video_error,
        });
      };
      socket.onclose = () => {
        onConnection(false);
        if (!closed) timer = window.setTimeout(open, 1500);
      };
      socket.onerror = () => socket?.close();
    };
    open();
    return () => { closed = true; if (timer) clearTimeout(timer); socket?.close(); };
  }
  getVideoStreamUrl() { return "/video/stream"; }
}

export const apiClient: ApiClient = new HttpApiClient();
