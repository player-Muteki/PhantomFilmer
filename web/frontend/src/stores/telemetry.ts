import { create } from "zustand";
import type { Telemetry } from "../api/client";

interface TelemetryStore extends Telemetry {
  websocketOnline: boolean;
  update: (data: Telemetry) => void;
  setWebsocketOnline: (online: boolean) => void;
}

export const useTelemetryStore = create<TelemetryStore>((set) => ({
  battery: 0, height: 0, yaw: null, frontDistance: null, mode: "未连接",
  connected: false, airborne: false, streaming: false, taskActive: false, websocketOnline: false,
  connectionState: "DISCONNECTED", connectionVerified: false, statusAgeSeconds: null,
  connectionError: null, frontTofSupported: null, videoOwner: "NONE", videoError: null,
  update: (data) => set(data),
  setWebsocketOnline: (websocketOnline) => set({ websocketOnline }),
}));
