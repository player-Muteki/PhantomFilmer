import { create } from "zustand";

export interface LogEntry { id: number; title: string; detail: string; time: string; tone?: "ok" | "warn" | "danger"; }
interface TaskStore {
  busy: boolean;
  logs: LogEntry[];
  setBusy: (busy: boolean) => void;
  addLog: (title: string, detail: string, tone?: LogEntry["tone"]) => void;
  clearLogs: () => void;
}

export const useTaskStore = create<TaskStore>((set) => ({
  busy: false,
  logs: [{ id: 1, title: "控制台已就绪", detail: "等待连接飞行器", time: new Date().toLocaleTimeString("zh-CN", { hour12: false }) }],
  setBusy: (busy) => set({ busy }),
  addLog: (title, detail, tone) => set((state) => ({ logs: [{ id: Date.now(), title, detail, time: new Date().toLocaleTimeString("zh-CN", { hour12: false }), tone }, ...state.logs].slice(0, 8) })),
  clearLogs: () => set({ logs: [] }),
}));
