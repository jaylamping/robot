import { create } from 'zustand'

export interface MotorSnapshot {
  can_id: number
  joint_name: string
  angle_rad: number
  velocity_rads: number
  torque_nm: number
  temperature_c: number
  mode: string
  faults: string[]
  online: boolean
}

export interface TelemetrySnapshot {
  timestamp_ms: number
  motors: MotorSnapshot[]
  system?: SystemSnapshot
}

export interface SystemSnapshot {
  cpu_usage_percent: number
  memory_used_mb: number
  memory_total_mb: number
  temperature_c?: number | null
}

const HISTORY_MAX = 600

interface TelemetryState {
  motors: Record<number, MotorSnapshot>
  history: Record<number, MotorSnapshot[]>
  lastTimestamp: number
  connected: boolean
  system: SystemSnapshot | null

  updateSnapshot: (snap: TelemetrySnapshot) => void
  setConnected: (connected: boolean) => void
}

export const useTelemetryStore = create<TelemetryState>((set) => ({
  motors: {},
  history: {},
  lastTimestamp: 0,
  connected: false,
  system: null,

  updateSnapshot: (snap) =>
    set((state) => {
      const nextMotors = { ...state.motors }
      const nextHistory = { ...state.history }

      for (const m of snap.motors) {
        nextMotors[m.can_id] = m

        const prev = nextHistory[m.can_id] ?? []
        const updated = [...prev, m]
        nextHistory[m.can_id] =
          updated.length > HISTORY_MAX
            ? updated.slice(updated.length - HISTORY_MAX)
            : updated
      }

      return {
        motors: nextMotors,
        history: nextHistory,
        lastTimestamp: snap.timestamp_ms,
        system: snap.system ?? state.system,
      }
    }),

  setConnected: (connected) => set({ connected }),
}))
