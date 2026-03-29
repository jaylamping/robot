import { keepPreviousData, useQueries, useQuery } from '@tanstack/react-query'
import {
  getArmPreflight,
  getArms,
  getConfig,
  getJointSlots,
  getLogs,
  getMotor,
  getMotors,
  getSequences,
  getStatus,
} from '@/lib/api'
import { linkKeys } from './keys'

/** Config / arms-shaped data — changes rarely; align with long-ish dashboard stale windows. */
const STALE_CONFIG_MS = 1000 * 60 * 2

/** Lists that should feel fresh but not hammer the Pi. */
const STALE_LIST_MS = 1000 * 30

/** Status bar / polling endpoints. */
const STALE_STATUS_MS = 1000 * 5

/** Preflight is expensive and should not refetch on every focus. */
const STALE_PREFLIGHT_MS = 1000 * 60

export function useRobotConfig() {
  return useQuery({
    queryKey: linkKeys.config(),
    queryFn: getConfig,
    placeholderData: keepPreviousData,
    staleTime: STALE_CONFIG_MS,
  })
}

export function useRobotMotors() {
  return useQuery({
    queryKey: linkKeys.motors(),
    queryFn: getMotors,
    placeholderData: keepPreviousData,
    staleTime: STALE_LIST_MS,
  })
}

export function useRobotJointSlots() {
  return useQuery({
    queryKey: linkKeys.jointSlots(),
    queryFn: getJointSlots,
    placeholderData: keepPreviousData,
    staleTime: STALE_LIST_MS,
  })
}

export function useRobotMotor(canId: number, enabled = true) {
  return useQuery({
    queryKey: linkKeys.motor(canId),
    queryFn: () => getMotor(canId),
    placeholderData: keepPreviousData,
    staleTime: STALE_LIST_MS,
    enabled: enabled && Number.isFinite(canId),
  })
}

export function useRobotArms() {
  return useQuery({
    queryKey: linkKeys.arms(),
    queryFn: getArms,
    placeholderData: keepPreviousData,
    staleTime: STALE_LIST_MS,
  })
}

export function useRobotServerStatus(options?: { refetchInterval?: number | false }) {
  return useQuery({
    queryKey: linkKeys.status(),
    queryFn: getStatus,
    placeholderData: keepPreviousData,
    staleTime: STALE_STATUS_MS,
    refetchInterval: options?.refetchInterval ?? false,
  })
}

export function useRobotSequences() {
  return useQuery({
    queryKey: linkKeys.sequences(),
    queryFn: getSequences,
    placeholderData: keepPreviousData,
    staleTime: STALE_LIST_MS,
  })
}

export function useRobotLogs(limit: number, options?: { refetchInterval?: number | false }) {
  return useQuery({
    queryKey: linkKeys.logs(limit),
    queryFn: () => getLogs(limit),
    placeholderData: keepPreviousData,
    staleTime: STALE_LIST_MS,
    refetchInterval: options?.refetchInterval,
  })
}

/** Pass `side` like passing a query object into `useAudienceCount` — it becomes part of `queryKey`. */
export function useRobotArmPreflight(side: string) {
  return useQuery({
    queryKey: linkKeys.armPreflight(side),
    queryFn: () => getArmPreflight(side),
    placeholderData: keepPreviousData,
    staleTime: STALE_PREFLIGHT_MS,
    refetchOnWindowFocus: false,
    enabled: !!side,
  })
}

/** Overview homing card: one observer per arm side, same key shape as `useRobotArmPreflight`. */
export function useRobotArmPreflights(armSides: string[]) {
  return useQueries({
    queries: armSides.map((side) => ({
      queryKey: linkKeys.armPreflight(side),
      queryFn: () => getArmPreflight(side),
      placeholderData: keepPreviousData,
      staleTime: STALE_PREFLIGHT_MS,
      refetchOnWindowFocus: false,
    })),
  })
}
