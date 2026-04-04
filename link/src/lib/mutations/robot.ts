import { useMutation, useQueryClient } from '@tanstack/react-query'
import type { QueryClient } from '@tanstack/react-query'
import * as api from '@/lib/api'
import { linkKeys } from '@/lib/queries/keys'

export function invalidateAfterConfigOrMotorListChange(qc: QueryClient) {
  void qc.invalidateQueries({ queryKey: linkKeys.config() })
  void qc.invalidateQueries({ queryKey: linkKeys.motors() })
  void qc.invalidateQueries({ queryKey: linkKeys.jointSlots() })
  void qc.invalidateQueries({ queryKey: linkKeys.arms() })
  void qc.invalidateQueries({ queryKey: linkKeys.armPreflightsRoot() })
}

export function invalidateAfterJointYamlChange(qc: QueryClient) {
  void qc.invalidateQueries({ queryKey: linkKeys.config() })
  void qc.invalidateQueries({ queryKey: linkKeys.arms() })
  void qc.invalidateQueries({ queryKey: linkKeys.motors() })
  void qc.invalidateQueries({ queryKey: linkKeys.armPreflightsRoot() })
}

/** Preflight reads live joint positions; invalidate after any move that can clear a limit violation. */
function invalidateArmPreflights(qc: QueryClient) {
  void qc.invalidateQueries({ queryKey: linkKeys.armPreflightsRoot() })
}

export function invalidateMotorDetail(qc: QueryClient, id: number) {
  void qc.invalidateQueries({ queryKey: linkKeys.motor(id) })
}

export function useDiscoverMutation() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: api.discoverMotors,
    onSuccess: () => invalidateAfterConfigOrMotorListChange(qc),
  })
}

export function useAssignMotorMutation() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: ({
      id,
      section,
      joint,
    }: {
      id: number
      section: string
      joint: string
    }) => api.assignMotor(id, section, joint),
    onSuccess: () => invalidateAfterConfigOrMotorListChange(qc),
  })
}

export function useUnassignMotorMutation() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: (id: number) => api.unassignMotor(id),
    onSuccess: () => invalidateAfterConfigOrMotorListChange(qc),
  })
}

export function useHomeArmMutation() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: ({ side, override }: { side: string; override?: boolean }) =>
      api.homeArm(side, override ?? false),
    onSuccess: (_data, { side }) => {
      void qc.invalidateQueries({ queryKey: linkKeys.armPreflight(side) })
      void qc.invalidateQueries({ queryKey: linkKeys.arms() })
      void qc.invalidateQueries({ queryKey: linkKeys.motors() })
    },
  })
}

export function useEnableArmMutation() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: (side: string) => api.enableArm(side),
    onSuccess: () => {
      void qc.invalidateQueries({ queryKey: linkKeys.arms() })
      void qc.invalidateQueries({ queryKey: linkKeys.motors() })
    },
  })
}

export function useDisableArmMutation() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: (side: string) => api.disableArm(side),
    onSuccess: () => {
      void qc.invalidateQueries({ queryKey: linkKeys.arms() })
      void qc.invalidateQueries({ queryKey: linkKeys.motors() })
    },
  })
}

export function useSetArmPoseMutation() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: ({ side, pose }: { side: string; pose: api.PoseRequest }) =>
      api.setArmPose(side, pose),
    onSuccess: () => {
      void qc.invalidateQueries({ queryKey: linkKeys.arms() })
      void qc.invalidateQueries({ queryKey: linkKeys.motors() })
      invalidateArmPreflights(qc)
    },
  })
}

export function useUpdateJointLimitsMutation() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: ({
      section,
      joint,
      minRad,
      maxRad,
    }: {
      section: string
      joint: string
      minRad: number
      maxRad: number
    }) => api.updateJointLimits(section, joint, minRad, maxRad),
    onSuccess: () => invalidateAfterJointYamlChange(qc),
  })
}

export function useUpdateJointHomeMutation() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: ({
      section,
      joint,
      homeRad,
      setCurrent,
    }: {
      section: string
      joint: string
      homeRad?: number
      setCurrent?: boolean
    }) => api.updateJointHome(section, joint, homeRad, setCurrent),
    onSuccess: () => invalidateAfterJointYamlChange(qc),
  })
}

export function useZeroReframeHomeMutation() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: ({ section, joint }: { section: string; joint: string }) =>
      api.zeroReframeHome(section, joint),
    onSuccess: () => invalidateAfterJointYamlChange(qc),
  })
}

export function useEnableMotorMutation() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: (id: number) => api.enableMotor(id),
    onSuccess: (_d, id) => invalidateMotorDetail(qc, id),
  })
}

export function useDisableMotorMutation() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: (id: number) => api.disableMotor(id),
    onSuccess: (_d, id) => invalidateMotorDetail(qc, id),
  })
}

export function useZeroMotorMutation() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: (id: number) => api.zeroMotor(id),
    onSuccess: (_d, id) => {
      invalidateMotorDetail(qc, id)
      void qc.invalidateQueries({ queryKey: linkKeys.motors() })
      invalidateArmPreflights(qc)
    },
  })
}

export function useMoveMotorMutation() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: ({
      id,
      position_rad,
      kp,
      kd,
    }: {
      id: number
      position_rad: number
      kp?: number
      kd?: number
    }) => api.moveMotor(id, position_rad, kp, kd),
    onSuccess: (_d, vars) => {
      invalidateMotorDetail(qc, vars.id)
      invalidateArmPreflights(qc)
    },
  })
}

export function useControlMotorMutation() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: ({
      id,
      position,
      velocity,
      kp,
      kd,
      torque,
    }: {
      id: number
      position: number
      velocity: number
      kp: number
      kd: number
      torque: number
    }) => api.controlMotor(id, position, velocity, kp, kd, torque),
    onSuccess: (_d, vars) => {
      invalidateMotorDetail(qc, vars.id)
      invalidateArmPreflights(qc)
    },
  })
}

export function useSpinMotorMutation() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: ({
      id,
      velocity_rads,
      kd,
    }: {
      id: number
      velocity_rads: number
      kd?: number
    }) => api.spinMotor(id, velocity_rads, kd),
    onSuccess: (_d, vars) => {
      invalidateMotorDetail(qc, vars.id)
      invalidateArmPreflights(qc)
    },
  })
}

export function useTorqueMotorMutation() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: ({ id, torque_nm }: { id: number; torque_nm: number }) =>
      api.torqueMotor(id, torque_nm),
    onSuccess: (_d, vars) => {
      invalidateMotorDetail(qc, vars.id)
      invalidateArmPreflights(qc)
    },
  })
}

export function useJogMotorMutation() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: ({
      id,
      delta_deg,
      kp,
      kd,
    }: {
      id: number
      delta_deg: number
      kp?: number
      kd?: number
    }) => api.jogMotor(id, delta_deg, kp, kd),
    onSuccess: (_d, vars) => {
      invalidateMotorDetail(qc, vars.id)
      invalidateArmPreflights(qc)
    },
  })
}

export function useStopMotorMutation() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: (id: number) => api.stopMotor(id),
    onSuccess: (_d, id) => invalidateMotorDetail(qc, id),
  })
}

export function useEstopMutation() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: api.estopAll,
    onSuccess: () => {
      void qc.invalidateQueries({ queryKey: linkKeys.motors() })
    },
  })
}

export function useRunSequenceMutation() {
  const qc = useQueryClient()
  return useMutation({
    mutationFn: (name: string) => api.runSequence(name),
    onSuccess: () => {
      void qc.invalidateQueries({ queryKey: linkKeys.armPreflightsRoot() })
      void qc.invalidateQueries({ queryKey: linkKeys.arms() })
      void qc.invalidateQueries({ queryKey: linkKeys.motors() })
    },
  })
}
