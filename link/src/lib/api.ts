export interface MotorInfo {
  can_id: number
  joint_name: string
  actuator_type: string
  limits: [number, number]
  online: boolean
}

export interface MotorDetail extends MotorInfo {
  angle_rad: number
  velocity_rads: number
  torque_nm: number
  temperature_c: number
  mode: string
  faults: string[]
}

export interface CommandResponse {
  success: boolean
  error?: string
  angle_rad?: number
  velocity_rads?: number
  torque_nm?: number
}

export interface RobotConfig {
  bus: {
    port: string
    baud: number
    can_bitrate: number
    host_id: number
  }
  actuators: Record<string, {
    max_torque: number
    max_speed: number
    max_current: number
    gear_ratio: number
    weight_kg: number
    voltage_nominal: number
  }>
  arm_left?: Record<string, unknown>
  arm_right?: Record<string, unknown>
  waist?: Record<string, unknown>
  torso?: {
    frame: string
    dimensions_mm: [number, number, number]
  }
}

export interface ServerStatus {
  uptime_secs: number
  mode: string
  motor_count: number
  transport_type: string
}

export interface ArmInfo {
  side: string
  joints: ArmJointInfo[]
}

export interface ArmJointInfo {
  name: string
  can_id: number | null
  actuator: string
  limits: [number, number]
  home_rad: number
  angle_rad?: number
  velocity_rads?: number
  torque_nm?: number
  online: boolean
}

export interface PoseRequest {
  joints: Record<string, number>
  kp?: number
  kd?: number
}

export interface LogEntry {
  timestamp_ms: number
  level: string
  target: string
  message: string
}

// -- Homing types --

export interface JointHomingResult {
  joint_name: string
  status: string
  start_position_rad: number
  end_position_rad: number
  home_target_rad: number
  error_rad: number
  stall_backoffs: number
  duration_ms: number
}

export interface HomeResponse {
  success: boolean
  error?: string
  joints: JointHomingResult[]
  preflight?: PreflightResult
}

export interface PreflightViolation {
  exceeded_by_rad: number
  exceeded_by_deg: number
  which_limit: string
  suggested_fix: string
  multiturn: boolean
}

export interface PreflightJoint {
  joint_name: string
  current_rad: number
  current_deg: number
  limit_min_rad: number
  limit_max_rad: number
  limit_min_deg: number
  limit_max_deg: number
  home_rad: number
  violation: PreflightViolation | null
  online: boolean
}

export interface PreflightResult {
  pass: boolean
  joints: PreflightJoint[]
}

export interface JointHomeStatus {
  joint_name: string
  home_rad: number
  current_rad: number
  error_rad: number
  at_home: boolean
  limits: [number, number]
}

const BASE = '/api'

async function fetchJson<T>(path: string, init?: RequestInit): Promise<T> {
  const res = await fetch(`${BASE}${path}`, init)
  if (!res.ok) {
    const text = await res.text().catch(() => res.statusText)
    throw new Error(`API ${res.status}: ${text}`)
  }
  return res.json()
}

export function getConfig(): Promise<RobotConfig> {
  return fetchJson('/config')
}

export function getStatus(): Promise<ServerStatus> {
  return fetchJson('/status')
}

export function getMotors(): Promise<MotorInfo[]> {
  return fetchJson('/motors')
}

export function getMotor(id: number): Promise<MotorDetail> {
  return fetchJson(`/motors/${id}`)
}

export function enableMotor(id: number): Promise<CommandResponse> {
  return fetchJson(`/motors/${id}/enable`, { method: 'POST' })
}

export function disableMotor(id: number): Promise<CommandResponse> {
  return fetchJson(`/motors/${id}/disable`, { method: 'POST' })
}

export function zeroMotor(id: number): Promise<CommandResponse> {
  return fetchJson(`/motors/${id}/zero`, { method: 'POST' })
}

export function moveMotor(
  id: number,
  position_rad: number,
  kp?: number,
  kd?: number,
): Promise<CommandResponse> {
  return fetchJson(`/motors/${id}/move`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ position_rad, kp, kd }),
  })
}

export function controlMotor(
  id: number,
  position: number,
  velocity: number,
  kp: number,
  kd: number,
  torque: number,
): Promise<CommandResponse> {
  return fetchJson(`/motors/${id}/control`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ position, velocity, kp, kd, torque }),
  })
}

export function getArms(): Promise<ArmInfo[]> {
  return fetchJson('/arms')
}

export function enableArm(side: string): Promise<CommandResponse> {
  return fetchJson(`/arms/${side}/enable`, { method: 'POST' })
}

export function disableArm(side: string): Promise<CommandResponse> {
  return fetchJson(`/arms/${side}/disable`, { method: 'POST' })
}

export function homeArm(side: string, overridePreflight = false): Promise<HomeResponse> {
  return fetchJson(`/arms/${side}/home`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ override_preflight: overridePreflight }),
  })
}

export function getArmPreflight(side: string): Promise<PreflightResult> {
  return fetchJson(`/arms/${side}/preflight`)
}

export function getHomingStatus(side: string): Promise<JointHomeStatus[]> {
  return fetchJson(`/arms/${side}/home-status`)
}

export function updateJointLimits(
  section: string,
  joint: string,
  minRad: number,
  maxRad: number,
): Promise<CommandResponse> {
  return fetchJson(`/joints/${section}/${joint}/limits`, {
    method: 'PUT',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ min_rad: minRad, max_rad: maxRad }),
  })
}

export function updateJointHome(
  section: string,
  joint: string,
  homeRad?: number,
  setCurrent?: boolean,
): Promise<CommandResponse> {
  return fetchJson(`/joints/${section}/${joint}/home`, {
    method: 'PUT',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(
      setCurrent ? { set_current: true } : { home_rad: homeRad },
    ),
  })
}

export function getLogs(limit = 200): Promise<LogEntry[]> {
  return fetchJson(`/logs?limit=${limit}`)
}

export function setArmPose(side: string, pose: PoseRequest): Promise<CommandResponse> {
  return fetchJson(`/arms/${side}/pose`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(pose),
  })
}

export interface DiscoverResult {
  discovered: number[]
  removed: number[]
  total: number
}

export function discoverMotors(): Promise<DiscoverResult> {
  return fetchJson('/discover', { method: 'POST' })
}

export interface SequenceInfo {
  name: string
  description: string
}

export function spinMotor(id: number, velocity_rads: number, kd?: number): Promise<CommandResponse> {
  return fetchJson(`/motors/${id}/spin`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ velocity_rads, kd }),
  })
}

export function torqueMotor(id: number, torque_nm: number): Promise<CommandResponse> {
  return fetchJson(`/motors/${id}/torque`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ torque_nm }),
  })
}

export function jogMotor(id: number, delta_deg: number, kp?: number, kd?: number): Promise<CommandResponse> {
  return fetchJson(`/motors/${id}/jog`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ delta_deg, kp, kd }),
  })
}

export function stopMotor(id: number): Promise<CommandResponse> {
  return fetchJson(`/motors/${id}/stop`, { method: 'POST' })
}

export function estopAll(): Promise<CommandResponse> {
  return fetchJson('/estop', { method: 'POST' })
}

export function getSequences(): Promise<SequenceInfo[]> {
  return fetchJson('/sequences')
}

export function runSequence(name: string): Promise<CommandResponse> {
  return fetchJson(`/sequences/${name}/run`, { method: 'POST' })
}

export interface JointSlot {
  section: string
  joint: string
  can_id: number | null
  display_name: string
}

export function getJointSlots(): Promise<JointSlot[]> {
  return fetchJson('/joint-slots')
}

export function assignMotor(id: number, section: string, joint: string): Promise<CommandResponse> {
  return fetchJson(`/motors/${id}/assign`, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ section, joint }),
  })
}

export function unassignMotor(id: number): Promise<CommandResponse> {
  return fetchJson(`/motors/${id}/unassign`, { method: 'POST' })
}
