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

export function homeArm(side: string): Promise<CommandResponse> {
  return fetchJson(`/arms/${side}/home`, { method: 'POST' })
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
