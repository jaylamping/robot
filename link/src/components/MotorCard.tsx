import { useState } from 'react'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import {
  DropdownMenu,
  DropdownMenuContent,
  DropdownMenuGroup,
  DropdownMenuItem,
  DropdownMenuLabel,
  DropdownMenuSeparator,
  DropdownMenuTrigger,
} from '@/components/ui/dropdown-menu'
import type { MotorInfo, JointSlot } from '@/lib/api'
import { useAssignMotorMutation, useUnassignMotorMutation } from '@/lib/mutations/robot'
import { useTelemetryStore, type MotorSnapshot } from '@/stores/telemetry'
import { LuPencil, LuX } from 'react-icons/lu'

interface MotorCardProps {
  motor: MotorInfo
  jointSlots?: JointSlot[]
  onClick?: () => void
}

const SECTION_LABELS: Record<string, string> = {
  arm_left: 'Left Arm',
  arm_right: 'Right Arm',
  waist: 'Waist',
}

export function MotorCard({ motor, jointSlots, onClick }: MotorCardProps) {
  const live = useTelemetryStore((s) => s.motors[motor.can_id]) as MotorSnapshot | undefined
  const [assigning, setAssigning] = useState(false)
  const assignMut = useAssignMotorMutation()
  const unassignMut = useUnassignMotorMutation()

  const isOnline = live?.online ?? motor.online
  const hasFaults = (live?.faults?.length ?? 0) > 0
  const highTemp = (live?.temperature_c ?? 0) > 60

  const isUnassigned = motor.joint_name.startsWith('motor_')

  const statusVariant = hasFaults
    ? 'destructive'
    : isOnline
      ? 'default'
      : 'secondary'

  const statusLabel = hasFaults
    ? 'Fault'
    : highTemp
      ? 'Hot'
      : isOnline
        ? 'Online'
        : 'Offline'

  const dotColor = hasFaults
    ? 'bg-red-500'
    : highTemp
      ? 'bg-amber-500'
      : isOnline
        ? 'bg-emerald-500 animate-pulse'
        : 'bg-muted-foreground'

  const handleAssign = async (section: string, joint: string) => {
    setAssigning(true)
    try {
      const res = await assignMut.mutateAsync({
        id: motor.can_id,
        section,
        joint,
      })
      if (!res.success) {
        console.error('Assign failed:', res.error)
      }
    } catch (e) {
      console.error('Assign error:', e)
    } finally {
      setAssigning(false)
    }
  }

  const handleUnassign = async (e: React.MouseEvent) => {
    e.stopPropagation()
    setAssigning(true)
    try {
      const res = await unassignMut.mutateAsync(motor.can_id)
      if (!res.success) {
        console.error('Unassign failed:', res.error)
      }
    } catch (e) {
      console.error('Unassign error:', e)
    } finally {
      setAssigning(false)
    }
  }

  const slotsBySection = jointSlots ? groupSlotsBySection(jointSlots) : {}

  return (
    <Card
      className="cursor-pointer transition-colors hover:bg-accent/50"
      size="sm"
      onClick={onClick}
    >
      <CardHeader>
        <div className="flex items-center justify-between gap-2">
          <div className="flex items-center gap-1.5 min-w-0">
            <CardTitle className="truncate">{formatJointName(motor.joint_name)}</CardTitle>
            {jointSlots && (
              <DropdownMenu>
                <DropdownMenuTrigger
                  className="inline-flex shrink-0 items-center justify-center rounded-md p-1 text-muted-foreground hover:bg-accent hover:text-foreground transition-colors"
                  onClick={(e) => e.stopPropagation()}
                  disabled={assigning}
                >
                  <LuPencil className="size-3" />
                </DropdownMenuTrigger>
                <DropdownMenuContent align="start" side="bottom" sideOffset={4}>
                  {Object.entries(slotsBySection).map(([section, slots], si) => (
                    <DropdownMenuGroup key={section}>
                      {si > 0 && <DropdownMenuSeparator />}
                      <DropdownMenuLabel>{SECTION_LABELS[section] ?? section}</DropdownMenuLabel>
                      {slots.map((slot) => {
                        const isSelf = slot.can_id === motor.can_id
                        const isOccupied = slot.can_id !== null && !isSelf
                        return (
                          <DropdownMenuItem
                            key={`${slot.section}-${slot.joint}`}
                            onClick={(e) => {
                              e.stopPropagation()
                              if (!isSelf && !assigning) {
                                handleAssign(slot.section, slot.joint)
                              }
                            }}
                            className={isSelf ? 'text-emerald-400' : isOccupied ? 'opacity-50' : ''}
                          >
                            <span>{formatJointName(slot.joint)}</span>
                            {isSelf && (
                              <span className="ml-auto text-[10px] text-emerald-400/70">current</span>
                            )}
                            {isOccupied && (
                              <span className="ml-auto text-[10px] text-muted-foreground">ID {slot.can_id}</span>
                            )}
                          </DropdownMenuItem>
                        )
                      })}
                    </DropdownMenuGroup>
                  ))}
                  {!isUnassigned && (
                    <>
                      <DropdownMenuSeparator />
                      <DropdownMenuItem
                        onClick={(e) => {
                          e.stopPropagation()
                          handleUnassign(e as unknown as React.MouseEvent)
                        }}
                        className="text-destructive"
                      >
                        <LuX className="size-3" />
                        Unassign
                      </DropdownMenuItem>
                    </>
                  )}
                </DropdownMenuContent>
              </DropdownMenu>
            )}
          </div>
          <Badge variant={statusVariant} className="gap-1.5 shrink-0">
            <span className={`inline-block size-1.5 rounded-full ${dotColor}`} />
            {statusLabel}
          </Badge>
        </div>
      </CardHeader>
      <CardContent>
        <div className="space-y-1.5 text-xs text-muted-foreground">
          <Row label="CAN ID" value={<span className="font-mono">{motor.can_id}</span>} />
          <Row label="Actuator" value={<span className="uppercase">{motor.actuator_type}</span>} />
          {live?.mode && (
            <Row label="Mode" value={live.mode} />
          )}
          {live && isOnline ? (
            <>
              <Row label="Position" value={<span className="font-mono">{((live.angle_rad * 180) / Math.PI).toFixed(1)}°</span>} />
              <Row label="Velocity" value={<span className="font-mono">{live.velocity_rads.toFixed(2)} rad/s</span>} />
              <Row label="Torque" value={<span className="font-mono">{live.torque_nm.toFixed(2)} N·m</span>} />
              <Row
                label="Temp"
                value={
                  <span className={`font-mono ${highTemp ? 'text-amber-400' : ''}`}>
                    {live.temperature_c.toFixed(1)} °C
                  </span>
                }
              />
            </>
          ) : (
            <Row
              label="Limits"
              value={
                <span className="font-mono">
                  {((motor.limits[0] * 180) / Math.PI).toFixed(0)}° / {((motor.limits[1] * 180) / Math.PI).toFixed(0)}°
                </span>
              }
            />
          )}
        </div>

        {hasFaults && live && (
          <div className="mt-2 border-t pt-2">
            {live.faults.map((f, i) => (
              <p key={i} className="truncate font-mono text-xs text-destructive">{f}</p>
            ))}
          </div>
        )}
      </CardContent>
    </Card>
  )
}

function Row({ label, value }: { label: string; value: React.ReactNode }) {
  return (
    <div className="flex justify-between">
      <span>{label}</span>
      <span className="text-foreground">{value}</span>
    </div>
  )
}

function formatJointName(name: string): string {
  return name
    .split('_')
    .map((w) => w.charAt(0).toUpperCase() + w.slice(1))
    .join(' ')
}

function groupSlotsBySection(slots: JointSlot[]): Record<string, JointSlot[]> {
  const groups: Record<string, JointSlot[]> = {}
  for (const slot of slots) {
    if (!groups[slot.section]) groups[slot.section] = []
    groups[slot.section].push(slot)
  }
  return groups
}
