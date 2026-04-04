import { useState } from 'react'
import { useQueryClient } from '@tanstack/react-query'
import { Button } from '@/components/ui/button'
import { Badge } from '@/components/ui/badge'
import { useHomeArmMutation, useZeroMotorMutation } from '@/lib/mutations/robot'
import { linkKeys, useRobotArms } from '@/lib/queries'
import { getArmPreflight, type PreflightJoint, type PreflightResult } from '@/lib/api'
import { toast } from 'sonner'
import { LuTriangleAlert, LuRotateCw, LuShieldAlert, LuRefreshCw } from 'react-icons/lu'

interface PreflightAlertProps {
  side: string
  preflight: PreflightResult
  onDismiss?: () => void
}

export function PreflightAlert({ side, preflight, onDismiss }: PreflightAlertProps) {
  const qc = useQueryClient()
  const homeMut = useHomeArmMutation()
  const zeroMut = useZeroMotorMutation()
  const armsQ = useRobotArms()
  const [overriding, setOverriding] = useState(false)
  const [checking, setChecking] = useState(false)
  const [zeroingJoint, setZeroingJoint] = useState<string | null>(null)

  if (preflight.pass) return null

  const violations = preflight.joints.filter((j) => j.violation)
  const hasMultiturn = violations.some((j) => j.violation?.multiturn)

  const armJoints = armsQ.data
    ?.find((a) => a.side === side)
    ?.joints

  const getCanId = (jointName: string): number | null => {
    return armJoints?.find((j) => j.name === jointName)?.can_id ?? null
  }

  const handleRecheck = async () => {
    setChecking(true)
    try {
      const result = await qc.fetchQuery({
        queryKey: linkKeys.armPreflight(side),
        queryFn: () => getArmPreflight(side),
      })
      if (result.pass) {
        toast.success('Pre-flight check passed')
        onDismiss?.()
      }
    } catch (e) {
      toast.error('Pre-flight check failed', {
        description: e instanceof Error ? e.message : String(e),
      })
    } finally {
      setChecking(false)
    }
  }

  const handleOverride = async () => {
    if (!confirm('This will attempt to home joints despite limit violations. Only proceed if you are physically present and can E-STOP. Continue?')) {
      return
    }
    setOverriding(true)
    try {
      const result = await homeMut.mutateAsync({ side, override: true })
      if (result.success) {
        toast.success(`${side} arm homed with override`, {
          description: result.error ?? undefined,
        })
        onDismiss?.()
      } else {
        toast.error(`${side} arm homing failed`, { description: result.error })
      }
      await qc.refetchQueries({ queryKey: linkKeys.armPreflight(side) })
    } catch (e) {
      toast.error('Override homing failed', {
        description: e instanceof Error ? e.message : String(e),
      })
    } finally {
      setOverriding(false)
    }
  }

  const handleZeroEncoder = async (joint: PreflightJoint) => {
    const canId = getCanId(joint.joint_name)
    if (canId == null) {
      toast.error('Cannot zero encoder', { description: 'No CAN ID assigned to this joint' })
      return
    }
    if (!confirm(`This will set CAN ID ${canId} (${formatJointName(joint.joint_name)}) encoder to zero at its current physical position. The joint should be near its intended zero position. Continue?`)) {
      return
    }
    setZeroingJoint(joint.joint_name)
    try {
      const result = await zeroMut.mutateAsync(canId)
      if (result.success) {
        toast.success(`Encoder zeroed for ${formatJointName(joint.joint_name)}`)
        await qc.refetchQueries({ queryKey: linkKeys.armPreflight(side) })
      } else {
        toast.error('Zero failed', { description: result.error })
      }
    } catch (e) {
      toast.error('Zero encoder failed', {
        description: e instanceof Error ? e.message : String(e),
      })
    } finally {
      setZeroingJoint(null)
    }
  }

  return (
    <div className="rounded-lg border border-red-500/50 bg-red-500/10 p-4 mb-4">
      <div className="flex items-start gap-3">
        <LuTriangleAlert className="size-5 text-red-400 mt-0.5 shrink-0" />
        <div className="flex-1 min-w-0">
          <h3 className="text-sm font-semibold text-red-400">
            Joint Limit Violation — {side.charAt(0).toUpperCase() + side.slice(1)} Arm
          </h3>
          <p className="text-xs text-red-300/80 mt-1">
            {violations.length} joint{violations.length !== 1 ? 's' : ''} outside configured limits.
            Homing is blocked until resolved.
          </p>

          <div className="mt-3 space-y-2">
            {violations.map((joint) => {
              const isMultiturn = joint.violation!.multiturn
              const canId = getCanId(joint.joint_name)
              return (
                <div
                  key={joint.joint_name}
                  className="flex items-start gap-3 rounded-md border border-red-500/20 bg-red-500/5 px-3 py-2"
                >
                  <LuShieldAlert className="size-4 text-red-400 mt-0.5 shrink-0" />
                  <div className="flex-1 min-w-0">
                    <div className="flex items-center gap-2 flex-wrap">
                      <span className="text-sm font-medium text-foreground">
                        {formatJointName(joint.joint_name)}
                      </span>
                      {isMultiturn ? (
                        <Badge className="text-[10px] bg-amber-500/20 text-amber-400 border-amber-500/30">
                          multi-turn
                        </Badge>
                      ) : (
                        <Badge variant="destructive" className="text-[10px]">
                          {joint.violation!.which_limit} limit
                        </Badge>
                      )}
                    </div>
                    <div className="mt-1 text-xs text-muted-foreground">
                      {isMultiturn ? (
                        <>
                          <span>
                            Joint angle (canonical):{' '}
                            <strong>{formatDisplayDeg(joint.current_deg)}</strong>
                          </span>
                          <span className="mx-2">|</span>
                          <span>~{formatTurnsAccumulated(joint.current_rad)} turns (est.)</span>
                        </>
                      ) : (
                        <>
                          <span>Current: <strong>{joint.current_deg.toFixed(1)}°</strong></span>
                          <span className="mx-2">|</span>
                          <span>
                            Limit: {joint.violation!.which_limit === 'min'
                              ? `${joint.limit_min_deg.toFixed(1)}°`
                              : `${joint.limit_max_deg.toFixed(1)}°`}
                          </span>
                          <span className="mx-2">|</span>
                          <span className="text-red-400">
                            {joint.violation!.exceeded_by_deg.toFixed(1)}° past
                          </span>
                        </>
                      )}
                    </div>
                    <p className="mt-1 text-xs text-amber-400/90">
                      {joint.violation!.suggested_fix}
                    </p>
                    {isMultiturn && canId != null && (
                      <Button
                        variant="outline"
                        size="sm"
                        onClick={() => void handleZeroEncoder(joint)}
                        disabled={zeroingJoint != null}
                        className="mt-2 gap-1.5 h-6 text-[11px]"
                      >
                        <LuRefreshCw className={`size-3 ${zeroingJoint === joint.joint_name ? 'animate-spin' : ''}`} />
                        {zeroingJoint === joint.joint_name ? 'Zeroing...' : 'Zero Encoder'}
                      </Button>
                    )}
                  </div>
                </div>
              )
            })}
          </div>

          <div className="mt-3 flex gap-2 flex-wrap">
            <Button
              variant="outline"
              size="sm"
              onClick={() => void handleRecheck()}
              disabled={checking}
              className="gap-1.5"
            >
              <LuRotateCw className={`size-3.5 ${checking ? 'animate-spin' : ''}`} />
              Re-check
            </Button>
            {hasMultiturn && (
              <Button
                variant="outline"
                size="sm"
                onClick={() => void handleZeroAll()}
                disabled={zeroingJoint != null}
                className="gap-1.5"
              >
                <LuRefreshCw className="size-3.5" />
                Zero All Encoders
              </Button>
            )}
            <Button
              variant="destructive"
              size="sm"
              onClick={() => void handleOverride()}
              disabled={overriding}
              className="gap-1.5"
            >
              <LuTriangleAlert className="size-3.5" />
              {overriding ? 'Homing...' : 'Override & Home'}
            </Button>
          </div>
        </div>
      </div>
    </div>
  )

  async function handleZeroAll() {
    const multiturnJoints = violations.filter((j) => j.violation?.multiturn)
    const canIds = multiturnJoints
      .map((j) => ({ joint: j, canId: getCanId(j.joint_name) }))
      .filter((x): x is { joint: PreflightJoint; canId: number } => x.canId != null)

    if (canIds.length === 0) {
      toast.error('No CAN IDs assigned to multi-turn joints')
      return
    }

    if (!confirm(`This will zero ${canIds.length} encoder(s) at their current physical positions. All joints should be near their intended zero positions. Continue?`)) {
      return
    }

    setZeroingJoint('__all__')
    let errors = 0
    for (const { joint, canId } of canIds) {
      try {
        const result = await zeroMut.mutateAsync(canId)
        if (!result.success) {
          toast.error(`Zero failed for ${formatJointName(joint.joint_name)}`, { description: result.error })
          errors++
        }
      } catch (e) {
        toast.error(`Zero failed for ${formatJointName(joint.joint_name)}`, {
          description: e instanceof Error ? e.message : String(e),
        })
        errors++
      }
    }
    setZeroingJoint(null)
    if (errors === 0) {
      toast.success(`Zeroed ${canIds.length} encoder(s)`)
    }
    await qc.refetchQueries({ queryKey: linkKeys.armPreflight(side) })
  }
}

function formatJointName(name: string): string {
  return name
    .split('_')
    .map((w) => w.charAt(0).toUpperCase() + w.slice(1))
    .join(' ')
}

/** Avoid scientific notation / overflow garbage if API ever sends corrupt floats. */
function formatDisplayDeg(deg: number): string {
  if (!Number.isFinite(deg) || Math.abs(deg) > 1e7) {
    return '—'
  }
  return `${deg.toFixed(1)}°`
}

function formatTurnsAccumulated(currentRad: number): string {
  if (!Number.isFinite(currentRad)) {
    return '—'
  }
  const turns = Math.abs(currentRad / (2 * Math.PI))
  if (turns > 1e6) {
    return '—'
  }
  return turns.toFixed(1)
}
