import { useState } from 'react'
import { useQueryClient } from '@tanstack/react-query'
import { Button } from '@/components/ui/button'
import { Badge } from '@/components/ui/badge'
import { useHomeArmMutation } from '@/lib/mutations/robot'
import { linkKeys } from '@/lib/queries'
import type { PreflightResult } from '@/lib/api'
import { toast } from 'sonner'
import { LuTriangleAlert, LuRotateCw, LuShieldAlert } from 'react-icons/lu'

interface PreflightAlertProps {
  side: string
  preflight: PreflightResult
  onDismiss?: () => void
}

export function PreflightAlert({ side, preflight, onDismiss }: PreflightAlertProps) {
  const qc = useQueryClient()
  const homeMut = useHomeArmMutation()
  const [overriding, setOverriding] = useState(false)
  const [checking, setChecking] = useState(false)

  if (preflight.pass) return null

  const violations = preflight.joints.filter((j) => j.violation)

  const handleRecheck = async () => {
    setChecking(true)
    try {
      await qc.refetchQueries({ queryKey: linkKeys.armPreflight(side) })
      const result = qc.getQueryData<PreflightResult>(linkKeys.armPreflight(side))
      if (result?.pass) {
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
            {violations.map((joint) => (
              <div
                key={joint.joint_name}
                className="flex items-start gap-3 rounded-md border border-red-500/20 bg-red-500/5 px-3 py-2"
              >
                <LuShieldAlert className="size-4 text-red-400 mt-0.5 shrink-0" />
                <div className="flex-1 min-w-0">
                  <div className="flex items-center gap-2">
                    <span className="text-sm font-medium text-foreground">
                      {formatJointName(joint.joint_name)}
                    </span>
                    <Badge variant="destructive" className="text-[10px]">
                      {joint.violation!.which_limit} limit
                    </Badge>
                  </div>
                  <div className="mt-1 text-xs text-muted-foreground">
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
                  </div>
                  <p className="mt-1 text-xs text-amber-400/90">
                    {joint.violation!.suggested_fix}
                  </p>
                </div>
              </div>
            ))}
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
}

function formatJointName(name: string): string {
  return name
    .split('_')
    .map((w) => w.charAt(0).toUpperCase() + w.slice(1))
    .join(' ')
}
