import { useState } from 'react'
import { useQueryClient } from '@tanstack/react-query'
import { Button } from '@/components/ui/button'
import { Badge } from '@/components/ui/badge'
import { linkKeys } from '@/lib/queries'
import { getArmPreflight, type PreflightJoint, type PreflightResult } from '@/lib/api'
import { toast } from 'sonner'
import { LuInfo, LuRotateCw, LuX } from 'react-icons/lu'

interface PreflightSoftNoticeProps {
  side: string
  preflight: PreflightResult
  onDismiss?: () => void
}

export function PreflightSoftNotice({ side, preflight, onDismiss }: PreflightSoftNoticeProps) {
  const qc = useQueryClient()
  const [checking, setChecking] = useState(false)

  const warned = preflight.joints.filter((j) => j.soft_warning != null)
  if (!preflight.pass || warned.length === 0) return null

  const handleRecheck = async () => {
    setChecking(true)
    try {
      const result = await qc.fetchQuery({
        queryKey: linkKeys.armPreflight(side),
        queryFn: () => getArmPreflight(side),
      })
      if (!result.joints.some((j) => j.soft_warning != null)) {
        toast.success('No joints in tolerance band')
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

  return (
    <div className="rounded-lg border border-amber-500/40 bg-amber-500/10 p-4 mb-4">
      <div className="flex items-start gap-3">
        <LuInfo className="size-5 text-amber-400 mt-0.5 shrink-0" />
        <div className="flex-1 min-w-0">
          <div className="flex items-start justify-between gap-2">
            <div>
              <h3 className="text-sm font-semibold text-amber-200">
                Near limit (within pre-flight tolerance) — {side.charAt(0).toUpperCase() + side.slice(1)} Arm
              </h3>
              <p className="text-xs text-amber-200/75 mt-1">
                {warned.length} joint{warned.length !== 1 ? 's are' : ' is'} slightly outside the configured
                limits but inside the mechanical slack used for homing. Homing is{' '}
                <span className="font-medium text-amber-100">not blocked</span>.
              </p>
            </div>
            {onDismiss && (
              <Button
                variant="ghost"
                size="icon"
                className="size-7 shrink-0 text-amber-400/80 hover:text-amber-300"
                onClick={onDismiss}
                aria-label="Dismiss notice"
              >
                <LuX className="size-4" />
              </Button>
            )}
          </div>

          <div className="mt-3 space-y-2">
            {warned.map((joint) => (
              <JointSoftRow key={joint.joint_name} joint={joint} />
            ))}
          </div>

          <div className="mt-3 flex gap-2 flex-wrap">
            <Button
              variant="outline"
              size="sm"
              onClick={() => void handleRecheck()}
              disabled={checking}
              className="gap-1.5 border-amber-500/30 text-amber-100 hover:bg-amber-500/10"
            >
              <LuRotateCw className={`size-3.5 ${checking ? 'animate-spin' : ''}`} />
              Re-check
            </Button>
          </div>
        </div>
      </div>
    </div>
  )
}

function JointSoftRow({ joint }: { joint: PreflightJoint }) {
  const w = joint.soft_warning!
  return (
    <div className="flex items-start gap-3 rounded-md border border-amber-500/20 bg-amber-500/5 px-3 py-2">
      <div className="flex-1 min-w-0">
        <div className="flex items-center gap-2 flex-wrap">
          <span className="text-sm font-medium text-foreground">{formatJointName(joint.joint_name)}</span>
          <Badge className="text-[10px] bg-amber-500/15 text-amber-300 border-amber-500/25">
            {w.which_limit} limit · {w.nominal_exceeded_by_deg.toFixed(1)}° outside nominal
          </Badge>
        </div>
        <div className="mt-1 text-xs text-muted-foreground">
          Current <strong>{joint.current_deg.toFixed(1)}°</strong>
          <span className="mx-2">|</span>
          Nominal limit {w.which_limit === 'min' ? joint.limit_min_deg.toFixed(1) : joint.limit_max_deg.toFixed(1)}°
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
