import { useState } from 'react'
import { useQueryClient } from '@tanstack/react-query'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { useTelemetryStore } from '@/stores/telemetry'
import type { HomeResponse } from '@/lib/api'
import { PreflightAlert } from '@/components/PreflightAlert'
import { PreflightSoftNotice } from '@/components/PreflightSoftNotice'
import { linkKeys, useRobotArmPreflights } from '@/lib/queries'
import { useHomeArmMutation } from '@/lib/mutations/robot'
import { toast } from 'sonner'
import { LuHouse, LuCheck, LuCircleAlert, LuCircleMinus } from 'react-icons/lu'

interface HomingStatusCardProps {
  armSides: string[]
}

export function HomingStatusCard({ armSides }: HomingStatusCardProps) {
  const motors = useTelemetryStore((s) => s.motors)
  const qc = useQueryClient()
  const preflightQueries = useRobotArmPreflights(armSides)
  const homeMut = useHomeArmMutation()
  const [dismissedPreflight, setDismissedPreflight] = useState<Record<string, boolean>>({})
  const [dismissedSoftPreflight, setDismissedSoftPreflight] = useState<Record<string, boolean>>({})
  const [homeResults, setHomeResults] = useState<Record<string, HomeResponse>>({})
  const [homingSide, setHomingSide] = useState<string | null>(null)

  const motorList = Object.values(motors)
  const motorsWithHome = motorList.filter((m) => m.home_rad != null)
  const homedCount = motorsWithHome.filter((m) => m.at_home).length
  const awayCount = motorsWithHome.length - homedCount

  const handleHome = async (side: string) => {
    setHomingSide(side)
    try {
      const result = await homeMut.mutateAsync({ side, override: false })
      setHomeResults((prev) => ({ ...prev, [side]: result }))
      if (result.success) {
        setDismissedSoftPreflight((d) => ({ ...d, [side]: false }))
        toast.success(`${side} arm homed`, {
          description: result.error ?? `${result.joints.length} joints processed`,
        })
      } else if (result.preflight) {
        void qc.invalidateQueries({ queryKey: linkKeys.armPreflight(side) })
        setDismissedPreflight((d) => ({ ...d, [side]: false }))
        setDismissedSoftPreflight((d) => ({ ...d, [side]: false }))
        toast.error(`${side} arm: pre-flight failed`, { description: result.error })
      } else {
        toast.error(`${side} arm: homing failed`, { description: result.error })
      }
    } catch (e) {
      toast.error(`${side} arm: homing error`, {
        description: e instanceof Error ? e.message : String(e),
      })
    } finally {
      setHomingSide(null)
    }
  }

  return (
    <Card>
      <CardHeader>
        <div className="flex items-center justify-between">
          <CardTitle className="flex items-center gap-2 text-base">
            <LuHouse className="size-4" />
            Homing Status
          </CardTitle>
          <div className="flex gap-2">
            {awayCount === 0 && motorsWithHome.length > 0 ? (
              <Badge className="bg-emerald-500/10 text-emerald-400 border-emerald-500/20">
                <LuCheck className="size-3 mr-1" />
                All Homed
              </Badge>
            ) : awayCount > 0 ? (
              <Badge className="bg-amber-500/10 text-amber-400 border-amber-500/20">
                <LuCircleAlert className="size-3 mr-1" />
                {awayCount} away from home
              </Badge>
            ) : (
              <Badge variant="secondary">
                <LuCircleMinus className="size-3 mr-1" />
                No data
              </Badge>
            )}
          </div>
        </div>
      </CardHeader>
      <CardContent className="space-y-4">
        {armSides.map((side, i) => {
          const pq = preflightQueries[i]
          const pf = pq?.data
          const showPreflight = pf && !pf.pass && !dismissedPreflight[side]
          const showSoftPreflight =
            pf &&
            pf.pass &&
            pf.joints.some((j) => j.soft_warning != null) &&
            !dismissedSoftPreflight[side]
          return (
            <div key={side}>
              {showPreflight && (
                <PreflightAlert
                  side={side}
                  preflight={pf}
                  onDismiss={() => setDismissedPreflight((d) => ({ ...d, [side]: true }))}
                />
              )}
              {showSoftPreflight && (
                <PreflightSoftNotice
                  side={side}
                  preflight={pf}
                  onDismiss={() => setDismissedSoftPreflight((d) => ({ ...d, [side]: true }))}
                />
              )}

              <div className="flex items-center justify-between mb-2">
                <h4 className="text-sm font-medium capitalize">{side} Arm</h4>
                <Button
                  variant="outline"
                  size="sm"
                  onClick={() => void handleHome(side)}
                  disabled={homingSide !== null}
                  className="gap-1.5 h-7 text-xs"
                >
                  <LuHouse className="size-3" />
                  {homingSide === side ? 'Homing...' : 'Home'}
                </Button>
              </div>

              <div className="grid grid-cols-2 sm:grid-cols-4 gap-2">
                {motorsWithHome
                  .filter((m) => m.joint_name.startsWith(`${side}_`))
                  .map((m) => (
                    <div
                      key={m.can_id}
                      className="rounded-md border px-2.5 py-1.5 text-xs"
                    >
                      <div className="flex items-center gap-1.5 mb-0.5">
                        <span
                          className={`size-1.5 rounded-full ${
                            m.at_home
                              ? 'bg-emerald-400'
                              : m.online
                                ? 'bg-amber-400'
                                : 'bg-zinc-500'
                          }`}
                        />
                        <span className="font-medium truncate">
                          {formatJointName(m.joint_name.replace(`${side}_`, ''))}
                        </span>
                      </div>
                      <div className="text-muted-foreground font-mono">
                        {m.home_error_rad != null
                          ? `${(m.home_error_rad * (180 / Math.PI)).toFixed(1)}° err`
                          : 'N/A'}
                      </div>
                    </div>
                  ))}
              </div>

              {homeResults[side] && homeResults[side].joints.length > 0 && (
                <div className="mt-2 text-xs space-y-0.5">
                  {homeResults[side].joints.map((j) => (
                    <div
                      key={j.joint_name}
                      className="flex items-center gap-2 text-muted-foreground"
                    >
                      <StatusDot status={j.status} />
                      <span className="font-medium">{formatJointName(j.joint_name)}</span>
                      <span>{j.status.replace(/_/g, ' ')}</span>
                      <span className="ml-auto font-mono">{j.duration_ms}ms</span>
                    </div>
                  ))}
                </div>
              )}
            </div>
          )
        })}
      </CardContent>
    </Card>
  )
}

function StatusDot({ status }: { status: string }) {
  const color =
    status === 'already_home' || status === 'homed'
      ? 'bg-emerald-400'
      : status === 'stalled_but_homed'
        ? 'bg-amber-400'
        : status === 'error' || status === 'timed_out'
          ? 'bg-red-400'
          : 'bg-zinc-500'
  return <span className={`size-1.5 rounded-full ${color}`} />
}

function formatJointName(name: string): string {
  return name
    .split('_')
    .map((w) => w.charAt(0).toUpperCase() + w.slice(1))
    .join(' ')
}
