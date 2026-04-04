import { createFileRoute, useNavigate } from '@tanstack/react-router'
import { useState } from 'react'
import type { DiscoverResult, MotorInfo } from '@/lib/api'
import { useTelemetryStore, type MotorSnapshot } from '@/stores/telemetry'
import { MotorCard } from '@/components/MotorCard'
import { RobotDiagram } from '@/components/RobotDiagram'
import { HomingStatusCard } from '@/components/HomingStatusCard'
import { Badge } from '@/components/ui/badge'
import { Button } from '@/components/ui/button'
import { useRobotConfig, useRobotJointSlots, useRobotMotors } from '@/lib/queries'
import { useDiscoverMutation } from '@/lib/mutations/robot'
import { toast } from 'sonner'
import { LuBot, LuRadar } from 'react-icons/lu'

export const Route = createFileRoute('/')({
  component: OverviewPage,
})

/** Same rule as MotorCard: prefer live telemetry when present, else REST list `online`. */
function isMotorShowingOnline(
  m: MotorInfo,
  telemetryMotors: Record<number, MotorSnapshot>,
): boolean {
  const live = telemetryMotors[m.can_id]
  return live?.online ?? m.online
}

function OverviewPage() {
  const [discoverResult, setDiscoverResult] = useState<DiscoverResult | null>(null)
  const navigate = useNavigate()
  const telemetryMotors = useTelemetryStore((s) => s.motors)

  const motorsQ = useRobotMotors()
  const configQ = useRobotConfig()
  const jointSlotsQ = useRobotJointSlots()
  const discoverMut = useDiscoverMutation()

  const loading = motorsQ.isPending || configQ.isPending || jointSlotsQ.isPending
  const error =
    motorsQ.error ?? configQ.error ?? jointSlotsQ.error
      ? String(motorsQ.error?.message ?? configQ.error?.message ?? jointSlotsQ.error?.message ?? 'Error')
      : null

  const motors = motorsQ.data ?? []
  const config = configQ.data ?? null
  const jointSlots = jointSlotsQ.data ?? []

  const handleDiscover = async () => {
    setDiscoverResult(null)
    try {
      const result = await discoverMut.mutateAsync()
      setDiscoverResult(result)
    } catch (e) {
      setDiscoverResult(null)
      toast.error('Discovery failed', {
        description: e instanceof Error ? e.message : String(e),
      })
    }
  }

  if (loading) {
    return (
      <div className="flex items-center justify-center h-64">
        <p className="text-muted-foreground text-sm">Loading motors...</p>
      </div>
    )
  }

  if (error) {
    return (
      <div className="flex items-center justify-center h-64">
        <div className="text-center">
          <p className="text-destructive text-sm mb-2">Failed to load motors</p>
          <p className="text-muted-foreground text-xs font-mono">{error}</p>
        </div>
      </div>
    )
  }

  if (motors.length === 0) {
    return (
      <div className="flex flex-col items-center justify-center h-64 text-center">
        <LuBot className="size-12 text-muted-foreground/50 mb-4" />
        <h2 className="text-lg font-medium text-foreground mb-1">No motors configured</h2>
        <p className="text-sm text-muted-foreground max-w-sm">
          Add motor CAN IDs to <code className="text-xs bg-muted px-1 py-0.5 rounded">config/robot.yaml</code> and
          restart the link server.
        </p>
      </div>
    )
  }

  const onlineCount = motors.filter((m) => isMotorShowingOnline(m, telemetryMotors)).length
  const faultCount = motors.filter((m) => {
    const live = telemetryMotors[m.can_id]
    return (live?.faults.length ?? 0) > 0
  }).length
  const offlineCount = motors.length - onlineCount

  const onlineMotors = motors.filter((m) => isMotorShowingOnline(m, telemetryMotors))
  const offlineMotors = motors.filter((m) => !isMotorShowingOnline(m, telemetryMotors))

  const homingArmSides = [
    ...(config?.arm_left ? ['left'] : []),
    ...(config?.arm_right ? ['right'] : []),
  ]

  return (
    <div>
      <div className="mb-6">
        <div className="flex items-center justify-between">
          <h2 className="text-xl font-semibold">Overview</h2>
          <Button
            variant="outline"
            size="sm"
            onClick={() => void handleDiscover()}
            disabled={discoverMut.isPending}
            title="Drop unassigned motor handles (not in robot.yaml), then scan the bus for devices"
            className="gap-2"
          >
            <LuRadar className={`size-4 ${discoverMut.isPending ? 'animate-spin' : ''}`} />
            {discoverMut.isPending ? 'Scanning...' : 'Discover'}
          </Button>
        </div>
        <div className="mt-2 flex flex-wrap gap-2">
          <Badge variant="outline">{motors.length} configured</Badge>
          {onlineCount > 0 && (
            <Badge className="bg-emerald-500/10 text-emerald-400 border-emerald-500/20">
              {onlineCount} online
            </Badge>
          )}
          {offlineCount > 0 && (
            <Badge variant="secondary">{offlineCount} offline</Badge>
          )}
          {faultCount > 0 && (
            <Badge variant="destructive">{faultCount} faulted</Badge>
          )}
        </div>
        {discoverResult && (
          <div className="mt-3 rounded-md border border-border bg-muted/50 px-3 py-2 text-sm space-y-1">
            {discoverResult.pruned_ghosts.length > 0 && (
              <p className="text-sky-400">
                Dropped ghosts (not in robot.yaml):{' '}
                {discoverResult.pruned_ghosts.map(id => `motor ${id}`).join(', ')}
              </p>
            )}
            {discoverResult.discovered.length > 0 && (
              <p className="text-emerald-400">
                Found: {discoverResult.discovered.map(id => `motor ${id}`).join(', ')}
              </p>
            )}
            {discoverResult.removed.length > 0 && (
              <p className="text-amber-400">
                Removed (offline): {discoverResult.removed.map(id => `motor ${id}`).join(', ')}
              </p>
            )}
            {discoverResult.discovered.length === 0 &&
              discoverResult.removed.length === 0 &&
              discoverResult.pruned_ghosts.length === 0 && (
                <p className="text-muted-foreground">No changes — {discoverResult.total} motor(s) online</p>
              )}
          </div>
        )}
      </div>

      {config && (
        <div className="mb-6">
          <HomingStatusCard armSides={homingArmSides} />
        </div>
      )}

      {config && (
        <div className="mb-6">
          <RobotDiagram
            armLeftCanIds={extractArmCanIds(config.arm_left)}
            armRightCanIds={extractArmCanIds(config.arm_right)}
            waistCanId={extractWaistCanId(config.waist)}
          />
        </div>
      )}

      {onlineMotors.length > 0 && (
        <section className="mb-8">
          <h3 className="text-sm font-medium text-muted-foreground mb-3 uppercase tracking-wider">
            Online
          </h3>
          <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 xl:grid-cols-4 gap-3">
            {onlineMotors.map((motor) => (
              <MotorCard
                key={motor.can_id}
                motor={motor}
                jointSlots={jointSlots}
                onClick={() => navigate({ to: '/motor/$id', params: { id: String(motor.can_id) } })}
              />
            ))}
          </div>
        </section>
      )}

      {offlineMotors.length > 0 && (
        <section>
          <h3 className="text-sm font-medium text-muted-foreground mb-3 uppercase tracking-wider">
            {onlineMotors.length > 0 ? 'Offline / Unassigned' : 'All Motors'}
          </h3>
          <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-3 xl:grid-cols-4 gap-3">
            {offlineMotors.map((motor) => (
              <MotorCard
                key={motor.can_id}
                motor={motor}
                jointSlots={jointSlots}
                onClick={() => navigate({ to: '/motor/$id', params: { id: String(motor.can_id) } })}
              />
            ))}
          </div>
        </section>
      )}
    </div>
  )
}

function extractArmCanIds(arm: Record<string, unknown> | undefined): (number | null)[] {
  if (!arm) return [null, null, null, null]
  const joints = ['shoulder_pitch', 'shoulder_roll', 'upper_arm_yaw', 'elbow_pitch']
  return joints.map((name) => {
    const joint = arm[name] as { can_id?: number | null } | undefined
    return joint?.can_id ?? null
  })
}

function extractWaistCanId(waist: Record<string, unknown> | undefined): number | null {
  if (!waist) return null
  const rotation = waist['rotation'] as { can_id?: number | null } | undefined
  return rotation?.can_id ?? null
}
