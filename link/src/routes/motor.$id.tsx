import { createFileRoute, Link } from '@tanstack/react-router'
import { useTelemetryStore } from '@/stores/telemetry'
import { useRobotMotor } from '@/lib/queries'
import { TelemetryChart } from '@/components/TelemetryChart'
import { MotorControl } from '@/components/MotorControl'
import { Card, CardContent } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { LuArrowLeft } from 'react-icons/lu'

export const Route = createFileRoute('/motor/$id')({
  component: MotorDetailPage,
})

function MotorDetailPage() {
  const { id } = Route.useParams()
  const canId = Number(id)

  const motor = useTelemetryStore((s) => s.motors[canId])
  const history = useTelemetryStore((s) => s.history[canId] ?? [])

  const detailQ = useRobotMotor(canId)
  const detail = detailQ.data

  const limitsRad: [number, number] | undefined = detail
    ? [detail.limits[0], detail.limits[1]]
    : undefined

  if (!motor) {
    return (
      <div className="text-center py-16">
        <p className="text-muted-foreground mb-4">No telemetry data for motor {canId}</p>
        <p className="text-xs text-muted-foreground mb-4">
          The motor may be offline or telemetry is not yet connected.
        </p>
        <Link to="/" className="inline-flex items-center gap-1.5 text-sm text-primary hover:underline">
          Back to overview
        </Link>
      </div>
    )
  }

  const hasFaults = motor.faults.length > 0
  const highTemp = motor.temperature_c > 60

  return (
    <div>
      <div className="mb-6">
        <Link to="/" className="mb-2 -ml-2 inline-flex items-center gap-1 rounded-md px-2 py-1.5 text-sm text-muted-foreground hover:text-foreground hover:bg-accent transition-colors">
          <LuArrowLeft className="size-4" />
          Overview
        </Link>
        <h2 className="text-xl font-semibold">{formatJointName(motor.joint_name)}</h2>
        <div className="mt-1 flex flex-wrap items-center gap-2">
          <Badge variant="outline">CAN {motor.can_id}</Badge>
          <Badge variant="secondary">{motor.mode}</Badge>
          <Badge className={motor.online
            ? 'bg-emerald-500/10 text-emerald-400 border-emerald-500/20'
            : 'bg-muted text-muted-foreground'
          }>
            {motor.online ? 'Online' : 'Offline'}
          </Badge>
        </div>
      </div>

      <div className="grid grid-cols-2 md:grid-cols-4 gap-3 mb-6">
        <StatCard
          label="Position"
          value={`${((motor.angle_rad * 180) / Math.PI).toFixed(1)}°`}
          sub={`${motor.angle_rad.toFixed(3)} rad`}
        />
        <StatCard
          label="Velocity"
          value={motor.velocity_rads.toFixed(3)}
          sub="rad/s"
        />
        <StatCard
          label="Torque"
          value={motor.torque_nm.toFixed(3)}
          sub="N·m"
        />
        <StatCard
          label="Temperature"
          value={`${motor.temperature_c.toFixed(1)}°C`}
          warn={highTemp}
        />
      </div>

      <div className="space-y-4 mb-6">
        <TelemetryChart
          history={history}
          dataKey="angle_rad"
          label="Position (rad)"
          unit="rad"
          color="hsl(217, 91%, 60%)"
        />
        <TelemetryChart
          history={history}
          dataKey="velocity_rads"
          label="Velocity (rad/s)"
          unit="rad/s"
          color="hsl(160, 84%, 39%)"
        />
        <TelemetryChart
          history={history}
          dataKey="torque_nm"
          label="Torque (N·m)"
          unit="N·m"
          color="hsl(38, 92%, 50%)"
        />
      </div>

      <div className="grid grid-cols-1 md:grid-cols-2 gap-4">
        <MotorControl canId={canId} currentAngleRad={motor.angle_rad} limitsRad={limitsRad} />

        <Card>
          <CardContent className="pt-4">
            <h3 className="text-sm font-medium text-muted-foreground mb-2">Faults</h3>
            {!hasFaults ? (
              <p className="text-sm text-emerald-400">No faults</p>
            ) : (
              <ul className="space-y-1">
                {motor.faults.map((f, i) => (
                  <li key={i} className="text-sm text-destructive font-mono">{f}</li>
                ))}
              </ul>
            )}
          </CardContent>
        </Card>
      </div>
    </div>
  )
}

function StatCard({
  label,
  value,
  sub,
  warn,
}: {
  label: string
  value: string
  sub?: string
  warn?: boolean
}) {
  return (
    <Card size="sm">
      <CardContent>
        <p className="text-xs text-muted-foreground mb-0.5">{label}</p>
        <p className={`text-lg font-mono leading-tight ${warn ? 'text-amber-400' : ''}`}>
          {value}
        </p>
        {sub && <p className="text-xs text-muted-foreground">{sub}</p>}
      </CardContent>
    </Card>
  )
}

function formatJointName(name: string): string {
  return name
    .split('_')
    .map((w) => w.charAt(0).toUpperCase() + w.slice(1))
    .join(' ')
}
