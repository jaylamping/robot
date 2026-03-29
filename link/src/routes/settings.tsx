import { createFileRoute } from '@tanstack/react-router'
import { useRobotConfig } from '@/lib/queries'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from '@/components/ui/table'

export const Route = createFileRoute('/settings')({
  component: SettingsPage,
})

function SettingsPage() {
  const configQ = useRobotConfig()

  if (configQ.isError) {
    return (
      <div className="flex items-center justify-center h-64">
        <p className="text-destructive text-sm">{configQ.error.message}</p>
      </div>
    )
  }

  if (configQ.isPending || !configQ.data) {
    return (
      <div className="flex items-center justify-center h-64">
        <p className="text-muted-foreground text-sm">Loading configuration...</p>
      </div>
    )
  }

  const config = configQ.data

  return (
    <div>
      <h2 className="text-xl font-semibold mb-6">Settings</h2>
      <p className="text-sm text-muted-foreground mb-6">
        Read-only view of <code className="bg-muted px-1 py-0.5 rounded text-xs">config/robot.yaml</code>.
        Edit the file and restart the link server to apply changes.
      </p>

      <div className="grid grid-cols-1 lg:grid-cols-2 gap-4 mb-6">
        <Card>
          <CardHeader>
            <CardTitle>Bus</CardTitle>
          </CardHeader>
          <CardContent>
            <Table>
              <TableBody>
                <TableRow>
                  <TableCell className="text-muted-foreground">Port</TableCell>
                  <TableCell className="font-mono text-right">{config.bus.port}</TableCell>
                </TableRow>
                <TableRow>
                  <TableCell className="text-muted-foreground">Baud Rate</TableCell>
                  <TableCell className="font-mono text-right">{config.bus.baud.toLocaleString()}</TableCell>
                </TableRow>
                <TableRow>
                  <TableCell className="text-muted-foreground">CAN Bitrate</TableCell>
                  <TableCell className="font-mono text-right">{(config.bus.can_bitrate / 1000).toLocaleString()} kbps</TableCell>
                </TableRow>
                <TableRow>
                  <TableCell className="text-muted-foreground">Host ID</TableCell>
                  <TableCell className="font-mono text-right">0x{config.bus.host_id.toString(16).toUpperCase()}</TableCell>
                </TableRow>
              </TableBody>
            </Table>
          </CardContent>
        </Card>

        {config.torso && (
          <Card>
            <CardHeader>
              <CardTitle>Torso</CardTitle>
            </CardHeader>
            <CardContent>
              <Table>
                <TableBody>
                  <TableRow>
                    <TableCell className="text-muted-foreground">Frame</TableCell>
                    <TableCell className="font-mono text-right">{config.torso.frame}</TableCell>
                  </TableRow>
                  <TableRow>
                    <TableCell className="text-muted-foreground">Dimensions</TableCell>
                    <TableCell className="font-mono text-right">
                      {config.torso.dimensions_mm[0]} × {config.torso.dimensions_mm[1]} × {config.torso.dimensions_mm[2]} mm
                    </TableCell>
                  </TableRow>
                </TableBody>
              </Table>
            </CardContent>
          </Card>
        )}
      </div>

      <Card className="mb-6">
        <CardHeader>
          <CardTitle>Actuator Specifications</CardTitle>
        </CardHeader>
        <CardContent>
          <Table>
            <TableHeader>
              <TableRow>
                <TableHead>Type</TableHead>
                <TableHead className="text-right">Max Torque</TableHead>
                <TableHead className="text-right">Max Speed</TableHead>
                <TableHead className="text-right">Gear Ratio</TableHead>
                <TableHead className="text-right">Weight</TableHead>
                <TableHead className="text-right">Voltage</TableHead>
              </TableRow>
            </TableHeader>
            <TableBody>
              {Object.entries(config.actuators).map(([name, spec]) => (
                <TableRow key={name}>
                  <TableCell className="font-medium uppercase">{name}</TableCell>
                  <TableCell className="font-mono text-right">{spec.max_torque} N·m</TableCell>
                  <TableCell className="font-mono text-right">{spec.max_speed} rad/s</TableCell>
                  <TableCell className="font-mono text-right">{spec.gear_ratio}:1</TableCell>
                  <TableCell className="font-mono text-right">{spec.weight_kg} kg</TableCell>
                  <TableCell className="font-mono text-right">{spec.voltage_nominal} V</TableCell>
                </TableRow>
              ))}
            </TableBody>
          </Table>
        </CardContent>
      </Card>

      {(config.arm_left || config.arm_right) && (
        <Card>
          <CardHeader>
            <CardTitle>Joint Limits</CardTitle>
          </CardHeader>
          <CardContent>
            <Table>
              <TableHeader>
                <TableRow>
                  <TableHead>Joint</TableHead>
                  <TableHead className="text-right">CAN ID</TableHead>
                  <TableHead className="text-right">Min</TableHead>
                  <TableHead className="text-right">Max</TableHead>
                  <TableHead className="w-40">Range</TableHead>
                </TableRow>
              </TableHeader>
              <TableBody>
                {['arm_left', 'arm_right'].map((armKey) => {
                  const arm = (armKey === 'arm_left' ? config.arm_left : config.arm_right) as Record<string, ArmJoint> | undefined
                  if (!arm) return null
                  const side = armKey === 'arm_left' ? 'Left' : 'Right'
                  return Object.entries(arm).map(([name, joint]) => {
                    if (typeof joint !== 'object' || !joint.limits) return null
                    const minDeg = (joint.limits[0] * 180) / Math.PI
                    const maxDeg = (joint.limits[1] * 180) / Math.PI
                    const totalRange = 360
                    const barLeft = ((minDeg + 180) / totalRange) * 100
                    const barWidth = ((maxDeg - minDeg) / totalRange) * 100

                    return (
                      <TableRow key={`${armKey}_${name}`}>
                        <TableCell>{side} {formatJointName(name)}</TableCell>
                        <TableCell className="font-mono text-right">{joint.can_id ?? '—'}</TableCell>
                        <TableCell className="font-mono text-right">{minDeg.toFixed(0)}°</TableCell>
                        <TableCell className="font-mono text-right">{maxDeg.toFixed(0)}°</TableCell>
                        <TableCell>
                          <div className="relative h-2 w-full rounded-full bg-muted">
                            <div
                              className="absolute h-full rounded-full bg-primary/60"
                              style={{ left: `${barLeft}%`, width: `${barWidth}%` }}
                            />
                          </div>
                        </TableCell>
                      </TableRow>
                    )
                  })
                })}
              </TableBody>
            </Table>
          </CardContent>
        </Card>
      )}
    </div>
  )
}

interface ArmJoint {
  can_id?: number | null
  actuator?: string
  limits: [number, number]
  home_rad?: number
}

function formatJointName(name: string): string {
  return name.split('_').map((w) => w.charAt(0).toUpperCase() + w.slice(1)).join(' ')
}
