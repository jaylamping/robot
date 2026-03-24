import { createFileRoute } from '@tanstack/react-router'
import { useEffect, useState, type ReactNode } from 'react'
import { getStatus, getConfig, type ServerStatus, type RobotConfig } from '@/lib/api'
import { useTelemetryStore } from '@/stores/telemetry'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Badge } from '@/components/ui/badge'
import { Separator } from '@/components/ui/separator'
import {
  Table,
  TableBody,
  TableCell,
  TableHead,
  TableHeader,
  TableRow,
} from '@/components/ui/table'
import { LuServer, LuRadio, LuCpu, LuThermometer, LuMemoryStick } from 'react-icons/lu'

export const Route = createFileRoute('/system')({
  component: SystemPage,
})

function SystemPage() {
  const [status, setStatus] = useState<ServerStatus | null>(null)
  const [config, setConfig] = useState<RobotConfig | null>(null)
  const [error, setError] = useState<string | null>(null)
  const connected = useTelemetryStore((s) => s.connected)
  const motors = useTelemetryStore((s) => s.motors)
  const systemTelemetry = useTelemetryStore((s) => s.system)

  useEffect(() => {
    Promise.all([getStatus(), getConfig()])
      .then(([s, c]) => { setStatus(s); setConfig(c) })
      .catch((e) => setError(e.message))
  }, [])

  const refreshStatus = () => {
    getStatus().then(setStatus).catch(() => {})
  }

  useEffect(() => {
    const interval = setInterval(refreshStatus, 5000)
    return () => clearInterval(interval)
  }, [])

  if (error) {
    return (
      <div className="flex items-center justify-center h-64">
        <p className="text-destructive text-sm">{error}</p>
      </div>
    )
  }

  return (
    <div>
      <h2 className="text-xl font-semibold mb-6">System</h2>

      <div className="grid grid-cols-1 md:grid-cols-2 lg:grid-cols-4 gap-3 mb-6">
        <StatusCard
          icon={<LuServer className="size-4" />}
          label="Uptime"
          value={status ? formatUptime(status.uptime_secs) : '—'}
        />
        <StatusCard
          icon={<LuCpu className="size-4" />}
          label="Mode"
          value={status?.mode ?? '—'}
          badge={status?.mode === 'hardware' ? 'default' : 'secondary'}
        />
        <StatusCard
          icon={<LuRadio className="size-4" />}
          label="Transport"
          value={status?.transport_type ?? '—'}
        />
        <StatusCard
          icon={<LuThermometer className="size-4" />}
          label="Telemetry"
          value={connected ? 'Connected' : 'Disconnected'}
          badge={connected ? 'default' : 'destructive'}
        />
      </div>

      <Card className="mb-6">
        <CardHeader>
          <CardTitle>Pi Telemetry</CardTitle>
        </CardHeader>
        <CardContent>
          {!systemTelemetry ? (
            <TelemetryLoading />
          ) : (
            <div className="grid grid-cols-1 sm:grid-cols-3 gap-3">
              <Metric
                icon={<LuCpu className="size-4" />}
                label="CPU"
                value={`${systemTelemetry.cpu_usage_percent.toFixed(1)}%`}
              />
              <Metric
                icon={<LuMemoryStick className="size-4" />}
                label="Memory"
                value={`${systemTelemetry.memory_used_mb} / ${systemTelemetry.memory_total_mb} MB`}
              />
              <Metric
                icon={<LuThermometer className="size-4" />}
                label="Temp"
                value={
                  systemTelemetry.temperature_c == null
                    ? 'N/A'
                    : `${systemTelemetry.temperature_c.toFixed(1)} C`
                }
              />
            </div>
          )}
        </CardContent>
      </Card>

      <div className="grid grid-cols-1 lg:grid-cols-2 gap-4">
        {config && (
          <Card>
            <CardHeader>
              <CardTitle>Bus Configuration</CardTitle>
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
        )}

        <Card>
          <CardHeader>
            <CardTitle>Motor Summary</CardTitle>
          </CardHeader>
          <CardContent>
            {Object.keys(motors).length === 0 ? (
              <p className="text-sm text-muted-foreground">No motors reporting</p>
            ) : (
              <Table>
                <TableHeader>
                  <TableRow>
                    <TableHead>Joint</TableHead>
                    <TableHead className="text-right">CAN</TableHead>
                    <TableHead className="text-right">Mode</TableHead>
                    <TableHead className="text-right">Status</TableHead>
                  </TableRow>
                </TableHeader>
                <TableBody>
                  {Object.values(motors)
                    .sort((a, b) => a.can_id - b.can_id)
                    .map((m) => (
                      <TableRow key={m.can_id}>
                        <TableCell>{formatJointName(m.joint_name)}</TableCell>
                        <TableCell className="font-mono text-right">{m.can_id}</TableCell>
                        <TableCell className="text-right">{m.mode}</TableCell>
                        <TableCell className="text-right">
                          <Badge variant={m.online ? 'default' : 'secondary'} className="text-xs">
                            {m.online ? 'Online' : 'Offline'}
                          </Badge>
                        </TableCell>
                      </TableRow>
                    ))}
                </TableBody>
              </Table>
            )}
          </CardContent>
        </Card>
      </div>

      <Separator className="my-6" />

      <Card>
        <CardHeader>
          <CardTitle>WebTransport</CardTitle>
        </CardHeader>
        <CardContent>
          <div className="grid grid-cols-1 sm:grid-cols-3 gap-4 text-sm">
            <div>
              <p className="text-muted-foreground mb-1">Status</p>
              <Badge variant={connected ? 'default' : 'secondary'}>
                {connected ? 'Connected' : 'Disconnected'}
              </Badge>
            </div>
            <div>
              <p className="text-muted-foreground mb-1">QUIC Port</p>
              <p className="font-mono">4433</p>
            </div>
            <div>
              <p className="text-muted-foreground mb-1">Protocol</p>
              <p className="font-mono">WebTransport over HTTP/3</p>
            </div>
          </div>
        </CardContent>
      </Card>
    </div>
  )
}

function TelemetryLoading() {
  return (
    <div className="space-y-3">
      <div className="h-2 w-full overflow-hidden rounded-full bg-muted">
        <div className="h-full w-1/3 animate-pulse rounded-full bg-primary/70" />
      </div>
      <div className="grid grid-cols-1 sm:grid-cols-3 gap-3">
        <div className="h-16 rounded-md border border-border bg-muted/40 animate-pulse" />
        <div className="h-16 rounded-md border border-border bg-muted/40 animate-pulse" />
        <div className="h-16 rounded-md border border-border bg-muted/40 animate-pulse" />
      </div>
      <p className="text-xs text-muted-foreground">Connecting to live Pi telemetry...</p>
    </div>
  )
}

function StatusCard({
  icon,
  label,
  value,
  badge,
}: {
  icon: React.ReactNode
  label: string
  value: string
  badge?: 'default' | 'secondary' | 'destructive'
}) {
  return (
    <Card size="sm">
      <CardContent>
        <div className="flex items-center gap-2 text-muted-foreground mb-1">
          {icon}
          <span className="text-xs">{label}</span>
        </div>
        {badge ? (
          <Badge variant={badge} className="mt-1">{value}</Badge>
        ) : (
          <p className="font-mono text-sm">{value}</p>
        )}
      </CardContent>
    </Card>
  )
}

function Metric({
  icon,
  label,
  value,
}: {
  icon: ReactNode
  label: string
  value: string
}) {
  return (
    <div className="rounded-md border border-border bg-muted/30 px-3 py-2">
      <div className="mb-1 flex items-center gap-2 text-muted-foreground">
        {icon}
        <span className="text-xs uppercase tracking-wide">{label}</span>
      </div>
      <p className="text-sm font-mono">{value}</p>
    </div>
  )
}

function formatUptime(secs: number): string {
  const d = Math.floor(secs / 86400)
  const h = Math.floor((secs % 86400) / 3600)
  const m = Math.floor((secs % 3600) / 60)
  const s = secs % 60
  if (d > 0) return `${d}d ${h}h ${m}m`
  if (h > 0) return `${h}h ${m}m ${s}s`
  if (m > 0) return `${m}m ${s}s`
  return `${s}s`
}

function formatJointName(name: string): string {
  return name.split('_').map((w) => w.charAt(0).toUpperCase() + w.slice(1)).join(' ')
}
