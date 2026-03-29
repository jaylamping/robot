import { createFileRoute } from '@tanstack/react-router'
import { useEffect, useState } from 'react'
import { toast } from 'sonner'
import type { CommandResponse, SequenceInfo } from '@/lib/api'
import { useRobotMotors, useRobotSequences } from '@/lib/queries'
import {
  useDiscoverMutation,
  useEstopMutation,
  useEnableMotorMutation,
  useDisableMotorMutation,
  useZeroMotorMutation,
  useMoveMotorMutation,
  useControlMotorMutation,
  useSpinMotorMutation,
  useTorqueMotorMutation,
  useJogMotorMutation,
  useStopMotorMutation,
  useRunSequenceMutation,
} from '@/lib/mutations/robot'
import { useTelemetryStore } from '@/stores/telemetry'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Input } from '@/components/ui/input'
import { Slider } from '@/components/ui/slider'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import { Badge } from '@/components/ui/badge'
import {
  Select,
  SelectContent,
  SelectItem,
  SelectTrigger,
  SelectValue,
} from '@/components/ui/select'
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from '@/components/ui/dialog'
import { LuOctagonX, LuRadar } from 'react-icons/lu'

export const Route = createFileRoute('/test')({
  component: TestPage,
})

function TestPage() {
  const motorsQ = useRobotMotors()
  const sequencesQ = useRobotSequences()
  const discoverMut = useDiscoverMutation()
  const estopMut = useEstopMutation()
  const enableMut = useEnableMotorMutation()
  const disableMut = useDisableMotorMutation()
  const zeroMut = useZeroMotorMutation()
  const stopMut = useStopMotorMutation()

  const [selectedMotorId, setSelectedMotorId] = useState<number | null>(null)
  const [busy, setBusy] = useState(false)

  const motors = motorsQ.data ?? []
  const sequences = sequencesQ.data ?? []
  const loading = motorsQ.isPending || sequencesQ.isPending

  useEffect(() => {
    if (motors.length > 0 && (selectedMotorId === null || !motors.some((x) => x.can_id === selectedMotorId))) {
      setSelectedMotorId(motors[0].can_id)
    }
  }, [motors, selectedMotorId])

  useEffect(() => {
    if (motorsQ.isError && motorsQ.error) {
      toast.error('Failed to load motors', { description: motorsQ.error.message })
    }
    if (sequencesQ.isError && sequencesQ.error) {
      toast.error('Failed to load sequences', { description: sequencesQ.error.message })
    }
  }, [motorsQ.isError, motorsQ.error, sequencesQ.isError, sequencesQ.error])

  async function handleDiscover() {
    try {
      const result = await discoverMut.mutateAsync()
      if (result.discovered.length > 0) {
        toast.success(`Discovered ${result.discovered.length} motor(s)`, {
          description: `CAN IDs: ${result.discovered.join(', ')}`,
        })
      } else if (result.removed.length > 0) {
        toast.info(`Removed ${result.removed.length} offline motor(s)`)
      } else {
        toast.info(`No changes — ${result.total} motor(s) online`)
      }
    } catch (e) {
      toast.error('Discovery failed', {
        description: e instanceof Error ? e.message : String(e),
      })
    }
  }

  async function exec(label: string, fn: () => Promise<CommandResponse>) {
    setBusy(true)
    try {
      const res = await fn()
      if (res.success) {
        toast.success(`${label}: OK`, {
          description: res.angle_rad != null
            ? `pos: ${res.angle_rad.toFixed(3)} rad, vel: ${res.velocity_rads?.toFixed(3)} rad/s, trq: ${res.torque_nm?.toFixed(3)} N·m`
            : undefined,
        })
      } else {
        toast.error(`${label} failed`, { description: res.error })
      }
    } catch (e) {
      toast.error(`${label} failed`, {
        description: e instanceof Error ? e.message : String(e),
      })
    } finally {
      setBusy(false)
    }
  }

  async function handleEstop() {
    setBusy(true)
    try {
      const res = await estopMut.mutateAsync()
      if (res.success) {
        toast.success('E-STOP: All motors disabled')
      } else {
        toast.error('E-STOP partial failure', { description: res.error })
      }
    } catch (e) {
      toast.error('E-STOP failed', {
        description: e instanceof Error ? e.message : String(e),
      })
    } finally {
      setBusy(false)
    }
  }

  if (loading) {
    return (
      <div className="flex items-center justify-center h-64">
        <p className="text-muted-foreground text-sm">Loading test panel...</p>
      </div>
    )
  }

  const selectedMotor = motors.find((m) => m.can_id === selectedMotorId)

  return (
    <div className="space-y-6">
      <div className="flex items-center justify-between">
        <h2 className="text-xl font-semibold">Test Panel</h2>
        <div className="flex items-center gap-3">
          <Button
            variant="outline"
            size="sm"
            onClick={() => void handleDiscover()}
            disabled={discoverMut.isPending || busy}
            className="gap-2"
          >
            <LuRadar className={`size-4 ${discoverMut.isPending ? 'animate-spin' : ''}`} />
            {discoverMut.isPending ? 'Scanning...' : 'Discover'}
          </Button>
          <Button
            variant="destructive"
            size="lg"
            className="gap-2 font-bold"
            onClick={handleEstop}
            disabled={busy}
          >
            <LuOctagonX className="size-5" />
            E-STOP ALL
          </Button>
        </div>
      </div>

      <div className="grid grid-cols-1 lg:grid-cols-3 gap-6">
        <div className="lg:col-span-2 space-y-4">
          <Card>
            <CardHeader>
              <CardTitle className="flex items-center justify-between">
                <span>Motor Controls</span>
                <Select
                  value={selectedMotorId != null ? String(selectedMotorId) : undefined}
                  onValueChange={(v) => setSelectedMotorId(Number(v))}
                >
                  <SelectTrigger className="w-56">
                    <SelectValue placeholder="Select motor" />
                  </SelectTrigger>
                  <SelectContent>
                    {motors.map((m) => (
                      <SelectItem key={m.can_id} value={String(m.can_id)}>
                        {m.joint_name} (CAN {m.can_id})
                      </SelectItem>
                    ))}
                  </SelectContent>
                </Select>
              </CardTitle>
            </CardHeader>
            <CardContent className="space-y-4">
              {selectedMotorId != null && selectedMotor ? (
                <>
                  <TelemetryReadout canId={selectedMotorId} />

                  <div className="flex flex-wrap gap-2">
                    <ConfirmButton
                      label="Enable"
                      description="This will energize the motor. Make sure the area is clear."
                      variant="default"
                      confirmVariant="default"
                      disabled={busy}
                      onConfirm={() => exec('Enable', () => enableMut.mutateAsync(selectedMotorId))}
                    />
                    <ConfirmButton
                      label="Disable"
                      description="This will de-energize the motor. It may drop under gravity."
                      variant="outline"
                      confirmVariant="destructive"
                      disabled={busy}
                      onConfirm={() => exec('Disable', () => disableMut.mutateAsync(selectedMotorId))}
                    />
                    <Button
                      variant="outline"
                      disabled={busy}
                      onClick={() => exec('Set Zero', () => zeroMut.mutateAsync(selectedMotorId))}
                    >
                      Set Zero
                    </Button>
                    <Button
                      variant="destructive"
                      disabled={busy}
                      onClick={() => exec('Stop', () => stopMut.mutateAsync(selectedMotorId))}
                    >
                      <LuOctagonX className="size-4 mr-1" />
                      Stop
                    </Button>
                  </div>

                  <Tabs defaultValue="jog">
                    <TabsList>
                      <TabsTrigger value="jog">Jog</TabsTrigger>
                      <TabsTrigger value="spin">Spin</TabsTrigger>
                      <TabsTrigger value="torque">Torque</TabsTrigger>
                      <TabsTrigger value="position">Position</TabsTrigger>
                      <TabsTrigger value="raw">Raw MIT</TabsTrigger>
                    </TabsList>

                    <TabsContent value="jog" className="pt-3">
                      <JogTab canId={selectedMotorId} busy={busy} exec={exec} />
                    </TabsContent>

                    <TabsContent value="spin" className="pt-3">
                      <SpinTab canId={selectedMotorId} busy={busy} exec={exec} />
                    </TabsContent>

                    <TabsContent value="torque" className="pt-3">
                      <TorqueTab canId={selectedMotorId} busy={busy} exec={exec} />
                    </TabsContent>

                    <TabsContent value="position" className="pt-3">
                      <PositionTab
                        canId={selectedMotorId}
                        limitsRad={selectedMotor.limits as [number, number]}
                        busy={busy}
                        exec={exec}
                      />
                    </TabsContent>

                    <TabsContent value="raw" className="pt-3">
                      <RawMitTab canId={selectedMotorId} busy={busy} exec={exec} />
                    </TabsContent>
                  </Tabs>
                </>
              ) : (
                <p className="text-sm text-muted-foreground">Select a motor to begin testing.</p>
              )}
            </CardContent>
          </Card>
        </div>

        <div>
          <SequencePanel sequences={sequences} busy={busy} exec={exec} />
        </div>
      </div>
    </div>
  )
}

function TelemetryReadout({ canId }: { canId: number }) {
  const motor = useTelemetryStore((s) => s.motors[canId])

  if (!motor) {
    return (
      <div className="rounded-md border border-dashed p-3">
        <p className="text-xs text-muted-foreground">No telemetry data for CAN {canId}</p>
      </div>
    )
  }

  return (
    <div className="grid grid-cols-2 sm:grid-cols-4 gap-3">
      <ReadoutCell label="Position" value={`${motor.angle_rad.toFixed(3)} rad`} />
      <ReadoutCell label="Velocity" value={`${motor.velocity_rads.toFixed(3)} rad/s`} />
      <ReadoutCell label="Torque" value={`${motor.torque_nm.toFixed(3)} N·m`} />
      <ReadoutCell
        label="Status"
        value={
          <div className="flex items-center gap-1.5">
            <span
              className={`inline-block size-2 rounded-full ${motor.online ? 'bg-emerald-500' : 'bg-muted-foreground'}`}
            />
            <span>{motor.mode}</span>
            {motor.faults.length > 0 && (
              <Badge variant="destructive" className="text-[10px] px-1 py-0">
                {motor.faults.length} fault{motor.faults.length !== 1 ? 's' : ''}
              </Badge>
            )}
          </div>
        }
      />
    </div>
  )
}

function ReadoutCell({ label, value }: { label: string; value: React.ReactNode }) {
  return (
    <div className="rounded-md bg-muted/50 p-2">
      <p className="text-[10px] uppercase tracking-wider text-muted-foreground mb-0.5">{label}</p>
      <p className="text-sm font-mono font-medium">{value}</p>
    </div>
  )
}

type ExecFn = (label: string, fn: () => Promise<CommandResponse>) => Promise<void>

function JogTab({ canId, busy, exec }: { canId: number; busy: boolean; exec: ExecFn }) {
  const jogMut = useJogMotorMutation()
  const [customDeg, setCustomDeg] = useState(15)
  const [kp, setKp] = useState(30)
  const [kd, setKd] = useState(1)

  const jogPresets = [1, 5, 10, 45]

  return (
    <div className="space-y-3">
      <div className="grid grid-cols-2 gap-2">
        <LabeledInput label="kp" value={kp} onChange={setKp} step={1} min={0} max={5000} />
        <LabeledInput label="kd" value={kd} onChange={setKd} step={0.1} min={0} max={100} />
      </div>
      <div className="flex flex-wrap gap-2">
        {jogPresets.map((deg) => (
          <div key={deg} className="flex gap-1">
            <Button
              variant="outline"
              size="sm"
              disabled={busy}
              onClick={() =>
                exec(`Jog -${deg}°`, () =>
                  jogMut.mutateAsync({ id: canId, delta_deg: -deg, kp, kd }),
                )
              }
            >
              -{deg}°
            </Button>
            <Button
              variant="outline"
              size="sm"
              disabled={busy}
              onClick={() =>
                exec(`Jog +${deg}°`, () =>
                  jogMut.mutateAsync({ id: canId, delta_deg: deg, kp, kd }),
                )
              }
            >
              +{deg}°
            </Button>
          </div>
        ))}
      </div>
      <div className="flex gap-2 items-end">
        <div className="flex-1">
          <LabeledInput label="Custom (°)" value={customDeg} onChange={setCustomDeg} step={1} />
        </div>
        <Button
          variant="outline"
          disabled={busy}
          onClick={() =>
            exec(`Jog -${customDeg}°`, () =>
              jogMut.mutateAsync({ id: canId, delta_deg: -customDeg, kp, kd }),
            )
          }
        >
          -{customDeg}°
        </Button>
        <Button
          variant="outline"
          disabled={busy}
          onClick={() =>
            exec(`Jog +${customDeg}°`, () =>
              jogMut.mutateAsync({ id: canId, delta_deg: customDeg, kp, kd }),
            )
          }
        >
          +{customDeg}°
        </Button>
      </div>
    </div>
  )
}

function SpinTab({ canId, busy, exec }: { canId: number; busy: boolean; exec: ExecFn }) {
  const spinMut = useSpinMotorMutation()
  const [velocity, setVelocity] = useState(0)
  const [kd, setKd] = useState(1)

  return (
    <div className="space-y-3">
      <div className="grid grid-cols-2 gap-2">
        <LabeledInput label="Velocity (rad/s)" value={velocity} onChange={setVelocity} step={0.5} min={-10} max={10} />
        <LabeledInput label="kd" value={kd} onChange={setKd} step={0.1} min={0} max={100} />
      </div>
      <div className="relative pt-1 pb-2">
        <Slider
          value={[velocity]}
          onValueChange={(v) => {
            const arr = Array.isArray(v) ? v : [v]
            setVelocity(arr[0])
          }}
          min={-10}
          max={10}
          step={0.1}
        />
        <div className="mt-1 flex justify-between text-[10px] text-muted-foreground font-mono">
          <span>-10 rad/s</span>
          <span>0</span>
          <span>+10 rad/s</span>
        </div>
      </div>
      <div className="flex gap-2">
        <Button
          className="flex-1"
          disabled={busy}
          onClick={() =>
            exec('Spin', () => spinMut.mutateAsync({ id: canId, velocity_rads: velocity, kd }))
          }
        >
          Start Spin
        </Button>
        <Button
          variant="destructive"
          disabled={busy}
          onClick={() =>
            exec('Stop Spin', () => spinMut.mutateAsync({ id: canId, velocity_rads: 0, kd }))
          }
        >
          Stop
        </Button>
      </div>
    </div>
  )
}

function TorqueTab({ canId, busy, exec }: { canId: number; busy: boolean; exec: ExecFn }) {
  const torqueMut = useTorqueMotorMutation()
  const [torque, setTorque] = useState(0)

  return (
    <div className="space-y-3">
      <LabeledInput label="Torque (N·m)" value={torque} onChange={setTorque} step={0.5} min={-30} max={30} />
      <div className="relative pt-1 pb-2">
        <Slider
          value={[torque]}
          onValueChange={(v) => {
            const arr = Array.isArray(v) ? v : [v]
            setTorque(arr[0])
          }}
          min={-30}
          max={30}
          step={0.1}
        />
        <div className="mt-1 flex justify-between text-[10px] text-muted-foreground font-mono">
          <span>-30 N·m</span>
          <span>0</span>
          <span>+30 N·m</span>
        </div>
      </div>
      <div className="flex gap-2">
        <Button
          className="flex-1"
          disabled={busy}
          onClick={() =>
            exec('Torque', () => torqueMut.mutateAsync({ id: canId, torque_nm: torque }))
          }
        >
          Apply Torque
        </Button>
        <Button
          variant="destructive"
          disabled={busy}
          onClick={() =>
            exec('Stop Torque', () => torqueMut.mutateAsync({ id: canId, torque_nm: 0 }))
          }
        >
          Stop
        </Button>
      </div>
    </div>
  )
}

function PositionTab({
  canId,
  limitsRad,
  busy,
  exec,
}: {
  canId: number
  limitsRad: [number, number]
  busy: boolean
  exec: ExecFn
}) {
  const moveMut = useMoveMotorMutation()
  const [positionDeg, setPositionDeg] = useState(0)
  const [kp, setKp] = useState(30)
  const [kd, setKd] = useState(1)

  const minDeg = (limitsRad[0] * 180) / Math.PI
  const maxDeg = (limitsRad[1] * 180) / Math.PI

  return (
    <div className="space-y-3">
      <div className="grid grid-cols-3 gap-2">
        <LabeledInput label="Target (°)" value={positionDeg} onChange={setPositionDeg} step={1} />
        <LabeledInput label="kp" value={kp} onChange={setKp} step={1} min={0} max={5000} />
        <LabeledInput label="kd" value={kd} onChange={setKd} step={0.1} min={0} max={100} />
      </div>
      <div className="relative pt-1 pb-2">
        <Slider
          value={[positionDeg]}
          onValueChange={(v) => {
            const arr = Array.isArray(v) ? v : [v]
            setPositionDeg(arr[0])
          }}
          min={minDeg}
          max={maxDeg}
          step={0.5}
        />
        <div className="mt-1 flex justify-between text-[10px] text-muted-foreground font-mono">
          <span>{minDeg.toFixed(0)}°</span>
          <span>{maxDeg.toFixed(0)}°</span>
        </div>
      </div>
      <Button
        className="w-full"
        disabled={busy}
        onClick={() =>
          exec('Move', () =>
            moveMut.mutateAsync({
              id: canId,
              position_rad: (positionDeg * Math.PI) / 180,
              kp,
              kd,
            }),
          )
        }
      >
        Move to Position
      </Button>
    </div>
  )
}

function RawMitTab({ canId, busy, exec }: { canId: number; busy: boolean; exec: ExecFn }) {
  const controlMut = useControlMotorMutation()
  const [pos, setPos] = useState(0)
  const [vel, setVel] = useState(0)
  const [kp, setKp] = useState(30)
  const [kd, setKd] = useState(1)
  const [trq, setTrq] = useState(0)

  return (
    <div className="space-y-3">
      <div className="grid grid-cols-5 gap-2">
        <LabeledInput label="pos (rad)" value={pos} onChange={setPos} step={0.1} />
        <LabeledInput label="vel (rad/s)" value={vel} onChange={setVel} step={0.5} />
        <LabeledInput label="kp" value={kp} onChange={setKp} step={1} />
        <LabeledInput label="kd" value={kd} onChange={setKd} step={0.1} />
        <LabeledInput label="trq (N·m)" value={trq} onChange={setTrq} step={0.5} />
      </div>
      <Button
        variant="outline"
        className="w-full"
        disabled={busy}
        onClick={() =>
          exec('Send Control', () =>
            controlMut.mutateAsync({
              id: canId,
              position: pos,
              velocity: vel,
              kp,
              kd,
              torque: trq,
            }),
          )
        }
      >
        Send
      </Button>
    </div>
  )
}

function SequencePanel({
  sequences,
  busy,
  exec,
}: {
  sequences: SequenceInfo[]
  busy: boolean
  exec: ExecFn
}) {
  const runSeqMut = useRunSequenceMutation()
  const [runningSeq, setRunningSeq] = useState<string | null>(null)

  async function handleRun(name: string) {
    setRunningSeq(name)
    await exec(`Sequence: ${name}`, () => runSeqMut.mutateAsync(name))
    setRunningSeq(null)
  }

  return (
    <Card>
      <CardHeader>
        <CardTitle>Sequences</CardTitle>
      </CardHeader>
      <CardContent className="space-y-2">
        {sequences.length === 0 ? (
          <p className="text-sm text-muted-foreground">No sequences available.</p>
        ) : (
          sequences.map((seq) => (
            <div
              key={seq.name}
              className="flex items-center justify-between rounded-md border p-3"
            >
              <div className="min-w-0 flex-1 mr-3">
                <p className="text-sm font-medium truncate">{seq.name}</p>
                <p className="text-xs text-muted-foreground truncate">{seq.description}</p>
              </div>
              <div className="flex items-center gap-2 shrink-0">
                {runningSeq === seq.name && (
                  <Badge variant="secondary" className="text-[10px]">Running</Badge>
                )}
                <Button
                  size="sm"
                  variant="outline"
                  disabled={busy}
                  onClick={() => handleRun(seq.name)}
                >
                  Run
                </Button>
              </div>
            </div>
          ))
        )}
      </CardContent>
    </Card>
  )
}

function LabeledInput({
  label,
  value,
  onChange,
  step,
  min,
  max,
}: {
  label: string
  value: number
  onChange: (v: number) => void
  step: number
  min?: number
  max?: number
}) {
  return (
    <div>
      <label className="mb-1 block text-xs text-muted-foreground">{label}</label>
      <Input
        type="number"
        value={value}
        onChange={(e) => onChange(Number(e.target.value))}
        step={step}
        min={min}
        max={max}
        className="font-mono"
      />
    </div>
  )
}

function ConfirmButton({
  label,
  description,
  variant,
  confirmVariant,
  disabled,
  onConfirm,
}: {
  label: string
  description: string
  variant: 'default' | 'destructive' | 'outline'
  confirmVariant: 'default' | 'destructive'
  disabled?: boolean
  onConfirm: () => void
}) {
  const [open, setOpen] = useState(false)

  return (
    <Dialog open={open} onOpenChange={setOpen}>
      <DialogTrigger render={<Button variant={variant} disabled={disabled} />}>
        {label}
      </DialogTrigger>
      <DialogContent>
        <DialogHeader>
          <DialogTitle>Confirm: {label}</DialogTitle>
          <DialogDescription>{description}</DialogDescription>
        </DialogHeader>
        <DialogFooter>
          <Button variant="outline" onClick={() => setOpen(false)}>
            Cancel
          </Button>
          <Button
            variant={confirmVariant}
            onClick={() => {
              setOpen(false)
              onConfirm()
            }}
          >
            {label}
          </Button>
        </DialogFooter>
      </DialogContent>
    </Dialog>
  )
}
