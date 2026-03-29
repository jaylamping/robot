import { useState } from 'react'
import { toast } from 'sonner'
import { Button } from '@/components/ui/button'
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card'
import { Input } from '@/components/ui/input'
import { Slider } from '@/components/ui/slider'
import { Tabs, TabsContent, TabsList, TabsTrigger } from '@/components/ui/tabs'
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from '@/components/ui/dialog'
import type { CommandResponse } from '@/lib/api'
import {
  useEnableMotorMutation,
  useDisableMotorMutation,
  useZeroMotorMutation,
  useMoveMotorMutation,
  useControlMotorMutation,
} from '@/lib/mutations/robot'

interface MotorControlProps {
  canId: number
  currentAngleRad: number
  limitsRad?: [number, number]
}

export function MotorControl({ canId, currentAngleRad, limitsRad }: MotorControlProps) {
  const [positionDeg, setPositionDeg] = useState((currentAngleRad * 180) / Math.PI)
  const [kp, setKp] = useState(30)
  const [kd, setKd] = useState(1)
  const [busy, setBusy] = useState(false)
  const enableMut = useEnableMotorMutation()
  const disableMut = useDisableMotorMutation()
  const zeroMut = useZeroMotorMutation()
  const moveMut = useMoveMotorMutation()
  const controlMut = useControlMotorMutation()

  const minDeg = limitsRad ? (limitsRad[0] * 180) / Math.PI : -180
  const maxDeg = limitsRad ? (limitsRad[1] * 180) / Math.PI : 180

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

  return (
    <Card>
      <CardHeader>
        <CardTitle>Controls</CardTitle>
      </CardHeader>
      <CardContent className="space-y-4">
        <div className="flex flex-wrap gap-2">
          <ConfirmButton
            label="Enable"
            description="This will energize the motor. Make sure the area is clear."
            variant="default"
            confirmVariant="default"
            disabled={busy}
            onConfirm={() => exec('Enable', () => enableMut.mutateAsync(canId))}
          />
          <ConfirmButton
            label="Disable"
            description="This will de-energize the motor. It may drop under gravity."
            variant="destructive"
            confirmVariant="destructive"
            disabled={busy}
            onConfirm={() => exec('Disable', () => disableMut.mutateAsync(canId))}
          />
          <Button
            variant="outline"
            disabled={busy}
            onClick={() => exec('Set Zero', () => zeroMut.mutateAsync(canId))}
          >
            Set Zero
          </Button>
        </div>

        <Tabs defaultValue="position">
          <TabsList>
            <TabsTrigger value="position">Position</TabsTrigger>
            <TabsTrigger value="raw">Raw MIT</TabsTrigger>
          </TabsList>

          <TabsContent value="position" className="space-y-3 pt-3">
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
              Move
            </Button>
          </TabsContent>

          <TabsContent value="raw" className="pt-3">
            <RawControlForm canId={canId} busy={busy} exec={exec} controlMut={controlMut} />
          </TabsContent>
        </Tabs>
      </CardContent>
    </Card>
  )
}

function RawControlForm({
  canId,
  busy,
  exec,
  controlMut,
}: {
  canId: number
  busy: boolean
  exec: (label: string, fn: () => Promise<CommandResponse>) => Promise<void>
  controlMut: ReturnType<typeof useControlMotorMutation>
}) {
  const [pos, setPos] = useState(0)
  const [vel, setVel] = useState(0)
  const [rkp, setRkp] = useState(30)
  const [rkd, setRkd] = useState(1)
  const [trq, setTrq] = useState(0)

  return (
    <div className="space-y-3">
      <div className="grid grid-cols-5 gap-2">
        <LabeledInput label="pos (rad)" value={pos} onChange={setPos} step={0.1} />
        <LabeledInput label="vel (rad/s)" value={vel} onChange={setVel} step={0.5} />
        <LabeledInput label="kp" value={rkp} onChange={setRkp} step={1} />
        <LabeledInput label="kd" value={rkd} onChange={setRkd} step={0.1} />
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
              kp: rkp,
              kd: rkd,
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
      <DialogTrigger>
        <Button variant={variant} disabled={disabled}>
          {label}
        </Button>
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
