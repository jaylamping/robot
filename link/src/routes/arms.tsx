import { createFileRoute, Link } from '@tanstack/react-router';
import { useEffect, useState } from 'react';
import { toast } from 'sonner';
import type { ArmInfo, CommandResponse, HomeResponse } from '@/lib/api';
import { startSweep, stopSweep } from '@/lib/api';
import { useRobotArmPreflight, useRobotArms } from '@/lib/queries';
import {
  useEnableArmMutation,
  useDisableArmMutation,
  useHomeArmMutation,
  useSetArmPoseMutation,
  useUpdateJointLimitsMutation,
  useUpdateJointHomeMutation,
  useMoveMotorMutation,
  useZeroMotorMutation,
} from '@/lib/mutations/robot';
import { useTelemetryStore } from '@/stores/telemetry';
import { PoseEditor } from '@/components/PoseEditor';
import { PreflightAlert } from '@/components/PreflightAlert';
import { Card, CardContent, CardHeader, CardTitle } from '@/components/ui/card';
import { Button } from '@/components/ui/button';
import { Badge } from '@/components/ui/badge';
import { Input } from '@/components/ui/input';
import { Slider } from '@/components/ui/slider';
import { Separator } from '@/components/ui/separator';
import {
  Dialog,
  DialogContent,
  DialogDescription,
  DialogFooter,
  DialogHeader,
  DialogTitle,
  DialogTrigger,
} from '@/components/ui/dialog';
import {
  LuPower,
  LuPowerOff,
  LuHouse,
  LuSettings,
  LuSave,
  LuCrosshair,
  LuChevronDown,
  LuChevronRight,
  LuLocateFixed,
  LuSquare,
  LuPlay,
} from 'react-icons/lu';

export const Route = createFileRoute('/arms')({
  component: ArmsPage,
});

function ArmsPage() {
  const armsQ = useRobotArms();

  if (armsQ.isPending) {
    return (
      <div className='flex items-center justify-center h-64'>
        <p className='text-muted-foreground text-sm'>
          Loading arm configuration...
        </p>
      </div>
    );
  }

  if (armsQ.isError) {
    return (
      <div className='flex items-center justify-center h-64'>
        <p className='text-destructive text-sm'>{armsQ.error.message}</p>
      </div>
    );
  }

  const arms = armsQ.data ?? [];
  if (arms.length === 0) {
    return (
      <div className='flex flex-col items-center justify-center h-64 text-center'>
        <h2 className='text-lg font-medium mb-1'>No arms configured</h2>
        <p className='text-sm text-muted-foreground'>
          Add arm joint CAN IDs to{' '}
          <code className='text-xs bg-muted px-1 py-0.5 rounded'>
            config/robot.yaml
          </code>
          .
        </p>
      </div>
    );
  }

  return (
    <div>
      <h2 className='text-xl font-semibold mb-6'>Arm Control</h2>
      <div className='space-y-6'>
        {arms.map((arm) => (
          <ArmPanel key={arm.side} arm={arm} />
        ))}
      </div>
    </div>
  );
}

function ArmPanel({ arm }: { arm: ArmInfo }) {
  const [busy, setBusy] = useState(false);
  const [hidePreflightBanner, setHidePreflightBanner] = useState(false);
  const [homeResult, setHomeResult] = useState<HomeResponse | null>(null);
  const motors = useTelemetryStore((s) => s.motors);

  const preflightQ = useRobotArmPreflight(arm.side);
  const preflight = preflightQ.data;
  const enableArmMut = useEnableArmMutation();
  const disableArmMut = useDisableArmMutation();
  const homeMut = useHomeArmMutation();
  const setPoseMut = useSetArmPoseMutation();

  const onlineJoints = arm.joints.filter(
    (j) => j.can_id != null && motors[j.can_id]?.online,
  );
  const totalJoints = arm.joints.filter((j) => j.can_id != null).length;

  const section = arm.side === 'left' ? 'arm_left' : 'arm_right';

  async function exec(label: string, fn: () => Promise<CommandResponse>) {
    setBusy(true);
    try {
      const res = await fn();
      if (res.success) {
        toast.success(`${arm.side} arm: ${label}`, {
          description: res.error ?? undefined,
        });
      } else {
        toast.error(`${arm.side} arm: ${label} failed`, {
          description: res.error,
        });
      }
    } catch (e) {
      toast.error(`${arm.side} arm: ${label} failed`, {
        description: e instanceof Error ? e.message : String(e),
      });
    } finally {
      setBusy(false);
    }
  }

  async function handleHome() {
    setBusy(true);
    try {
      const result = await homeMut.mutateAsync({
        side: arm.side,
        override: false,
      });
      setHomeResult(result);
      setHidePreflightBanner(false);
      if (result.success) {
        const jointSummary = result.joints
          .map(
            (j) =>
              `${formatJointName(j.joint_name)}: ${j.status.replace(/_/g, ' ')}`,
          )
          .join('\n');
        toast.success(`${arm.side} arm: Homed`, {
          description: result.error
            ? `${result.error}\n${jointSummary}`
            : jointSummary,
          duration: 8000,
        });
      } else if (result.preflight) {
        toast.error(`${arm.side} arm: Pre-flight check failed`, {
          description: result.error,
        });
      } else {
        toast.error(`${arm.side} arm: Homing failed`, {
          description: result.error,
        });
      }
    } catch (e) {
      toast.error(`${arm.side} arm: Homing failed`, {
        description: e instanceof Error ? e.message : String(e),
      });
    } finally {
      setBusy(false);
    }
  }

  return (
    <Card>
      <CardHeader>
        <div className='flex items-center justify-between'>
          <div>
            <CardTitle className='capitalize'>{arm.side} Arm</CardTitle>
            <p className='text-xs text-muted-foreground mt-1'>
              {onlineJoints.length}/{totalJoints} joints online
            </p>
          </div>
          <div className='flex gap-2'>
            <ConfirmAction
              label='Enable All'
              icon={<LuPower className='size-4' />}
              description={`Enable all ${totalJoints} joints on the ${arm.side} arm. Motors will energize.`}
              disabled={busy}
              onConfirm={() =>
                exec('Enable All', () => enableArmMut.mutateAsync(arm.side))
              }
            />
            <ConfirmAction
              label='Disable All'
              icon={<LuPowerOff className='size-4' />}
              description={`Disable all joints on the ${arm.side} arm. Motors will de-energize and may drop.`}
              variant='destructive'
              disabled={busy}
              onConfirm={() =>
                exec('Disable All', () => disableArmMut.mutateAsync(arm.side))
              }
            />
            <ConfirmAction
              label={busy ? 'Homing...' : 'Home'}
              icon={<LuHouse className='size-4' />}
              description={`This will enable motors and move all ${arm.side} arm joints to their configured home positions. Ensure the arm is clear of obstacles and you can reach E-STOP.`}
              disabled={busy}
              onConfirm={handleHome}
            />
          </div>
        </div>
      </CardHeader>
      <CardContent className='space-y-4'>
        {preflight && !preflight.pass && !hidePreflightBanner && (
          <PreflightAlert
            side={arm.side}
            preflight={preflight}
            onDismiss={() => setHidePreflightBanner(true)}
          />
        )}

        {homeResult && homeResult.joints.length > 0 && (
          <div className='rounded-md border bg-muted/30 p-3 mb-2'>
            <h4 className='text-xs font-semibold text-muted-foreground uppercase tracking-wider mb-2'>
              Last Homing Result
            </h4>
            <div className='space-y-1'>
              {homeResult.joints.map((j) => (
                <div
                  key={j.joint_name}
                  className='flex items-center gap-2 text-xs'
                >
                  <HomingStatusDot status={j.status} />
                  <span className='font-medium w-28 truncate'>
                    {formatJointName(j.joint_name)}
                  </span>
                  <Badge variant='secondary' className='text-[10px] h-4'>
                    {j.status.replace(/_/g, ' ')}
                  </Badge>
                  <span className='text-muted-foreground ml-auto font-mono'>
                    {(j.error_rad * (180 / Math.PI)).toFixed(1)}° err
                  </span>
                  <span className='text-muted-foreground font-mono'>
                    {j.duration_ms}ms
                  </span>
                </div>
              ))}
            </div>
          </div>
        )}

        {arm.joints.map((joint) => (
          <JointSlider key={joint.name} joint={joint} section={section} side={arm.side} />
        ))}

        <Separator />

        <PoseEditor
          armSide={arm.side}
          joints={arm.joints}
          onApply={(pose) =>
            exec('Set Pose', () =>
              setPoseMut.mutateAsync({
                side: arm.side,
                pose: { joints: pose },
              }),
            )
          }
        />
      </CardContent>
    </Card>
  );
}

function JointSlider({
  joint,
  section,
  side,
}: {
  joint: ArmInfo['joints'][number];
  section: string;
  side: string;
}) {
  const motor = useTelemetryStore((s) =>
    joint.can_id != null ? s.motors[joint.can_id] : undefined,
  );
  const [expanded, setExpanded] = useState(false);
  const [editMin, setEditMin] = useState('');
  const [editMax, setEditMax] = useState('');
  const [editHome, setEditHome] = useState('');
  const [saving, setSaving] = useState(false);
  const [dragging, setDragging] = useState(false);
  const [dragDeg, setDragDeg] = useState<number | null>(null);
  const [sweeping, setSweeping] = useState(false);
  const [sweepSpeed, setSweepSpeed] = useState(20);
  const limitsMut = useUpdateJointLimitsMutation();
  const homeMut = useUpdateJointHomeMutation();
  const moveMut = useMoveMotorMutation();
  const zeroMut = useZeroMotorMutation();

  const minDeg = (joint.limits[0] * 180) / Math.PI;
  const maxDeg = (joint.limits[1] * 180) / Math.PI;
  const currentDeg = motor ? (motor.angle_rad * 180) / Math.PI : null;
  const isOnline = motor?.online ?? false;
  const canMove = isOnline && joint.can_id != null;

  const homeDeg = (joint.home_rad * 180) / Math.PI;
  const homeError =
    motor?.home_error_rad != null
      ? motor.home_error_rad * (180 / Math.PI)
      : null;
  const atHome = motor?.at_home ?? false;

  const rawDisplayDeg = dragging && dragDeg != null ? dragDeg : currentDeg;
  const displayDeg =
    rawDisplayDeg != null
      ? Math.max(minDeg, Math.min(maxDeg, rawDisplayDeg))
      : null;

  const limitProximity = (() => {
    if (currentDeg == null) return 'normal';
    const marginDeg = 10;
    if (currentDeg <= minDeg || currentDeg >= maxDeg) return 'at_limit';
    if (currentDeg - minDeg < marginDeg || maxDeg - currentDeg < marginDeg)
      return 'near_limit';
    return 'normal';
  })();

  const sliderTrackClass =
    limitProximity === 'at_limit'
      ? '[&_[data-slider-range]]:bg-red-500'
      : limitProximity === 'near_limit'
        ? '[&_[data-slider-range]]:bg-amber-500'
        : dragging
          ? '[&_[data-slider-range]]:bg-blue-500'
          : '';

  const handleSliderChange = (val: number | readonly number[]) => {
    const deg = Array.isArray(val) ? val[0] : val;
    setDragging(true);
    setDragDeg(deg);
  };

  const handleSliderCommit = async (val: number | readonly number[]) => {
    const deg = Math.max(
      minDeg,
      Math.min(maxDeg, Array.isArray(val) ? val[0] : val),
    );
    setDragging(false);
    setDragDeg(null);
    if (!canMove) return;
    const rad = (deg * Math.PI) / 180;
    try {
      const res = await moveMut.mutateAsync({
        id: joint.can_id!,
        position_rad: rad,
        kp: 5.0,
        kd: 0.5,
      });
      if (!res.success) {
        toast.error(`Move failed for ${formatJointName(joint.name)}`, {
          description: res.error,
        });
      }
    } catch (e) {
      toast.error(`Move failed for ${formatJointName(joint.name)}`, {
        description: e instanceof Error ? e.message : String(e),
      });
    }
  };

  const handleExpand = () => {
    if (!expanded) {
      setEditMin(minDeg.toFixed(1));
      setEditMax(maxDeg.toFixed(1));
      setEditHome(homeDeg.toFixed(1));
    }
    setExpanded(!expanded);
  };

  const handleSaveLimits = async () => {
    const newMin = (parseFloat(editMin) * Math.PI) / 180;
    const newMax = (parseFloat(editMax) * Math.PI) / 180;
    if (isNaN(newMin) || isNaN(newMax) || newMin >= newMax) {
      toast.error('Invalid limits: min must be less than max');
      return;
    }
    setSaving(true);
    try {
      const res = await limitsMut.mutateAsync({
        section,
        joint: joint.name,
        minRad: newMin,
        maxRad: newMax,
      });
      if (res.success) {
        toast.success(`Limits saved for ${formatJointName(joint.name)}`);
      } else {
        toast.error('Failed to save limits', { description: res.error });
      }
    } catch (e) {
      toast.error('Failed to save limits', {
        description: e instanceof Error ? e.message : String(e),
      });
    } finally {
      setSaving(false);
    }
  };

  const handleSaveHome = async () => {
    const newHome = (parseFloat(editHome) * Math.PI) / 180;
    if (isNaN(newHome)) {
      toast.error('Invalid home angle');
      return;
    }
    setSaving(true);
    try {
      const res = await homeMut.mutateAsync({
        section,
        joint: joint.name,
        homeRad: newHome,
      });
      if (res.success) {
        toast.success(`Home saved for ${formatJointName(joint.name)}`);
      } else {
        toast.error('Failed to save home', { description: res.error });
      }
    } catch (e) {
      toast.error('Failed to save home', {
        description: e instanceof Error ? e.message : String(e),
      });
    } finally {
      setSaving(false);
    }
  };

  const handleSetCurrentAsHome = async () => {
    const posDeg = currentDeg != null ? currentDeg.toFixed(1) : '?';
    if (
      !confirm(
        `Set current position (${posDeg}°) as home for ${formatJointName(joint.name)}?\n\n` +
          `This saves the motor's current encoder reading as the home target in robot.yaml.`,
      )
    )
      return;
    setSaving(true);
    try {
      const res = await homeMut.mutateAsync({
        section,
        joint: joint.name,
        setCurrent: true,
      });
      if (res.success) {
        toast.success(
          `Home set to current position for ${formatJointName(joint.name)}`,
          {
            description:
              res.angle_rad != null
                ? `${((res.angle_rad * 180) / Math.PI).toFixed(1)}°`
                : undefined,
          },
        );
      } else {
        toast.error('Failed to set home', { description: res.error });
      }
    } catch (e) {
      toast.error('Failed to set home', {
        description: e instanceof Error ? e.message : String(e),
      });
    } finally {
      setSaving(false);
    }
  };

  const handleZeroEncoder = async () => {
    if (joint.can_id == null) return;
    if (
      !confirm(
        `Zero encoder for ${formatJointName(joint.name)} (CAN ${joint.can_id})?\n\n` +
          `This redefines the motor's current physical position as 0°. ` +
          `All position commands and limits are relative to this zero point.\n\n` +
          `Make sure the joint is at the position you want to be 0°.`,
      )
    )
      return;
    setSaving(true);
    try {
      const res = await zeroMut.mutateAsync(joint.can_id);
      if (res.success) {
        toast.success(`Encoder zeroed for ${formatJointName(joint.name)}`, {
          description: 'Current physical position is now 0°',
        });
      } else {
        toast.error('Zero failed', { description: res.error });
      }
    } catch (e) {
      toast.error('Zero encoder failed', {
        description: e instanceof Error ? e.message : String(e),
      });
    } finally {
      setSaving(false);
    }
  };

  const handleStartSweep = async () => {
    setSweeping(true);
    try {
      const res = await startSweep(side, joint.name, sweepSpeed);
      if (!res.success) {
        setSweeping(false);
        toast.error(`Sweep failed for ${formatJointName(joint.name)}`, {
          description: res.error,
        });
      }
    } catch (e) {
      setSweeping(false);
      toast.error(`Sweep error for ${formatJointName(joint.name)}`, {
        description: e instanceof Error ? e.message : String(e),
      });
    }
  };

  const handleStopSweep = async () => {
    setSweeping(false);
    try {
      await stopSweep(side, joint.name);
      toast.info(`Sweep stopping for ${formatJointName(joint.name)}`, {
        description: 'Finishing current pass, then returning to home.',
      });
    } catch (e) {
      toast.error(`Stop sweep failed for ${formatJointName(joint.name)}`, {
        description: e instanceof Error ? e.message : String(e),
      });
    }
  };

  useEffect(() => {
    if (!sweeping) return;

    // On page unload (refresh/close), sendBeacon is guaranteed to fire
    // even when fetch/XHR requests are cancelled by the browser.
    const handleUnload = () => {
      navigator.sendBeacon(`/api/arms/${side}/joints/${joint.name}/sweep/stop`);
    };
    window.addEventListener('beforeunload', handleUnload);

    return () => {
      window.removeEventListener('beforeunload', handleUnload);
      // Also fire on React unmount (navigation within the SPA).
      stopSweep(side, joint.name).catch(() => {});
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [sweeping, side, joint.name]);

  // Live-update sweep speed: debounce 300ms so we don't spam the API
  // on every keystroke, then send a new start command at the new speed.
  const [prevSpeed, setPrevSpeed] = useState(sweepSpeed);
  useEffect(() => {
    if (!sweeping || sweepSpeed === prevSpeed) return;
    const timer = setTimeout(() => {
      setPrevSpeed(sweepSpeed);
      startSweep(side, joint.name, sweepSpeed).catch(() => {});
    }, 300);
    return () => clearTimeout(timer);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [sweepSpeed, sweeping, side, joint.name]);

  const handleZeroAndSetHome = async () => {
    if (joint.can_id == null) return;
    if (
      !confirm(
        `Zero encoder AND set home for ${formatJointName(joint.name)} (CAN ${joint.can_id})?\n\n` +
          `This will:\n` +
          `1. Redefine the motor's current physical position as 0°\n` +
          `2. Save 0° as the home position in robot.yaml\n\n` +
          `Make sure the joint is at the position you want to be both zero and home.`,
      )
    )
      return;
    setSaving(true);
    try {
      const zeroRes = await zeroMut.mutateAsync(joint.can_id);
      if (!zeroRes.success) {
        toast.error('Zero failed', { description: zeroRes.error });
        setSaving(false);
        return;
      }
      const homeRes = await homeMut.mutateAsync({
        section,
        joint: joint.name,
        homeRad: 0,
      });
      if (homeRes.success) {
        toast.success(
          `${formatJointName(joint.name)}: zeroed & home set to 0°`,
          {
            description:
              'Current physical position is now 0° and saved as home',
          },
        );
      } else {
        toast.error('Home save failed after zero', {
          description: homeRes.error,
        });
      }
    } catch (e) {
      toast.error('Zero & set home failed', {
        description: e instanceof Error ? e.message : String(e),
      });
    } finally {
      setSaving(false);
    }
  };

  return (
    <div className='space-y-1'>
      <div className='flex items-center justify-between'>
        <div className='flex items-center gap-2'>
          <span className='text-sm font-medium'>
            {formatJointName(joint.name)}
          </span>
          {joint.can_id != null && (
            <Link
              to='/motor/$id'
              params={{ id: String(joint.can_id) }}
              className='text-xs text-muted-foreground hover:text-foreground'
            >
              CAN {joint.can_id}
            </Link>
          )}
          {atHome && (
            <Badge className='bg-emerald-500/10 text-emerald-400 border-emerald-500/20 text-[10px] h-4'>
              Home
            </Badge>
          )}
          {homeError != null && !atHome && isOnline && (
            <span className='text-[10px] text-amber-400 font-mono'>
              {homeError.toFixed(1)}° off
            </span>
          )}
        </div>
        <div className='flex items-center gap-2'>
          {dragging && dragDeg != null ? (
            <span className='text-xs font-mono text-blue-400'>
              {dragDeg.toFixed(1)}°
            </span>
          ) : currentDeg != null ? (
            <span className='text-xs font-mono text-muted-foreground'>
              {currentDeg.toFixed(1)}°
            </span>
          ) : null}
          {limitProximity === 'at_limit' && (
            <Badge variant='destructive' className='text-[10px] h-4'>
              At limit
            </Badge>
          )}
          {limitProximity === 'near_limit' && (
            <Badge className='bg-amber-500/10 text-amber-400 border-amber-500/20 text-[10px] h-4'>
              Near limit
            </Badge>
          )}
          <Badge
            variant={isOnline ? 'default' : 'secondary'}
            className='text-xs'
          >
            {isOnline ? 'Online' : joint.can_id == null ? 'N/A' : 'Offline'}
          </Badge>
          <button
            onClick={handleExpand}
            className='text-muted-foreground hover:text-foreground p-0.5'
          >
            {expanded ? (
              <LuChevronDown className='size-3.5' />
            ) : (
              <LuChevronRight className='size-3.5' />
            )}
          </button>
        </div>
      </div>
      <div className={`relative ${sliderTrackClass}`}>
        <Slider
          value={displayDeg != null ? [displayDeg] : [0]}
          min={minDeg}
          max={maxDeg}
          step={0.5}
          disabled={!canMove}
          onValueChange={handleSliderChange}
          onValueCommitted={handleSliderCommit}
        />
        <div className='mt-0.5 flex justify-between text-[10px] text-muted-foreground font-mono'>
          <span>{minDeg.toFixed(0)}°</span>
          <span>home: {homeDeg.toFixed(0)}°</span>
          <span>{maxDeg.toFixed(0)}°</span>
        </div>
      </div>

      {expanded && (
        <div className='ml-2 mt-2 p-3 rounded-md border bg-muted/20 space-y-3'>
          <div>
            <h5 className='text-xs font-semibold text-muted-foreground uppercase tracking-wider mb-1.5 flex items-center gap-1'>
              <LuLocateFixed className='size-3' /> Encoder Zero
            </h5>
            <p className='text-[10px] text-muted-foreground mb-2'>
              Position the joint where you want 0° to be, then zero the encoder.
              {currentDeg != null && (
                <>
                  {' '}
                  Raw encoder reads <strong>{currentDeg.toFixed(1)}°</strong>.
                </>
              )}
            </p>
            <div className='flex gap-2'>
              <Button
                variant='outline'
                size='sm'
                onClick={handleZeroEncoder}
                disabled={saving || !isOnline}
                className='gap-1 h-7 text-xs'
                title="Set the encoder's current position as 0 rad"
              >
                <LuLocateFixed className='size-3' />
                Zero Encoder
              </Button>
              <Button
                variant='default'
                size='sm'
                onClick={handleZeroAndSetHome}
                disabled={saving || !isOnline}
                className='gap-1 h-7 text-xs'
                title='Zero the encoder AND set home to 0° in one step'
              >
                <LuHouse className='size-3' />
                Zero & Set Home
              </Button>
            </div>
          </div>

          <Separator />

          <div>
            <div className='flex items-center justify-between mb-1.5'>
              <h5 className='text-xs font-semibold text-muted-foreground uppercase tracking-wider flex items-center gap-1'>
                <LuSettings className='size-3' /> Joint Limits
              </h5>
              <div className='flex items-center gap-1.5'>
                <div className='flex items-center gap-1'>
                  <Input
                    type='number'
                    value={sweepSpeed}
                    min={1}
                    max={80}
                    step={1}
                    onChange={(e) => {
                      const v = Math.round(parseFloat(e.target.value));
                      if (!isNaN(v)) setSweepSpeed(Math.max(1, Math.min(80, v)));
                    }}
                    className='h-6 w-14 text-[10px] px-1.5 text-center'
                    title='Sweep speed in °/sec (1–80)'
                  />
                  <span className='text-[10px] text-muted-foreground'>°/s</span>
                </div>
                {!sweeping ? (
                  <Button
                    variant='outline'
                    size='sm'
                    onClick={handleStartSweep}
                    disabled={saving || !isOnline}
                    className='gap-1 h-6 text-[10px] px-2'
                    title='Continuously sweep joint between limits'
                  >
                    <LuPlay className='size-3' />
                    Sweep
                  </Button>
                ) : (
                  <Button
                    variant='destructive'
                    size='sm'
                    onClick={handleStopSweep}
                    className='gap-1 h-6 text-[10px] px-2'
                    title='Stop sweep — finishes current pass then returns to home'
                  >
                    <LuSquare className='size-3' />
                    Stop
                  </Button>
                )}
              </div>
            </div>
            <div className='flex items-center gap-2'>
              <div className='flex-1'>
                <label className='text-[10px] text-muted-foreground'>
                  Min (°)
                </label>
                <Input
                  type='number'
                  value={editMin}
                  onChange={(e) => setEditMin(e.target.value)}
                  className='h-7 text-xs'
                />
              </div>
              <div className='flex-1'>
                <label className='text-[10px] text-muted-foreground'>
                  Max (°)
                </label>
                <Input
                  type='number'
                  value={editMax}
                  onChange={(e) => setEditMax(e.target.value)}
                  className='h-7 text-xs'
                />
              </div>
              <Button
                variant='outline'
                size='sm'
                onClick={handleSaveLimits}
                disabled={saving}
                className='gap-1 h-7 mt-3'
              >
                <LuSave className='size-3' />
                Save
              </Button>
            </div>
          </div>

          <Separator />

          <div>
            <h5 className='text-xs font-semibold text-muted-foreground uppercase tracking-wider mb-1.5 flex items-center gap-1'>
              <LuHouse className='size-3' /> Home Position
            </h5>
            <div className='flex items-center gap-2'>
              <div className='flex-1'>
                <label className='text-[10px] text-muted-foreground'>
                  Home (°)
                </label>
                <Input
                  type='number'
                  value={editHome}
                  onChange={(e) => setEditHome(e.target.value)}
                  className='h-7 text-xs'
                />
              </div>
              <Button
                variant='outline'
                size='sm'
                onClick={handleSaveHome}
                disabled={saving}
                className='gap-1 h-7 mt-3'
              >
                <LuSave className='size-3' />
                Save
              </Button>
              <Button
                variant='secondary'
                size='sm'
                onClick={handleSetCurrentAsHome}
                disabled={saving || !isOnline}
                className='gap-1 h-7 mt-3'
                title="Set the motor's current position as the new home"
              >
                <LuCrosshair className='size-3' />
                Set Current
              </Button>
            </div>
          </div>

        </div>
      )}
    </div>
  );
}

function ConfirmAction({
  label,
  icon,
  description,
  variant = 'outline',
  disabled,
  onConfirm,
}: {
  label: string;
  icon: React.ReactNode;
  description: string;
  variant?: 'outline' | 'destructive';
  disabled?: boolean;
  onConfirm: () => void;
}) {
  const [open, setOpen] = useState(false);

  return (
    <Dialog open={open} onOpenChange={setOpen}>
      <DialogTrigger
        render={<Button variant={variant} size='sm' disabled={disabled} />}
      >
        {icon}
        <span className='hidden sm:inline'>{label}</span>
      </DialogTrigger>
      <DialogContent>
        <DialogHeader>
          <DialogTitle>Confirm: {label}</DialogTitle>
          <DialogDescription>{description}</DialogDescription>
        </DialogHeader>
        <DialogFooter>
          <Button variant='outline' onClick={() => setOpen(false)}>
            Cancel
          </Button>
          <Button
            variant={variant === 'destructive' ? 'destructive' : 'default'}
            onClick={() => {
              setOpen(false);
              onConfirm();
            }}
          >
            {label}
          </Button>
        </DialogFooter>
      </DialogContent>
    </Dialog>
  );
}

function HomingStatusDot({ status }: { status: string }) {
  const color =
    status === 'already_home' || status === 'homed'
      ? 'bg-emerald-400'
      : status === 'stalled_but_homed'
        ? 'bg-amber-400'
        : status === 'error' || status === 'timed_out'
          ? 'bg-red-400'
          : 'bg-zinc-500';
  return <span className={`size-1.5 rounded-full shrink-0 ${color}`} />;
}

function formatJointName(name: string): string {
  return name
    .split('_')
    .map((w) => w.charAt(0).toUpperCase() + w.slice(1))
    .join(' ');
}
