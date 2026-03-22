import { useState } from 'react';
import {
  enableMotor,
  disableMotor,
  zeroMotor,
  moveMotor,
  controlMotor,
  type CommandResponse,
} from '../lib/api';

interface MotorControlProps {
  canId: number;
  currentAngleRad: number;
}

export const MotorControl = ({ canId, currentAngleRad }: MotorControlProps) => {
  const [positionDeg, setPositionDeg] = useState(
    (currentAngleRad * 180) / Math.PI
  );
  const [kp, setKp] = useState(30);
  const [kd, setKd] = useState(1);
  const [lastResult, setLastResult] = useState<CommandResponse | null>(null);
  const [busy, setBusy] = useState(false);

  async function exec(fn: () => Promise<CommandResponse>) {
    setBusy(true);
    try {
      const res = await fn();
      setLastResult(res);
    } catch (e) {
      setLastResult({
        success: false,
        error: e instanceof Error ? e.message : String(e),
      });
    } finally {
      setBusy(false);
    }
  }

  return (
    <div className='rounded-lg border border-zinc-800 bg-zinc-900 p-4 space-y-4'>
      <h3 className='text-sm font-medium text-zinc-400'>Controls</h3>

      <div className='flex flex-wrap gap-2'>
        <ActionButton
          label='Enable'
          disabled={busy}
          onClick={() => exec(() => enableMotor(canId))}
          variant='green'
        />
        <ActionButton
          label='Disable'
          disabled={busy}
          onClick={() => exec(() => disableMotor(canId))}
          variant='red'
        />
        <ActionButton
          label='Set Zero'
          disabled={busy}
          onClick={() => exec(() => zeroMotor(canId))}
          variant='default'
        />
      </div>

      <div className='border-t border-zinc-800 pt-4'>
        <p className='text-xs text-zinc-500 mb-2'>Position Command</p>
        <div className='grid grid-cols-3 gap-2 mb-2'>
          <div>
            <label className='text-xs text-zinc-500'>Target (°)</label>
            <input
              type='number'
              value={positionDeg}
              onChange={(e) => setPositionDeg(Number(e.target.value))}
              className='w-full bg-zinc-800 border border-zinc-700 rounded px-2 py-1 text-sm text-zinc-100 font-mono'
              step={1}
            />
          </div>
          <div>
            <label className='text-xs text-zinc-500'>kp</label>
            <input
              type='number'
              value={kp}
              onChange={(e) => setKp(Number(e.target.value))}
              className='w-full bg-zinc-800 border border-zinc-700 rounded px-2 py-1 text-sm text-zinc-100 font-mono'
              step={1}
              min={0}
              max={5000}
            />
          </div>
          <div>
            <label className='text-xs text-zinc-500'>kd</label>
            <input
              type='number'
              value={kd}
              onChange={(e) => setKd(Number(e.target.value))}
              className='w-full bg-zinc-800 border border-zinc-700 rounded px-2 py-1 text-sm text-zinc-100 font-mono'
              step={0.1}
              min={0}
              max={100}
            />
          </div>
        </div>
        <div className='mb-2'>
          <input
            type='range'
            min={-180}
            max={180}
            step={0.5}
            value={positionDeg}
            onChange={(e) => setPositionDeg(Number(e.target.value))}
            className='w-full accent-blue-500'
          />
        </div>
        <ActionButton
          label='Move'
          disabled={busy}
          onClick={() =>
            exec(() => moveMotor(canId, (positionDeg * Math.PI) / 180, kp, kd))
          }
          variant='blue'
        />
      </div>

      <div className='border-t border-zinc-800 pt-4'>
        <p className='text-xs text-zinc-500 mb-2'>Raw MIT Control</p>
        <RawControlForm canId={canId} busy={busy} exec={exec} />
      </div>

      {lastResult && (
        <div
          className={`text-xs font-mono p-2 rounded ${
            lastResult.success
              ? 'bg-emerald-950/50 text-emerald-300'
              : 'bg-red-950/50 text-red-300'
          }`}
        >
          {lastResult.success
            ? `OK — pos: ${lastResult.angle_rad?.toFixed(3)} rad, vel: ${lastResult.velocity_rads?.toFixed(3)} rad/s, trq: ${lastResult.torque_nm?.toFixed(3)} N·m`
            : `Error: ${lastResult.error}`}
        </div>
      )}
    </div>
  );
};

function RawControlForm({
  canId,
  busy,
  exec,
}: {
  canId: number;
  busy: boolean;
  exec: (fn: () => Promise<CommandResponse>) => Promise<void>;
}) {
  const [pos, setPos] = useState(0);
  const [vel, setVel] = useState(0);
  const [rkp, setRkp] = useState(30);
  const [rkd, setRkd] = useState(1);
  const [trq, setTrq] = useState(0);

  return (
    <div>
      <div className='grid grid-cols-5 gap-2 mb-2'>
        {[
          { label: 'pos (rad)', value: pos, set: setPos, step: 0.1 },
          { label: 'vel (rad/s)', value: vel, set: setVel, step: 0.5 },
          { label: 'kp', value: rkp, set: setRkp, step: 1 },
          { label: 'kd', value: rkd, set: setRkd, step: 0.1 },
          { label: 'trq (N·m)', value: trq, set: setTrq, step: 0.5 },
        ].map((f) => (
          <div key={f.label}>
            <label className='text-xs text-zinc-500'>{f.label}</label>
            <input
              type='number'
              value={f.value}
              onChange={(e) => f.set(Number(e.target.value))}
              className='w-full bg-zinc-800 border border-zinc-700 rounded px-2 py-1 text-sm text-zinc-100 font-mono'
              step={f.step}
            />
          </div>
        ))}
      </div>
      <ActionButton
        label='Send'
        disabled={busy}
        onClick={() => exec(() => controlMotor(canId, pos, vel, rkp, rkd, trq))}
        variant='default'
      />
    </div>
  );
}

function ActionButton({
  label,
  onClick,
  disabled,
  variant = 'default',
}: {
  label: string;
  onClick: () => void;
  disabled?: boolean;
  variant?: 'default' | 'green' | 'red' | 'blue';
}) {
  const colors = {
    default: 'bg-zinc-800 hover:bg-zinc-700 text-zinc-200',
    green: 'bg-emerald-900/60 hover:bg-emerald-800/60 text-emerald-300',
    red: 'bg-red-900/60 hover:bg-red-800/60 text-red-300',
    blue: 'bg-blue-900/60 hover:bg-blue-800/60 text-blue-300',
  };

  return (
    <button
      onClick={onClick}
      disabled={disabled}
      className={`px-3 py-1.5 rounded text-sm font-medium transition-colors disabled:opacity-40 disabled:cursor-not-allowed ${colors[variant]}`}
    >
      {label}
    </button>
  );
}
