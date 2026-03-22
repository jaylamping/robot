import type { MotorInfo } from '../lib/api';
import { useTelemetryStore, type MotorSnapshot } from '../stores/telemetry';

interface MotorCardProps {
  motor: MotorInfo;
  onClick?: () => void;
}

export const MotorCard = ({ motor, onClick }: MotorCardProps) => {
  const live = useTelemetryStore((s) => s.motors[motor.can_id]) as
    | MotorSnapshot
    | undefined;

  const isOnline = live?.online ?? motor.online;
  const hasFaults = (live?.faults?.length ?? 0) > 0;
  const highTemp = (live?.temperature_c ?? 0) > 60;

  const borderColor = hasFaults
    ? 'border-red-500/50'
    : highTemp
      ? 'border-amber-500/50'
      : isOnline
        ? 'border-emerald-500/40'
        : 'border-zinc-700';

  const statusDot = hasFaults
    ? 'bg-red-500'
    : highTemp
      ? 'bg-amber-500'
      : isOnline
        ? 'bg-emerald-500'
        : 'bg-zinc-600';

  const statusLabel = hasFaults
    ? 'Fault'
    : highTemp
      ? 'Hot'
      : isOnline
        ? 'Online'
        : 'Offline';

  return (
    <button
      onClick={onClick}
      className={`w-full text-left p-4 rounded-lg border ${borderColor} bg-zinc-900 hover:bg-zinc-800/80 transition-colors cursor-pointer`}
    >
      <div className='flex items-center justify-between mb-3'>
        <h3 className='text-sm font-medium text-zinc-100'>
          {formatJointName(motor.joint_name)}
        </h3>
        <div className='flex items-center gap-1.5'>
          <span className={`inline-block w-2 h-2 rounded-full ${statusDot}`} />
          <span className='text-xs text-zinc-500'>{statusLabel}</span>
        </div>
      </div>

      <div className='space-y-1.5 text-xs text-zinc-400'>
        <div className='flex justify-between'>
          <span>CAN ID</span>
          <span className='text-zinc-300 font-mono'>{motor.can_id}</span>
        </div>
        <div className='flex justify-between'>
          <span>Actuator</span>
          <span className='text-zinc-300 uppercase'>{motor.actuator_type}</span>
        </div>
        {live && isOnline ? (
          <>
            <div className='flex justify-between'>
              <span>Position</span>
              <span className='text-zinc-300 font-mono'>
                {((live.angle_rad * 180) / Math.PI).toFixed(1)}°
              </span>
            </div>
            <div className='flex justify-between'>
              <span>Velocity</span>
              <span className='text-zinc-300 font-mono'>
                {live.velocity_rads.toFixed(2)} rad/s
              </span>
            </div>
            <div className='flex justify-between'>
              <span>Torque</span>
              <span className='text-zinc-300 font-mono'>
                {live.torque_nm.toFixed(2)} N·m
              </span>
            </div>
            <div className='flex justify-between'>
              <span>Temp</span>
              <span
                className={`font-mono ${highTemp ? 'text-amber-400' : 'text-zinc-300'}`}
              >
                {live.temperature_c.toFixed(1)} °C
              </span>
            </div>
          </>
        ) : (
          <div className='flex justify-between'>
            <span>Limits</span>
            <span className='text-zinc-300 font-mono'>
              {((motor.limits[0] * 180) / Math.PI).toFixed(0)}° /{' '}
              {((motor.limits[1] * 180) / Math.PI).toFixed(0)}°
            </span>
          </div>
        )}
      </div>

      {hasFaults && live && (
        <div className='mt-2 pt-2 border-t border-zinc-800'>
          {live.faults.map((f, i) => (
            <p key={i} className='text-xs text-red-400 font-mono truncate'>
              {f}
            </p>
          ))}
        </div>
      )}
    </button>
  );
};

function formatJointName(name: string): string {
  return name
    .split('_')
    .map((w) => w.charAt(0).toUpperCase() + w.slice(1))
    .join(' ');
}
