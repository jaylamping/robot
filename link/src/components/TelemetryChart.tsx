import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  ResponsiveContainer,
  Tooltip,
} from 'recharts';
import type { MotorSnapshot } from '../stores/telemetry';

interface TelemetryChartProps {
  history: MotorSnapshot[];
  dataKey: keyof MotorSnapshot;
  label: string;
  unit: string;
  color: string;
}

export const TelemetryChart = ({
  history,
  dataKey,
  label,
  unit,
  color,
}: TelemetryChartProps) => {
  const data = history.map((snap, i) => ({
    idx: i,
    value: snap[dataKey] as number,
  }));

  return (
    <div className='rounded-lg border border-zinc-800 bg-zinc-900 p-4'>
      <h3 className='text-sm font-medium text-zinc-400 mb-3'>{label}</h3>
      <ResponsiveContainer width='100%' height={160}>
        <LineChart data={data}>
          <CartesianGrid strokeDasharray='3 3' stroke='#27272a' />
          <XAxis dataKey='idx' tick={false} axisLine={{ stroke: '#3f3f46' }} />
          <YAxis
            width={50}
            tick={{ fill: '#71717a', fontSize: 11 }}
            axisLine={{ stroke: '#3f3f46' }}
            tickFormatter={(v: number) => v.toFixed(1)}
          />
          <Tooltip
            contentStyle={{
              backgroundColor: '#18181b',
              border: '1px solid #3f3f46',
              borderRadius: '6px',
              fontSize: '12px',
            }}
            labelStyle={{ display: 'none' }}
            formatter={(value) => [
              `${Number(value).toFixed(3)} ${unit}`,
              label,
            ]}
          />
          <Line
            type='monotone'
            dataKey='value'
            stroke={color}
            strokeWidth={1.5}
            dot={false}
            isAnimationActive={false}
          />
        </LineChart>
      </ResponsiveContainer>
    </div>
  );
};
