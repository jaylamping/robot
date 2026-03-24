import { useState, useRef } from 'react'
import uPlot from 'uplot'
import UplotReact from 'uplot-react'
import 'uplot/dist/uPlot.min.css'
import { Card, CardContent, CardHeader, CardTitle, CardAction } from '@/components/ui/card'
import { Button } from '@/components/ui/button'
import type { MotorSnapshot } from '@/stores/telemetry'

interface TelemetryChartProps {
  history: MotorSnapshot[]
  dataKey: keyof MotorSnapshot
  label: string
  unit: string
  color: string
  rateHz?: number
}

export function TelemetryChart({
  history,
  dataKey,
  label,
  unit,
  color,
  rateHz = 20,
}: TelemetryChartProps) {
  const [paused, setPaused] = useState(false)
  const [frozen, setFrozen] = useState<MotorSnapshot[]>([])
  const chartRef = useRef<uPlot | null>(null)

  const displayData = paused ? frozen : history
  const totalSamples = displayData.length

  const uData: uPlot.AlignedData = (() => {
    const times = new Float64Array(totalSamples)
    const values = new Float64Array(totalSamples)
    for (let i = 0; i < totalSamples; i++) {
      times[i] = -((totalSamples - 1 - i) / rateHz)
      values[i] = displayData[i][dataKey] as number
    }
    return [times, values]
  })()

  const opts: uPlot.Options = {
    width: 100,
    height: 200,
    cursor: {
      drag: { x: false, y: false },
    },
    scales: {
      x: { time: false },
    },
    axes: [
      {
        stroke: 'hsl(var(--muted-foreground))',
        grid: { stroke: 'hsl(var(--border))', dash: [3, 3] },
        ticks: { stroke: 'hsl(var(--border))' },
        font: '10px sans-serif',
        values: (_u: uPlot, vals: number[]) => vals.map((v) => `${v.toFixed(0)}s`),
      },
      {
        stroke: 'hsl(var(--muted-foreground))',
        grid: { stroke: 'hsl(var(--border))', dash: [3, 3] },
        ticks: { stroke: 'hsl(var(--border))' },
        font: '11px sans-serif',
        size: 55,
        values: (_u: uPlot, vals: number[]) => vals.map((v) => v.toFixed(1)),
      },
    ],
    series: [
      {},
      {
        label: unit,
        stroke: color,
        width: 1.5,
        points: { show: false },
      },
    ],
    plugins: [tooltipPlugin(label, unit)],
  }

  function onCreate(chart: uPlot) {
    chartRef.current = chart

    const ro = new ResizeObserver((entries) => {
      for (const entry of entries) {
        const width = entry.contentRect.width
        if (width > 0 && chart.width !== width) {
          chart.setSize({ width, height: 200 })
        }
      }
    })
    const parent = chart.root.parentElement
    if (parent) ro.observe(parent)

    const origDestroy = chart.destroy.bind(chart)
    chart.destroy = () => {
      ro.disconnect()
      origDestroy()
    }
  }

  function togglePause() {
    if (!paused) {
      setFrozen([...history])
    }
    setPaused(!paused)
  }

  return (
    <Card size="sm">
      <CardHeader>
        <CardTitle className="text-sm">{label}</CardTitle>
        <CardAction>
          <Button variant="ghost" size="xs" onClick={togglePause}>
            {paused ? 'Resume' : 'Pause'}
          </Button>
        </CardAction>
      </CardHeader>
      <CardContent>
        <UplotReact
          options={opts}
          data={uData}
          onCreate={onCreate}
        />
      </CardContent>
    </Card>
  )
}

function tooltipPlugin(label: string, unit: string): uPlot.Plugin {
  let tooltip: HTMLDivElement | null = null

  function init(u: uPlot) {
    tooltip = document.createElement('div')
    tooltip.style.cssText = `
      position: absolute;
      display: none;
      padding: 4px 8px;
      background: hsl(var(--popover));
      border: 1px solid hsl(var(--border));
      border-radius: var(--radius);
      font-size: 12px;
      color: hsl(var(--popover-foreground));
      pointer-events: none;
      z-index: 50;
      white-space: nowrap;
    `
    u.over.appendChild(tooltip)
  }

  function setCursor(u: uPlot) {
    if (!tooltip) return
    const idx = u.cursor.idx
    if (idx == null) {
      tooltip.style.display = 'none'
      return
    }
    const val = u.data[1][idx]
    if (val == null) {
      tooltip.style.display = 'none'
      return
    }
    tooltip.textContent = `${label}: ${val.toFixed(3)} ${unit}`
    tooltip.style.display = 'block'

    const left = u.valToPos(u.data[0][idx], 'x')
    const top = u.valToPos(val, 'y')
    tooltip.style.left = `${left + 10}px`
    tooltip.style.top = `${top - 10}px`
  }

  return {
    hooks: {
      init,
      setCursor,
    },
  }
}
