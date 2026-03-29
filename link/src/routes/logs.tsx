import { createFileRoute } from '@tanstack/react-router'
import { useEffect, useState, useRef } from 'react'
import type { LogEntry } from '@/lib/api'
import { useRobotLogs } from '@/lib/queries'
import { Card, CardContent, CardHeader, CardTitle, CardAction } from '@/components/ui/card'
import { Button } from '@/components/ui/button'
import { Badge } from '@/components/ui/badge'
import { ScrollArea } from '@/components/ui/scroll-area'
import { Switch } from '@/components/ui/switch'
import { LuRefreshCw } from 'react-icons/lu'

const LOG_LIMIT = 500

export const Route = createFileRoute('/logs')({
  component: LogsPage,
})

function LogsPage() {
  const [autoRefresh, setAutoRefresh] = useState(true)
  const bottomRef = useRef<HTMLDivElement>(null)

  const logsQ = useRobotLogs(LOG_LIMIT, {
    refetchInterval: autoRefresh ? 2000 : false,
  })

  const entries = logsQ.data ?? []
  const error = logsQ.error?.message ?? null

  useEffect(() => {
    bottomRef.current?.scrollIntoView({ behavior: 'smooth' })
  }, [entries.length])

  return (
    <div>
      <h2 className="text-xl font-semibold mb-6">Logs</h2>

      <Card>
        <CardHeader>
          <CardTitle>Server Logs</CardTitle>
          <CardAction>
            <div className="flex items-center gap-3">
              <div className="flex items-center gap-2 text-xs text-muted-foreground">
                <Switch
                  checked={autoRefresh}
                  onCheckedChange={setAutoRefresh}
                />
                Auto-refresh
              </div>
              <Button
                variant="ghost"
                size="icon-xs"
                onClick={() => void logsQ.refetch()}
                disabled={logsQ.isFetching}
              >
                <LuRefreshCw className={`size-3 ${logsQ.isFetching ? 'animate-spin' : ''}`} />
              </Button>
            </div>
          </CardAction>
        </CardHeader>
        <CardContent>
          {error && (
            <p className="text-destructive text-sm mb-2">{error}</p>
          )}

          {entries.length === 0 ? (
            <p className="text-muted-foreground text-sm">No log entries yet.</p>
          ) : (
            <ScrollArea className="h-[60vh] rounded-md border bg-muted/30 p-1">
              <div className="font-mono text-xs space-y-px">
                {entries.map((entry, i) => (
                  <LogLine key={i} entry={entry} />
                ))}
                <div ref={bottomRef} />
              </div>
            </ScrollArea>
          )}
        </CardContent>
      </Card>
    </div>
  )
}

function LogLine({ entry }: { entry: LogEntry }) {
  const levelColor =
    entry.level === 'ERROR' || entry.level === 'error'
      ? 'text-red-400'
      : entry.level === 'WARN' || entry.level === 'warn'
        ? 'text-amber-400'
        : 'text-muted-foreground'

  const ts = new Date(entry.timestamp_ms)
  const timeStr = ts.toLocaleTimeString(undefined, {
    hour12: false,
    hour: '2-digit',
    minute: '2-digit',
    second: '2-digit',
    fractionalSecondDigits: 3,
  })

  return (
    <div className="flex gap-2 py-0.5 px-1 hover:bg-muted/50 rounded-sm">
      <span className="text-muted-foreground shrink-0">{timeStr}</span>
      <Badge variant="outline" className={`text-[10px] h-4 px-1 shrink-0 ${levelColor}`}>
        {entry.level}
      </Badge>
      <span className="text-muted-foreground shrink-0 truncate max-w-[140px]" title={entry.target}>
        {entry.target}
      </span>
      <span className="break-all">{entry.message}</span>
    </div>
  )
}
