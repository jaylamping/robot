import { createFileRoute } from '@tanstack/react-router'
import { useEffect, useMemo, useState, useRef } from 'react'
import type { LogEntry } from '@/lib/api'
import { useRobotLogs } from '@/lib/queries'
import { Card, CardContent, CardHeader, CardTitle, CardAction } from '@/components/ui/card'
import { Button } from '@/components/ui/button'
import { Badge } from '@/components/ui/badge'
import { ScrollArea } from '@/components/ui/scroll-area'
import { Switch } from '@/components/ui/switch'
import { LuRefreshCw } from 'react-icons/lu'

const LOG_LIMIT = 500

const LOG_LEVELS = ['TRACE', 'DEBUG', 'INFO', 'WARN', 'ERROR'] as const
type LogLevel = (typeof LOG_LEVELS)[number]

const LEVEL_RANK: Record<string, number> = {
  TRACE: 0, trace: 0,
  DEBUG: 1, debug: 1,
  INFO: 2, info: 2,
  WARN: 3, warn: 3,
  ERROR: 4, error: 4,
}

export const Route = createFileRoute('/logs')({
  component: LogsPage,
})

function LogsPage() {
  const [autoRefresh, setAutoRefresh] = useState(true)
  const [minLevel, setMinLevel] = useState<LogLevel>('INFO')
  const bottomRef = useRef<HTMLDivElement>(null)

  const logsQ = useRobotLogs(LOG_LIMIT, {
    refetchInterval: autoRefresh ? 2000 : false,
  })

  const allEntries = logsQ.data ?? []
  const error = logsQ.error?.message ?? null

  const entries = useMemo(() => {
    const threshold = LEVEL_RANK[minLevel] ?? 0
    return allEntries.filter((e) => (LEVEL_RANK[e.level] ?? 0) >= threshold)
  }, [allEntries, minLevel])

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
              <div className="flex items-center gap-1">
                {LOG_LEVELS.map((level) => (
                  <Button
                    key={level}
                    variant={minLevel === level ? 'default' : 'ghost'}
                    size="xs"
                    className="text-[10px] h-5 px-1.5"
                    onClick={() => setMinLevel(level)}
                  >
                    {level}
                  </Button>
                ))}
              </div>
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
            <p className="text-muted-foreground text-sm">
              {allEntries.length === 0
                ? 'No log entries yet.'
                : `No ${minLevel}+ entries. Try a lower level.`}
            </p>
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

          <p className="text-muted-foreground text-[10px] mt-2">
            Showing {entries.length} of {allEntries.length} entries
          </p>
        </CardContent>
      </Card>
    </div>
  )
}

const LEVEL_COLORS: Record<string, string> = {
  ERROR: 'text-red-400',
  error: 'text-red-400',
  WARN: 'text-amber-400',
  warn: 'text-amber-400',
  INFO: 'text-sky-400',
  info: 'text-sky-400',
  DEBUG: 'text-muted-foreground',
  debug: 'text-muted-foreground',
  TRACE: 'text-muted-foreground/60',
  trace: 'text-muted-foreground/60',
}

function LogLine({ entry }: { entry: LogEntry }) {
  const levelColor = LEVEL_COLORS[entry.level] ?? 'text-muted-foreground'

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
