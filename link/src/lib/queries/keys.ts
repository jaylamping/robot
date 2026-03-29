/**
 * TanStack Query key factory — same pattern as nested `overviewKeys`:
 * use `linkKeys.all` for broad invalidation, then scoped segments per resource.
 */
export const linkKeys = {
  all: ['link', 'robot'] as const,

  config: () => [...linkKeys.all, 'config'] as const,

  status: () => [...linkKeys.all, 'status'] as const,

  motors: () => [...linkKeys.all, 'motors'] as const,
  motor: (id: number) => [...linkKeys.all, 'motor', id] as const,

  jointSlots: () => [...linkKeys.all, 'joint-slots'] as const,

  arms: () => [...linkKeys.all, 'arms'] as const,

  sequences: () => [...linkKeys.all, 'sequences'] as const,

  logs: (limit: number) => [...linkKeys.all, 'logs', limit] as const,

  /** Single-arm preflight; include `side` in the key like audience params in `overviewKeys.audienceCount(query)`. */
  armPreflight: (side: string) => [...linkKeys.all, 'preflight', side] as const,
  armPreflightsRoot: () => [...linkKeys.all, 'preflight'] as const,

  armHomeStatus: (side: string) => [...linkKeys.all, 'home-status', side] as const,
} as const
