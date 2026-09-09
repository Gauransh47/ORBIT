import type { AdaptiveCellRecord, FrameJson, Manifest, Metrics } from '../types/orbit'
import { isFiniteNumber, isRenderableCell } from './cellVisual'

export type MetricRow = { k: string; v: string }

function num(metrics: Metrics, key: string): number | null {
  const value = metrics[key]
  return isFiniteNumber(value) ? value : null
}

/** Honest counts from exported JSON only. No FPS, latency, or accuracy. */
export function prototypeMetrics(
  frame: FrameJson | null,
  manifest: Manifest | null,
): MetricRow[] {
  if (!frame) return []
  const rows: MetricRow[] = []
  const m = frame.metrics ?? {}
  const cells = (frame.adaptive_cells ?? []).filter(isRenderableCell)

  const input = num(m, 'input_points') ?? num(m, 'point_count_full') ?? num(m, 'mapped_points')
  if (input != null) rows.push({ k: 'INPUT POINTS', v: input.toLocaleString() })

  const exportedPts = num(m, 'point_count_exported')
  if (exportedPts != null) rows.push({ k: 'EXPORTED POINTS', v: exportedPts.toLocaleString() })

  const cellCount = num(m, 'adaptive_cells') ?? num(m, 'adaptive_cells_exported') ?? cells.length
  rows.push({ k: 'ADAPTIVE CELLS', v: cellCount.toLocaleString() })

  if (input != null && cellCount > 0) {
    rows.push({ k: 'POINT-TO-CELL RATIO', v: `${(input / cellCount).toFixed(1)}×` })
  }

  const resSet = new Map<number, number>()
  for (const cell of cells) {
    const r = Number(cell.resolution.toFixed(4))
    resSet.set(r, (resSet.get(r) ?? 0) + 1)
  }
  if (resSet.size) {
    rows.push({ k: 'RESOLUTION LEVELS', v: String(resSet.size) })
    const sorted = [...resSet.entries()].sort((a, b) => a[0] - b[0])
    const finest = sorted[0]
    const coarsest = sorted[sorted.length - 1]
    rows.push({
      k: 'FINE / COARSE',
      v: `${finest[1].toLocaleString()} @ ${cm(finest[0])} · ${coarsest[1].toLocaleString()} @ ${cm(coarsest[0])}`,
    })
  }

  const tracks = num(m, 'live_tracks') ?? frame.tracks?.length
  if (tracks != null) rows.push({ k: 'LIVE TRACKS', v: String(tracks) })

  rows.push({ k: 'FRAME INDEX', v: String(frame.frame_index) })
  if (manifest?.frame_count != null) {
    rows.push({ k: 'EXPORTED FRAMES', v: String(manifest.frame_count) })
  }
  return rows
}

function cm(meters: number): string {
  if (meters < 1) return `${Math.round(meters * 100)} cm`
  return `${meters.toFixed(2)} m`
}

export function uniqueResolutions(cells: AdaptiveCellRecord[]): number[] {
  const set = new Set<number>()
  for (const cell of cells) {
    if (isRenderableCell(cell)) set.add(Number(cell.resolution.toFixed(4)))
  }
  return [...set].sort((a, b) => a - b)
}

/** Per-resolution range from ego using exported cell centres — not assumed ring radii. */
export function foveationFromExport(
  cells: AdaptiveCellRecord[],
  egoXy: [number, number] | null,
): { resolution: number; count: number; minRange: number; maxRange: number }[] {
  const ego = egoXy ?? [0, 0]
  const buckets = new Map<number, { count: number; minRange: number; maxRange: number }>()
  for (const cell of cells) {
    if (!isRenderableCell(cell)) continue
    const r = Number(cell.resolution.toFixed(4))
    const range = Math.hypot(cell.center[0] - ego[0], cell.center[1] - ego[1])
    const cur = buckets.get(r)
    if (!cur) buckets.set(r, { count: 1, minRange: range, maxRange: range })
    else {
      cur.count += 1
      cur.minRange = Math.min(cur.minRange, range)
      cur.maxRange = Math.max(cur.maxRange, range)
    }
  }
  return [...buckets.entries()]
    .sort((a, b) => a[0] - b[0])
    .map(([resolution, v]) => ({ resolution, ...v }))
}
