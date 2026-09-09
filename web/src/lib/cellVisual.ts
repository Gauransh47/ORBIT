import type { AdaptiveCellRecord } from '../types/orbit'
import { orbitToThree } from './orbitCoords'

/** Max InstancedMesh cells drawn. Full export remains in memory for counts/planning. */
export const MAX_RENDER_CELLS = 28000

export function evenPickIndices(count: number, max: number): number[] {
  const n = Math.max(0, Math.floor(count))
  const cap = Math.max(1, Math.floor(max))
  if (n <= cap) {
    return Array.from({ length: n }, (_, i) => i)
  }
  const out = new Array<number>(cap)
  for (let i = 0; i < cap; i++) {
    out[i] = Math.round((i * (n - 1)) / (cap - 1))
  }
  return out
}

export function isFiniteNumber(value: unknown): value is number {
  return typeof value === 'number' && Number.isFinite(value)
}

export function isRenderableCell(cell: AdaptiveCellRecord | null | undefined): cell is AdaptiveCellRecord {
  if (!cell || !Array.isArray(cell.center) || cell.center.length < 2) return false
  if (!isFiniteNumber(cell.center[0]) || !isFiniteNumber(cell.center[1])) return false
  if (!isFiniteNumber(cell.resolution) || cell.resolution <= 0) return false
  return true
}

export function cellElevation(cell: AdaptiveCellRecord): number | null {
  if (isFiniteNumber(cell.ground_elevation)) return cell.ground_elevation
  if (isFiniteNumber(cell.z_mean)) return cell.z_mean
  if (isFiniteNumber(cell.z_min) && isFiniteNumber(cell.z_max)) return (cell.z_min + cell.z_max) * 0.5
  if (isFiniteNumber(cell.z_min)) return cell.z_min
  if (isFiniteNumber(cell.z_max)) return cell.z_max
  if (isFiniteNumber(cell.obstacle_elevation)) return cell.obstacle_elevation
  return null
}

export function frameHasElevation(cells: AdaptiveCellRecord[]): boolean {
  return cells.some((c) => cellElevation(c) != null)
}

export function elevationRange(cells: AdaptiveCellRecord[]): { min: number; max: number } | null {
  let min = Infinity
  let max = -Infinity
  for (const cell of cells) {
    const v = cellElevation(cell)
    if (v == null) continue
    if (v < min) min = v
    if (v > max) max = v
  }
  if (!Number.isFinite(min) || !Number.isFinite(max)) return null
  return { min, max }
}

export function gridFocus(cells: AdaptiveCellRecord[]): {
  target: [number, number, number]
  radius: number
} {
  let minX = Infinity
  let maxX = -Infinity
  let minY = Infinity
  let maxY = -Infinity
  let minZ = Infinity
  let maxZ = -Infinity
  let any = false
  for (const cell of cells) {
    if (!isRenderableCell(cell)) continue
    const half = Math.max(cell.resolution, 0.03) * 0.5
    const elev = cellElevation(cell) ?? 0
    if (!isFiniteNumber(elev)) continue
    const [x, y, z] = orbitToThree(cell.center[0], cell.center[1], elev)
    if (![x, y, z].every(isFiniteNumber)) continue
    any = true
    minX = Math.min(minX, x - half)
    maxX = Math.max(maxX, x + half)
    minY = Math.min(minY, y)
    maxY = Math.max(maxY, y + half)
    minZ = Math.min(minZ, z - half)
    maxZ = Math.max(maxZ, z + half)
  }
  if (!any) return { target: [0, 0, 0], radius: 40 }
  const span = Math.max(maxX - minX, maxY - minY, maxZ - minZ, 0.4)
  const target: [number, number, number] = [(minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2]
  if (!target.every(isFiniteNumber)) return { target: [0, 0, 0], radius: 40 }
  return { target, radius: span * 0.85 }
}

export function pointsFocus(points: number[][]): {
  target: [number, number, number]
  radius: number
} {
  let minX = Infinity
  let maxX = -Infinity
  let minY = Infinity
  let maxY = -Infinity
  let minZ = Infinity
  let maxZ = -Infinity
  let any = false
  for (const p of points) {
    if (!p || p.length < 3) continue
    if (![p[0], p[1], p[2]].every(isFiniteNumber)) continue
    const [x, y, z] = orbitToThree(p[0], p[1], p[2])
    if (![x, y, z].every(isFiniteNumber)) continue
    any = true
    minX = Math.min(minX, x)
    maxX = Math.max(maxX, x)
    minY = Math.min(minY, y)
    maxY = Math.max(maxY, y)
    minZ = Math.min(minZ, z)
    maxZ = Math.max(maxZ, z)
  }
  if (!any) return { target: [0, 0, 0], radius: 40 }
  const span = Math.max(maxX - minX, maxY - minY, maxZ - minZ, 0.4)
  const target: [number, number, number] = [(minX + maxX) / 2, (minY + maxY) / 2, (minZ + maxZ) / 2]
  if (!target.every(isFiniteNumber)) return { target: [0, 0, 0], radius: 40 }
  return { target, radius: span * 0.85 }
}

export function resolutionColor(resolution: number): [number, number, number] {
  if (resolution <= 0.06) return [0.45, 0.96, 1]
  if (resolution <= 0.12) return [0.28, 0.9, 0.78]
  if (resolution <= 0.3) return [0.38, 0.62, 0.95]
  return [0.58, 0.7, 0.86]
}

export function semanticColor(name: string | undefined): [number, number, number] {
  if (name === 'GROUND') return [0.32, 0.92, 0.62]
  if (name === 'MIXED') return [0.45, 0.88, 1]
  if (name === 'OBSTACLE') return [1, 0.48, 0.22]
  return [0.42, 0.78, 0.82]
}

export function terrainColor(
  elev: number | null,
  range: { min: number; max: number } | null,
): [number, number, number] {
  if (elev == null || !range || range.max === range.min) {
    return [0.32, 0.82, 0.8]
  }
  const t = Math.min(1, Math.max(0, (elev - range.min) / (range.max - range.min)))
  const low: [number, number, number] = [0.16, 0.42, 0.95]
  const mid: [number, number, number] = [0.2, 0.86, 0.78]
  const high: [number, number, number] = [0.55, 0.96, 0.7]
  if (t < 0.5) {
    const u = t * 2
    return [
      low[0] + (mid[0] - low[0]) * u,
      low[1] + (mid[1] - low[1]) * u,
      low[2] + (mid[2] - low[2]) * u,
    ]
  }
  const u = (t - 0.5) * 2
  return [
    mid[0] + (high[0] - mid[0]) * u,
    mid[1] + (high[1] - mid[1]) * u,
    mid[2] + (high[2] - mid[2]) * u,
  ]
}
