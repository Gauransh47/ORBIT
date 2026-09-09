import type { AdaptiveCellRecord } from '../types/orbit'

export function cellElevation(cell: AdaptiveCellRecord): number | null {
  if (cell.ground_elevation != null && Number.isFinite(cell.ground_elevation)) {
    return cell.ground_elevation
  }
  if (cell.z_mean != null && Number.isFinite(cell.z_mean)) {
    return cell.z_mean
  }
  return null
}

export function frameHasElevation(cells: AdaptiveCellRecord[]): boolean {
  return cells.some((c) => cellElevation(c) != null)
}

export function elevationRange(cells: AdaptiveCellRecord[]): { min: number; max: number } | null {
  const vals = cells.map(cellElevation).filter((v): v is number => v != null)
  if (!vals.length) return null
  return { min: Math.min(...vals), max: Math.max(...vals) }
}

export function resolutionColor(resolution: number): [number, number, number] {
  if (resolution <= 0.06) return [0.45, 0.95, 1]
  if (resolution <= 0.12) return [0.24, 0.86, 0.78]
  if (resolution <= 0.3) return [0.24, 0.55, 0.95]
  return [0.35, 0.42, 0.58]
}

export function semanticColor(name: string | undefined): [number, number, number] {
  if (name === 'GROUND') return [0.18, 0.75, 0.6]
  if (name === 'MIXED') return [0.24, 0.86, 1]
  if (name === 'OBSTACLE') return [0.88, 0.54, 0.24]
  return [0.36, 0.42, 0.49]
}

export function terrainColor(elev: number | null, range: { min: number; max: number } | null): [number, number, number] {
  if (elev == null || !range || range.max === range.min) {
    return [0.22, 0.55, 0.5]
  }
  const t = (elev - range.min) / (range.max - range.min)
  return [0.12 + t * 0.2, 0.45 + t * 0.45, 0.42 + t * 0.2]
}
