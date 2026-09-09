import type { AdaptiveCellRecord } from '../../types/orbit'

function Row({ k, v }: { k: string; v: string }) {
  return (
    <div className="flex justify-between gap-6 py-1.5">
      <dt className="font-mono text-[10px] tracking-[0.16em] text-orbit-dim">{k}</dt>
      <dd className="text-right text-xs text-orbit-text">{v}</dd>
    </div>
  )
}

export default function MapInspector({ cell }: { cell: AdaptiveCellRecord | null }) {
  if (!cell) {
    return (
      <p className="text-xs leading-5 text-orbit-dim">Click a cell to inspect exported fields.</p>
    )
  }
  const rows: { k: string; v: string }[] = []
  rows.push({ k: 'CENTER', v: `${cell.center[0].toFixed(2)}, ${cell.center[1].toFixed(2)}` })
  rows.push({ k: 'RESOLUTION', v: `${cell.resolution.toFixed(3)} m` })
  if (cell.semantic_class) rows.push({ k: 'CLASS', v: cell.semantic_class })
  if (cell.ix != null && cell.iy != null) rows.push({ k: 'INDEX', v: `${cell.ix}, ${cell.iy}` })
  if (cell.level != null) rows.push({ k: 'LEVEL', v: String(cell.level) })
  if (cell.z_min != null) rows.push({ k: 'Z MIN', v: cell.z_min.toFixed(3) })
  if (cell.z_max != null) rows.push({ k: 'Z MAX', v: cell.z_max.toFixed(3) })
  if (cell.ground_elevation != null) rows.push({ k: 'GROUND Z', v: cell.ground_elevation.toFixed(3) })
  if (cell.obstacle_elevation != null) {
    rows.push({ k: 'OBSTACLE Z', v: cell.obstacle_elevation.toFixed(3) })
  }
  if (cell.z_mean != null) rows.push({ k: 'Z MEAN', v: cell.z_mean.toFixed(3) })
  if (cell.point_count != null) rows.push({ k: 'POINTS', v: String(cell.point_count) })
  if (cell.ground_count != null) rows.push({ k: 'GROUND PTS', v: String(cell.ground_count) })
  if (cell.obstacle_count != null) rows.push({ k: 'OBSTACLE PTS', v: String(cell.obstacle_count) })

  return (
    <div>
      <p className="font-mono text-[10px] tracking-[0.28em] text-orbit-cyan">CELL</p>
      <dl className="mt-2">{rows.map((r) => <Row key={r.k} k={r.k} v={r.v} />)}</dl>
    </div>
  )
}
