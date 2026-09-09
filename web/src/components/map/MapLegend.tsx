import type { MapVizMode } from './MapGrid'

function fmtRes(meters: number): string {
  if (meters < 1) return `${Math.round(meters * 100)} cm`
  return `${meters.toFixed(2)} m`
}

export default function MapLegend({
  mode,
  hasElevation,
  hasObstacles,
  resolutions,
  semanticClasses,
}: {
  mode: MapVizMode
  hasElevation: boolean
  hasObstacles: boolean
  resolutions?: number[]
  semanticClasses?: string[]
}) {
  const resRows = (resolutions ?? []).map((r, i, arr) => {
    const colors = ['#8cf8ff', '#59f2b8', '#739cff', '#c7a6ff']
    const rank =
      arr.length === 1
        ? 'Exported'
        : i === 0
          ? 'Finest'
          : i === arr.length - 1
            ? 'Coarsest'
            : i === 1
              ? 'Finer'
              : 'Coarser'
    return { c: colors[Math.min(i, colors.length - 1)], t: `${rank} · ${fmtRes(r)}` }
  })

  const semanticRows = [
    { id: 'GROUND', c: '#52ea9e', t: 'GROUND' },
    { id: 'MIXED', c: '#73e0ff', t: 'MIXED' },
    { id: 'OBSTACLE', c: '#ff7a38', t: 'OBSTACLE' },
  ].filter((row) => !semanticClasses?.length || semanticClasses.includes(row.id))

  const rows =
    mode === 'semantic'
      ? semanticRows.length
        ? semanticRows
        : [{ c: '#8a93a3', t: 'No semantic_class values in this export' }]
      : mode === 'resolution'
        ? resRows.length
          ? resRows
          : [{ c: '#8a93a3', t: 'No exported resolution values' }]
        : mode === 'obstacles'
          ? [
              { c: '#ff6a2c', t: 'Obstacle cells' },
              { c: '#e8c36a', t: 'Mixed cells' },
              { c: '#3d6f6a', t: 'Other exported cells' },
            ]
          : [
              { c: '#1f7af2', t: hasElevation ? 'Lower elevation' : 'Adaptive cells (flat)' },
              { c: '#33d4c4', t: hasElevation ? 'Mid elevation' : 'No elevation variation' },
              { c: '#8cf4b2', t: hasElevation ? 'Higher elevation' : 'No elevation in export' },
            ]

  return (
    <div>
      <p className="font-mono text-[10px] tracking-[0.28em] text-orbit-cyan">LEGEND</p>
      <ul className="mt-3 space-y-2">
        {rows.map((row) => (
          <li key={row.t} className="flex items-center gap-2 text-xs text-orbit-dim">
            <span className="h-2 w-2 shrink-0 rounded-sm" style={{ background: row.c }} />
            {row.t}
          </li>
        ))}
      </ul>
      <p className="mt-3 text-[11px] leading-5 text-orbit-dim">
        {mode === 'resolution'
          ? 'Smaller footprints are higher spatial detail, typically nearer the sensor. Larger footprints are coarser. Sizes come from exported resolution.'
          : 'Fine-resolution cells represent higher spatial detail near the sensor. Cell resolution increases with distance where the export supports it.'}
      </p>
      {mode === 'obstacles' && !hasObstacles ? (
        <p className="mt-2 text-[11px] leading-5 text-orbit-dim">No obstacle cells in this frame export.</p>
      ) : null}
    </div>
  )
}
