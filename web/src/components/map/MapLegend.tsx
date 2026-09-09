import type { MapVizMode } from './MapGrid'

export default function MapLegend({
  mode,
  hasElevation,
  hasObstacles,
}: {
  mode: MapVizMode
  hasElevation: boolean
  hasObstacles: boolean
}) {
  const rows =
    mode === 'semantic'
      ? [
          { c: '#2fbf9a', t: 'GROUND' },
          { c: '#3ddcff', t: 'MIXED' },
          { c: '#e08a3c', t: 'OBSTACLE' },
        ]
      : mode === 'resolution'
        ? [
            { c: '#73f2ff', t: 'Fine (~5 cm)' },
            { c: '#3ddcc7', t: 'Medium (~10 cm)' },
            { c: '#3d8cf2', t: 'Coarse (~25 cm)' },
            { c: '#596b94', t: 'Coarsest (~50 cm)' },
          ]
        : mode === 'obstacles'
          ? [
              { c: '#e08a3c', t: 'Obstacle cells' },
              { c: '#c4a574', t: 'Mixed cells' },
              { c: '#1a3030', t: 'Other cells' },
            ]
          : [
              { c: '#2fbf9a', t: hasElevation ? 'Lower elevation' : 'Adaptive cells (flat)' },
              { c: '#7ee0b8', t: hasElevation ? 'Higher elevation' : 'No elevation in export' },
            ]

  return (
    <div>
      <p className="font-mono text-[10px] tracking-[0.28em] text-orbit-cyan">LEGEND</p>
      <ul className="mt-3 space-y-2">
        {rows.map((row) => (
          <li key={row.t} className="flex items-center gap-2 text-xs text-orbit-dim">
            <span className="h-2 w-2 rounded-sm" style={{ background: row.c }} />
            {row.t}
          </li>
        ))}
      </ul>
      <p className="mt-3 text-[11px] leading-5 text-orbit-dim">
        Cell footprint is the exported resolution. Fine cells are smaller; coarse cells are larger.
      </p>
      {mode === 'obstacles' && !hasObstacles ? (
        <p className="mt-2 text-[11px] leading-5 text-orbit-dim">No obstacle cells in this frame export.</p>
      ) : null}
    </div>
  )
}
