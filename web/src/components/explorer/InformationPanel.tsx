function fmt(n: number | undefined | null, digits = 0): string {
  if (n === undefined || n === null || Number.isNaN(n)) return '—'
  return n.toLocaleString(undefined, { maximumFractionDigits: digits })
}

function Row({ k, v }: { k: string; v: string }) {
  return (
    <div className="grid grid-cols-[minmax(0,1fr)_auto] gap-4 py-2">
      <dt className="font-mono text-[10px] tracking-[0.18em] text-orbit-dim">{k}</dt>
      <dd className="text-right text-sm text-orbit-text">{v}</dd>
    </div>
  )
}

export default function InformationPanel({
  frameIndex,
  lastIndex,
  frameLabel,
  pointsFrame,
  worldFrame,
  poseSource,
  egoXy,
  headingRad,
  pointCountFull,
  pointCountExported,
  adaptiveCells,
  obstacleCells,
  liveTracks,
  confirmedTracks,
  groundMethod,
  groundInliers,
  selected,
}: {
  frameIndex: number
  lastIndex: number | undefined
  frameLabel: string
  pointsFrame?: string
  worldFrame?: string
  poseSource?: string
  egoXy?: [number, number]
  headingRad?: number
  pointCountFull?: number
  pointCountExported?: number
  adaptiveCells?: number
  obstacleCells?: number
  liveTracks?: number
  confirmedTracks?: number
  groundMethod?: string
  groundInliers?: number
  selected?: {
    title: string
    rows: { k: string; v: string }[]
  } | null
}) {
  return (
    <aside className="flex h-full min-h-0 flex-col justify-between">
      <div>
        <p className="font-mono text-[10px] tracking-[0.28em] text-orbit-cyan">FRAME INFORMATION</p>
        <dl className="mt-4 divide-y divide-orbit-line/80">
          <Row k="FRAME" v={frameLabel} />
          {pointsFrame ? <Row k="POINTS FRAME" v={pointsFrame} /> : null}
          {worldFrame ? <Row k="WORLD FRAME" v={worldFrame} /> : null}
          {pointCountFull !== undefined ? <Row k="MAPPED POINTS" v={fmt(pointCountFull)} /> : null}
          {pointCountExported !== undefined ? (
            <Row k="EXPORTED POINTS" v={fmt(pointCountExported)} />
          ) : null}
          {adaptiveCells !== undefined ? <Row k="ADAPTIVE CELLS" v={fmt(adaptiveCells)} /> : null}
          {obstacleCells !== undefined ? <Row k="OBSTACLE CELLS" v={fmt(obstacleCells)} /> : null}
          {liveTracks !== undefined ? <Row k="LIVE TRACKS" v={fmt(liveTracks)} /> : null}
          {confirmedTracks !== undefined ? (
            <Row k="CONFIRMED" v={fmt(confirmedTracks)} />
          ) : null}
          {egoXy ? (
            <Row k="EGO XY" v={`${egoXy[0].toFixed(2)}, ${egoXy[1].toFixed(2)}`} />
          ) : null}
          {headingRad !== undefined ? (
            <Row k="HEADING RAD" v={headingRad.toFixed(4)} />
          ) : null}
          {poseSource ? <Row k="POSE SOURCE" v={poseSource} /> : null}
          {groundMethod ? <Row k="GROUND" v={groundMethod} /> : null}
          {groundInliers !== undefined ? <Row k="GROUND INLIERS" v={fmt(groundInliers)} /> : null}
        </dl>
      </div>
      {selected ? (
        <div className="mt-8 border-t border-orbit-line pt-4">
          <p className="font-mono text-[10px] tracking-[0.28em] text-orbit-cyan">{selected.title}</p>
          <p className="mt-1 text-[11px] leading-4 text-orbit-dim">
            Fields below exist on the exported track or world object. Missing fields are omitted.
          </p>
          <dl className="mt-2 divide-y divide-orbit-line/80">
            {selected.rows.map((row) => (
              <Row key={row.k} k={row.k} v={row.v} />
            ))}
          </dl>
        </div>
      ) : (
        <p className="mt-8 text-xs leading-5 text-orbit-dim">
          Frame {String(frameIndex).padStart(2, '0')}
          {lastIndex !== undefined ? ` / ${String(lastIndex).padStart(2, '0')}` : ''}
          . Click a tracked object in Objects view to inspect exported fields.
        </p>
      )}
    </aside>
  )
}
