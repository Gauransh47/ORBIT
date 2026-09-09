import PageShell from '../components/PageShell'

export default function MapExplorer() {
  return (
    <PageShell kicker="2.5D MAP" title="Adaptive grid explorer (next phase)">
      <p>
        A dedicated Three.js view of AdaptiveCell centres, resolutions, and
        geometric class will live here: rotate, zoom, pan, reset, top /
        isometric / side cameras.
      </p>
      <p>
        Data will come from exported cells only. Fine rings (5 cm) and coarse
        rings (50 cm) stay distinguishable when the JSON includes{' '}
        <code className="text-orbit-cyan">resolution</code>.
      </p>
    </PageShell>
  )
}
