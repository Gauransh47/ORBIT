import { Link } from 'react-router-dom'
import PageShell from '../components/PageShell'

export default function Demo() {
  return (
    <PageShell kicker="INTERACTIVE DEMO" title="Frame explorer (next phase)">
      <p>
        This page is the home for PREV / NEXT frame inspection of real exported
        PipelineState: live LiDAR, LiDAR-frame-0 world map, adaptive grid,
        tracks, and metrics.
      </p>
      <p>
        Phase 1 does not render fake metrics. Export + playback land in a later
        phase, after <code className="text-orbit-cyan">web_export</code> writes
        JSON under <code className="text-orbit-cyan">exported_data/</code>.
      </p>
      <Link to="/" className="inline-block text-orbit-cyan">
        ← Back to home
      </Link>
    </PageShell>
  )
}
