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
        Phase 2 does not invent metrics. Real JSON playback is Phase 4, after
        the Phase 3 <code className="text-orbit-cyan">web_export</code> writer
        dumps PipelineState under <code className="text-orbit-cyan">exported_data/</code>.
      </p>
      <Link to="/" className="inline-block text-orbit-cyan">
        ← Back to home
      </Link>
    </PageShell>
  )
}
