import { Link } from 'react-router-dom'
import Section from '../Section'
import { CloudField, GridSketch, TrackSketch, WorldSketch } from './sketches'

const panels = [
  { t: 'Point cloud', s: 'Live ego scan — conceptual', Visual: CloudField },
  { t: 'World trajectory', s: 'LiDAR frame 0 — conceptual', Visual: WorldSketch },
  { t: 'Terrain grid', s: 'Adaptive cells — conceptual', Visual: GridSketch },
  { t: 'Tracking markers', s: 'Identities over time — conceptual', Visual: TrackSketch },
]

export default function DemoPreview() {
  return (
    <Section kicker="INTERACTIVE ORBIT DEMO" title="Frame-by-frame exploration of the system.">
      <p className="max-w-2xl text-sm leading-7 text-orbit-dim">
        The Interactive Demo is the ORBIT Explorer: it fetches exported
        scene JSON at runtime. The panels below are still conceptual sketches,
        not the live Three.js views.
      </p>
      <div className="mt-8 grid gap-4 md:grid-cols-2">
        {panels.map((p) => (
          <figure
            key={p.t}
            className="flex min-h-0 flex-col overflow-hidden rounded-2xl border border-orbit-line bg-orbit-panel"
          >
            <figcaption className="border-b border-orbit-line px-4 py-3">
              <p className="text-sm text-orbit-text">{p.t}</p>
              <p className="mt-1 text-xs leading-5 text-orbit-dim">{p.s}</p>
            </figcaption>
            <div className="bg-[#0a1018] px-3 py-2">
              <p.Visual className="h-36 w-full" />
            </div>
          </figure>
        ))}
      </div>
      <Link
        to="/demo"
        className="mt-8 inline-flex rounded-full bg-orbit-cyan px-6 py-2.5 text-sm font-medium text-orbit-bg hover:opacity-90"
      >
        Open interactive demo →
      </Link>
    </Section>
  )
}
