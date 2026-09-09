import { Link } from 'react-router-dom'
import Section from '../Section'
import { GridSketch, LidarSketch, ObjectsSketch, WorldSketch } from './sketches'

const items = [
  {
    k: 'LiDAR input',
    d: 'Each frame is a real scan in the current ego frame. The sensor sits at the origin.',
    Visual: LidarSketch,
  },
  {
    k: 'Adaptive terrain',
    d: 'A 2.5D grid whose cell size grows with range. Heights come from ground inliers.',
    Visual: GridSketch,
  },
  {
    k: 'Object understanding',
    d: 'Geometric proposals, not SemanticKITTI labels and not a neural segmenter.',
    Visual: ObjectsSketch,
  },
  {
    k: 'World model',
    d: 'Tracks live in LiDAR frame 0. Trajectory uses dataset poses, never invented motion.',
    Visual: WorldSketch,
  },
]

export default function Glance() {
  return (
    <Section kicker="ORBIT AT A GLANCE" title="A system that processes spatial information.">
      <div className="grid gap-4 sm:grid-cols-2">
        {items.map((item) => (
          <article
            key={item.k}
            className="flex h-full flex-col overflow-hidden rounded-2xl border border-orbit-line bg-orbit-panel"
          >
            <div className="h-28 border-b border-orbit-line bg-[#0a1018] px-3 pt-2">
              <p className="font-mono text-[10px] tracking-widest text-orbit-dim">CONCEPT</p>
              <item.Visual className="h-[5.5rem] w-full" />
            </div>
            <div className="flex flex-1 flex-col p-6">
              <h3 className="text-base font-medium text-orbit-text">{item.k}</h3>
              <p className="mt-3 text-sm leading-6 text-orbit-dim">{item.d}</p>
            </div>
          </article>
        ))}
      </div>
      <p className="mt-8 text-sm leading-6 text-orbit-dim">
        Prototype, not a production autonomy stack.{' '}
        <Link to="/about" className="text-orbit-cyan hover:underline">
          Read about the system →
        </Link>
      </p>
    </Section>
  )
}
