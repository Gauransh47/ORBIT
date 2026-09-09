import Section from '../Section'

const rows = [
  { k: 'Data', d: 'nuScenes v1.0-mini LIDAR_TOP keyframes, KITTI, synthetic PLY.' },
  { k: 'Perception', d: 'LiDAR-based geometric analysis: RANSAC ground and grid proposals.' },
  { k: 'Mapping', d: 'Adaptive 2.5D representation. World origin is LiDAR frame 0.' },
  { k: 'Tracking', d: 'Temporal object identities in that same world frame.' },
  { k: 'Visualization', d: 'This site: React, Vite, Tailwind, Canvas, Framer Motion. Three.js explorers after export.' },
]

export default function TechFoundation() {
  return (
    <Section kicker="TECHNOLOGY" title="System foundation.">
      <ul className="grid gap-4 md:grid-cols-2">
        {rows.map((row, i) => (
          <li
            key={row.k}
            className={`min-h-[7.5rem] rounded-2xl border border-orbit-line bg-orbit-panel p-5 ${
              i === rows.length - 1 ? 'md:col-span-2' : ''
            }`}
          >
            <p className="font-mono text-[11px] tracking-[0.2em] text-orbit-cyan">{row.k}</p>
            <p className="mt-3 text-sm leading-6 text-orbit-dim">{row.d}</p>
          </li>
        ))}
      </ul>
    </Section>
  )
}
