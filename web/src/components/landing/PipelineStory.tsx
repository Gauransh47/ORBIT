import { useEffect, useRef, useState } from 'react'
import { motion } from 'framer-motion'
import {
  GridSketch,
  LidarSketch,
  ObjectsSketch,
  RouteSketch,
  TerrainSketch,
  TrackSketch,
  WorldSketch,
} from './sketches'

const stages = [
  {
    name: 'Raw LiDAR',
    copy: 'A scan arrives in the current sensor frame. Origin at the LiDAR, +X forward.',
    Visual: LidarSketch,
  },
  {
    name: 'Ground / terrain analysis',
    copy: 'RANSAC separates ground inliers from the remainder. No learned terrain model.',
    Visual: TerrainSketch,
  },
  {
    name: 'Adaptive 2.5D grid',
    copy: 'Cell size grows with range: fine nearby, coarser at distance.',
    Visual: GridSketch,
  },
  {
    name: 'Object perception',
    copy: 'Obstacle cells form geometric proposals — VEHICLE-LIKE, WALL, POLE, OBSTACLE.',
    Visual: ObjectsSketch,
  },
  {
    name: 'Tracking',
    copy: 'Identities persist across frames in LiDAR frame 0.',
    Visual: TrackSketch,
  },
  {
    name: 'World model',
    copy: 'Confirmed objects live in one reference: the first processed LiDAR frame.',
    Visual: WorldSketch,
  },
  {
    name: 'Navigation',
    copy: 'A planned extension. The current runtime does not compute a driving path.',
    Visual: RouteSketch,
  },
]

export default function PipelineStory() {
  const refs = useRef<(HTMLLIElement | null)[]>([])
  const [active, setActive] = useState(0)
  const ActiveVisual = stages[active].Visual

  useEffect(() => {
    const nodes = refs.current.filter(Boolean) as HTMLLIElement[]
    const io = new IntersectionObserver(
      (entries) => {
        const visible = entries
          .filter((e) => e.isIntersecting)
          .sort((a, b) => b.intersectionRatio - a.intersectionRatio)[0]
        if (!visible) return
        const idx = nodes.indexOf(visible.target as HTMLLIElement)
        if (idx >= 0) setActive(idx)
      },
      { rootMargin: '-35% 0px -35% 0px', threshold: [0.2, 0.6] },
    )
    nodes.forEach((n) => io.observe(n))
    return () => io.disconnect()
  }, [])

  return (
    <div className="grid items-start gap-10 lg:grid-cols-[minmax(0,1fr)_minmax(16rem,1.05fr)]">
      <ol className="relative space-y-8 border-l border-orbit-line pl-6">
        <span
          className="absolute top-0 -left-px w-px origin-top bg-orbit-cyan/80 transition-transform duration-500 ease-out"
          style={{
            height: '100%',
            transform: `scaleY(${(active + 1) / stages.length})`,
          }}
        />
        {stages.map((stage, i) => (
          <li
            key={stage.name}
            ref={(el) => {
              refs.current[i] = el
            }}
            className="scroll-mt-28"
          >
            <p className="font-mono text-[10px] tracking-widest text-orbit-cyan">
              {String(i + 1).padStart(2, '0')}
              {i === 6 ? '  ·  PLANNED' : ''}
            </p>
            <h3
              className={`mt-1 text-lg leading-snug transition-colors ${
                active === i ? 'text-orbit-text' : 'text-orbit-dim'
              }`}
            >
              {stage.name}
            </h3>
            <p className="mt-2 max-w-md text-sm leading-6 text-orbit-dim">{stage.copy}</p>
          </li>
        ))}
      </ol>
      <div className="lg:sticky lg:top-28 lg:self-start">
        <motion.div
          key={active}
          initial={{ opacity: 0.35, y: 10 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ duration: 0.28 }}
          className="space-y-3"
        >
          <div className="overflow-hidden rounded-2xl border border-orbit-cyan/35 bg-[#0a1018] p-4">
            <p className="font-mono text-[10px] tracking-widest text-orbit-dim">
              CONCEPTUAL STAGE · NOT EXPORTED DATA
            </p>
            <ActiveVisual className="mt-2 h-40 w-full" />
            <p className="mt-2 text-sm text-orbit-text">{stages[active].name}</p>
          </div>
        </motion.div>
      </div>
    </div>
  )
}
