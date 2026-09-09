import { motion } from 'framer-motion'
import PageShell from '../components/PageShell'

const stages = [
  'Raw LiDAR',
  'Ground / terrain analysis',
  'Adaptive 2.5D grid',
  'Obstacle detection',
  'Object proposals',
  'Tracking',
  'World model',
  'Website A* planning demonstration (not Python runtime)',
]

export default function Pipeline() {
  return (
    <PageShell kicker="PIPELINE" title="How a frame becomes a world model">
      <p>
        Each processed frame follows the same geometric path. The website can
        demonstrate A* on exported occupancy; that is not a Python ORBIT runtime
        planner.
      </p>
      <ol className="mt-4 space-y-3">
        {stages.map((name, i) => (
          <motion.li
            key={name}
            initial={{ opacity: 0, x: -8 }}
            animate={{ opacity: 1, x: 0 }}
            transition={{ delay: i * 0.04 }}
            className="flex items-center gap-4 rounded-xl border border-orbit-line bg-orbit-panel px-4 py-3"
          >
            <span className="font-mono text-xs text-orbit-cyan">
              {String(i + 1).padStart(2, '0')}
            </span>
            <span className="text-sm text-orbit-text">{name}</span>
          </motion.li>
        ))}
      </ol>
    </PageShell>
  )
}
