import { motion } from 'framer-motion'
import PageShell from '../components/PageShell'

const flow = [
  { now: true, label: 'Terrain analysis' },
  { now: true, label: 'Obstacle geometry' },
  { now: true, label: 'Geometric cell class (GROUND / MIXED / OBSTACLE)' },
  { now: false, label: 'Traversability cost (planned)' },
  { now: false, label: 'Planned route (planned)' },
]

export default function Planning() {
  return (
    <PageShell kicker="PATH PLANNING" title="Planned extension, not current runtime">
      <p>
        The Python pipeline does not yet compute an autonomous driving path.
        This page is a placeholder for that interface.
      </p>
      <div className="grid gap-3">
        {flow.map((step) => (
          <motion.div
            key={step.label}
            initial={{ opacity: 0 }}
            animate={{ opacity: 1 }}
            className="rounded-xl border border-orbit-line bg-orbit-panel px-4 py-3"
          >
            <p className="font-mono text-[10px] tracking-widest text-orbit-cyan">
              {step.now ? 'CURRENT PROTOTYPE' : 'PLANNED EXTENSION'}
            </p>
            <p className="mt-1 text-sm text-orbit-text">{step.label}</p>
          </motion.div>
        ))}
      </div>
    </PageShell>
  )
}
