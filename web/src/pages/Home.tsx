import { Link } from 'react-router-dom'
import { motion } from 'framer-motion'
import LidarField from '../components/LidarField'

const pillars = [
  { k: 'LiDAR', d: 'Current-frame observations in the sensor frame.' },
  { k: '2.5D terrain', d: 'Range-adaptive cells from real ground elevation.' },
  { k: 'Objects', d: 'Geometric proposals tracked in LiDAR frame 0.' },
  { k: 'Motion', d: 'Ego trajectory from dataset poses — never invented.' },
]

export default function Home() {
  return (
    <div className="relative min-h-svh overflow-hidden">
      <LidarField />
      <div className="pointer-events-none absolute inset-0 bg-gradient-to-r from-orbit-bg via-orbit-bg/80 to-transparent" />
      <section className="relative z-10 mx-auto flex min-h-svh max-w-6xl flex-col justify-center px-6 pb-16 pt-28">
        <motion.p
          initial={{ opacity: 0, y: 8 }}
          animate={{ opacity: 1, y: 0 }}
          className="font-mono text-xs tracking-[0.35em] text-orbit-cyan"
        >
          SPATIAL INTELLIGENCE
        </motion.p>
        <motion.h1
          initial={{ opacity: 0, y: 12 }}
          animate={{ opacity: 1, y: 0 }}
          transition={{ delay: 0.05 }}
          className="mt-4 text-5xl font-semibold tracking-tight text-orbit-text md:text-7xl"
        >
          ORBIT
        </motion.h1>
        <p className="mt-3 max-w-xl text-lg text-orbit-dim md:text-xl">
          Operational Reconstruction &amp; Intelligent Terrain
        </p>
        <p className="mt-6 max-w-xl text-sm leading-7 text-orbit-dim">
          ORBIT transforms raw LiDAR observations into an evolving spatial
          understanding of terrain, obstacles, objects, and motion. It is a
          geometric prototype — not a learned drivability or full autonomy stack.
        </p>
        <div className="pointer-events-auto mt-8 flex flex-wrap gap-3">
          <Link
            to="/demo"
            className="rounded-full bg-orbit-cyan px-5 py-2 text-sm font-medium text-orbit-bg"
          >
            Interactive demo
          </Link>
          <Link
            to="/pipeline"
            className="rounded-full border border-orbit-line px-5 py-2 text-sm text-orbit-text"
          >
            Pipeline
          </Link>
        </div>
        <div className="mt-16 grid gap-4 sm:grid-cols-2 lg:grid-cols-4">
          {pillars.map((item) => (
            <article
              key={item.k}
              className="rounded-2xl border border-orbit-line bg-orbit-panel/70 p-4 backdrop-blur-sm"
            >
              <h2 className="text-sm font-medium text-orbit-text">{item.k}</h2>
              <p className="mt-2 text-xs leading-6 text-orbit-dim">{item.d}</p>
            </article>
          ))}
        </div>
      </section>
    </div>
  )
}
