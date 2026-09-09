import { Link } from 'react-router-dom'
import { motion } from 'framer-motion'
import LidarField from '../components/LidarField'
import Glance from '../components/landing/Glance'
import PipelineStory from '../components/landing/PipelineStory'
import DemoPreview from '../components/landing/DemoPreview'
import MapPreview from '../components/landing/MapPreview'
import PlanningPreview from '../components/landing/PlanningPreview'
import TechFoundation from '../components/landing/TechFoundation'
import Section from '../components/Section'

export default function Home() {
  return (
    <div className="bg-orbit-bg">
      <div className="relative min-h-svh overflow-hidden">
        <LidarField />
        <div className="pointer-events-none absolute inset-0 bg-gradient-to-r from-orbit-bg via-orbit-bg/88 to-orbit-bg/25" />
        <div className="pointer-events-none absolute inset-y-0 left-0 w-[min(100%,42rem)] bg-gradient-to-r from-orbit-bg to-transparent" />
        <div className="pointer-events-none absolute inset-x-0 bottom-0 h-40 bg-gradient-to-t from-orbit-bg to-transparent" />

        <section className="relative z-10 mx-auto flex min-h-svh max-w-6xl flex-col justify-center px-6 pb-24 pt-28">
          <motion.p
            initial={{ opacity: 0, y: 8 }}
            animate={{ opacity: 1, y: 0 }}
            className="font-mono text-[11px] tracking-[0.35em] text-orbit-cyan"
          >
            SPATIAL INTELLIGENCE
          </motion.p>
          <motion.h1
            initial={{ opacity: 0, y: 14 }}
            animate={{ opacity: 1, y: 0 }}
            transition={{ delay: 0.04 }}
            className="mt-4 text-balance text-[clamp(2.75rem,8vw,4.75rem)] font-semibold leading-none tracking-tight text-orbit-text"
          >
            ORBIT
          </motion.h1>
          <p className="mt-5 max-w-xl text-pretty text-[clamp(1.15rem,2.4vw,1.65rem)] leading-snug text-orbit-text/90">
            Operational Reconstruction
            <br />
            &amp; Intelligent Terrain
          </p>
          <p className="mt-6 max-w-lg text-sm leading-7 text-orbit-dim md:text-[0.95rem]">
            Adaptive variable-resolution 2.5D LiDAR mapping for dynamic environment
            perception — a prototype toward the intended DRDO/iDEX system.
          </p>
          <p className="mt-3 max-w-lg font-mono text-[11px] leading-5 tracking-wide text-orbit-dim/80">
            Geometric prototype · LiDAR frame 0 world · PointNet++ / Sparse CNN are future work
          </p>
          <div className="mt-10 flex flex-wrap items-center gap-3">
            <Link
              to="/demo"
              className="rounded-full bg-orbit-cyan px-6 py-2.5 text-sm font-medium text-orbit-bg hover:opacity-90"
            >
              Enter ORBIT →
            </Link>
            <Link
              to="/pipeline"
              className="rounded-full border border-orbit-line px-6 py-2.5 text-sm text-orbit-text hover:border-orbit-cyan/40"
            >
              Explore the system
            </Link>
          </div>
          <dl className="mt-14 grid max-w-lg grid-cols-2 gap-x-8 gap-y-4 font-mono text-[10px] tracking-widest text-orbit-dim sm:grid-cols-3">
            <div>
              <dt>ORIGIN</dt>
              <dd className="mt-1 text-orbit-cyan">LiDAR / 0</dd>
            </div>
            <div>
              <dt>SIGNAL</dt>
              <dd className="mt-1 text-orbit-cyan">XYZ scan</dd>
            </div>
            <div>
              <dt>MAP</dt>
              <dd className="mt-1 text-orbit-cyan">Adaptive 2.5D</dd>
            </div>
          </dl>
        </section>
      </div>

      <Glance />

      <Section kicker="PIPELINE" title="Follow a frame through the system.">
        <p className="mb-10 max-w-2xl text-sm leading-7 text-orbit-dim">
          Scroll the stages. The panel on the right is an educational sketch of
          the current step — not an exported nuScenes frame.
        </p>
        <PipelineStory />
      </Section>

      <DemoPreview />
      <MapPreview />
      <PlanningPreview />
      <TechFoundation />

      <footer className="border-t border-orbit-line px-6 py-10 text-center text-xs leading-6 text-orbit-dim">
        ORBIT prototype · LiDAR frame 0 world · website separate from the Python dashboard
      </footer>
    </div>
  )
}
