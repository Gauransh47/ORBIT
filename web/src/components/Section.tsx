import type { ReactNode } from 'react'
import { motion } from 'framer-motion'

export default function Section({
  id,
  kicker,
  title,
  children,
}: {
  id?: string
  kicker: string
  title: string
  children: ReactNode
}) {
  return (
    <section id={id} className="mx-auto w-full max-w-6xl px-6 py-20 md:py-28">
      <motion.div
        initial={{ opacity: 0, y: 18 }}
        whileInView={{ opacity: 1, y: 0 }}
        viewport={{ once: true, margin: '-80px' }}
        transition={{ duration: 0.45 }}
      >
        <p className="font-mono text-[11px] tracking-[0.32em] text-orbit-cyan">{kicker}</p>
        <h2 className="mt-3 max-w-3xl text-balance text-2xl font-medium tracking-tight text-orbit-text md:text-4xl">
          {title}
        </h2>
        <div className="mt-10">{children}</div>
      </motion.div>
    </section>
  )
}
