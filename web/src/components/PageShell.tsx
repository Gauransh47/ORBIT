import type { ReactNode } from 'react'

export default function PageShell({
  kicker,
  title,
  children,
}: {
  kicker: string
  title: string
  children: ReactNode
}) {
  return (
    <main className="mx-auto max-w-5xl px-6 pb-24 pt-28">
      <p className="font-mono text-xs tracking-[0.28em] text-orbit-cyan">{kicker}</p>
      <h1 className="mt-3 text-3xl font-medium tracking-tight text-orbit-text md:text-4xl">
        {title}
      </h1>
      <div className="mt-8 space-y-6 text-sm leading-7 text-orbit-dim">{children}</div>
    </main>
  )
}
