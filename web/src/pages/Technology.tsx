import PageShell from '../components/PageShell'

export default function Technology() {
  return (
    <PageShell kicker="TECHNOLOGY" title="Two runtimes, one pipeline">
      <ul className="list-disc space-y-2 pl-5">
        <li>Python ORBIT core: geometric perception, adaptive 2.5D grid, tracker, world model.</li>
        <li>Matplotlib dashboard: engineering console (unchanged by this website).</li>
        <li>Website: React, TypeScript, Vite, Tailwind, Three.js — visualization of exported JSON.</li>
        <li>Bridge: JSON export of PipelineState — not a live Python server.</li>
        <li>World frame: LiDAR frame 0, not nuScenes global.</li>
      </ul>
      <p className="mt-6 text-sm leading-6 text-orbit-dim">
        PointNet++ and sparse convolutional networks are planned future work for learned
        semantic perception. They are not part of the current stack.
      </p>
    </PageShell>
  )
}
