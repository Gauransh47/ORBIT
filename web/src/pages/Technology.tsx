import PageShell from '../components/PageShell'

export default function Technology() {
  return (
    <PageShell kicker="TECHNOLOGY" title="Two runtimes, one pipeline">
      <ul className="list-disc space-y-2 pl-5">
        <li>Python ORBIT core: perception, adaptive grid, tracker, world model.</li>
        <li>Matplotlib dashboard: engineering console (unchanged by this website).</li>
        <li>This site: React, TypeScript, Vite, Tailwind, Three.js (later phases).</li>
        <li>Bridge: JSON export of PipelineState — not a live Python server.</li>
        <li>World frame: LiDAR frame 0, not nuScenes global.</li>
      </ul>
    </PageShell>
  )
}
