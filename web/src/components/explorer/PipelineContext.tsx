import type { ViewMode } from '../../types/orbit'

const stages: { id: string; modes: ViewMode[] }[] = [
  { id: 'LiDAR input', modes: ['lidar'] },
  { id: 'Ground detection', modes: ['lidar', 'grid'] },
  { id: 'Object proposals', modes: ['lidar', 'objects'] },
  { id: 'Adaptive mapping', modes: ['grid'] },
  { id: 'Tracking', modes: ['objects'] },
  { id: 'World model', modes: ['world', 'objects'] },
]

export default function PipelineContext({ mode }: { mode: ViewMode }) {
  return (
    <ol className="flex flex-wrap items-center gap-x-3 gap-y-2 text-[11px] tracking-wide text-orbit-dim">
      {stages.map((stage, i) => {
        const on = stage.modes.includes(mode)
        return (
          <li key={stage.id} className="flex items-center gap-3">
            {i > 0 ? <span className="text-orbit-line">→</span> : null}
            <span className={on ? 'text-orbit-cyan' : ''}>{stage.id}</span>
          </li>
        )
      })}
    </ol>
  )
}
