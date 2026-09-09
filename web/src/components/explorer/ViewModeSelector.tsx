import type { ViewMode } from '../../types/orbit'
import { modeUnavailableLabel } from '../../lib/capabilities'

const modes: { id: ViewMode; label: string }[] = [
  { id: 'lidar', label: 'LiDAR' },
  { id: 'world', label: 'World' },
  { id: 'grid', label: 'Adaptive Grid' },
  { id: 'objects', label: 'Objects' },
]

export default function ViewModeSelector({
  mode,
  onChange,
  available,
}: {
  mode: ViewMode
  onChange: (mode: ViewMode) => void
  available: Record<ViewMode, boolean>
}) {
  return (
    <div className="flex flex-wrap gap-2">
      {modes.map((item) => {
        const on = available[item.id]
        return (
          <button
            key={item.id}
            type="button"
            disabled={!on}
            title={!on ? modeUnavailableLabel(item.id) : undefined}
            onClick={() => on && onChange(item.id)}
            className={`rounded-full px-4 py-1.5 text-[12px] tracking-wide ${
              !on
                ? 'cursor-not-allowed border border-orbit-line/60 text-orbit-dim/40'
                : mode === item.id
                  ? 'bg-orbit-cyan text-orbit-bg'
                  : 'border border-orbit-line text-orbit-dim hover:text-orbit-text'
            }`}
          >
            {item.label}
          </button>
        )
      })}
    </div>
  )
}
