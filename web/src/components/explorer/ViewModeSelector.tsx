import type { ViewMode } from '../../types/orbit'

const modes: { id: ViewMode; label: string }[] = [
  { id: 'lidar', label: 'LiDAR' },
  { id: 'world', label: 'World' },
  { id: 'grid', label: 'Adaptive Grid' },
  { id: 'objects', label: 'Objects' },
]

export default function ViewModeSelector({
  mode,
  onChange,
}: {
  mode: ViewMode
  onChange: (mode: ViewMode) => void
}) {
  return (
    <div className="flex flex-wrap gap-2">
      {modes.map((item) => (
        <button
          key={item.id}
          type="button"
          onClick={() => onChange(item.id)}
          className={`rounded-full px-4 py-1.5 text-[12px] tracking-wide ${
            mode === item.id
              ? 'bg-orbit-cyan text-orbit-bg'
              : 'border border-orbit-line text-orbit-dim hover:text-orbit-text'
          }`}
        >
          {item.label}
        </button>
      ))}
    </div>
  )
}
