export default function FrameControls({
  label,
  atStart,
  atEnd,
  playing,
  onPrev,
  onNext,
  onTogglePlay,
  compact,
}: {
  label: string
  atStart: boolean
  atEnd: boolean
  playing: boolean
  onPrev: () => void
  onNext: () => void
  onTogglePlay: () => void
  compact?: boolean
}) {
  const btn =
    'rounded-full border border-orbit-line px-3 py-1.5 text-sm text-orbit-text disabled:cursor-not-allowed disabled:opacity-30 hover:border-orbit-cyan/40 md:px-4'
  return (
    <div className={`flex flex-col items-center ${compact ? 'gap-2' : 'gap-4'}`}>
      <div className="flex flex-wrap items-center justify-center gap-2">
        <button type="button" className={`${btn} min-w-10 font-mono text-lg`} onClick={onPrev} disabled={atStart} aria-label="Previous frame">
          −
        </button>
        <p className="min-w-[9rem] text-center font-mono text-xs tracking-[0.18em] text-orbit-cyan md:text-sm">
          {label}
        </p>
        <button type="button" className={`${btn} min-w-10 font-mono text-lg`} onClick={onNext} disabled={atEnd} aria-label="Next frame">
          +
        </button>
        <button type="button" className={btn} onClick={onPrev} disabled={atStart}>
          Previous
        </button>
        <button type="button" className={btn} onClick={onTogglePlay} disabled={atEnd && !playing}>
          {playing ? 'Pause' : 'Play'}
        </button>
        <button type="button" className={btn} onClick={onNext} disabled={atEnd}>
          Next
        </button>
      </div>
      {compact ? null : (
        <p className="text-[10px] tracking-[0.2em] text-orbit-dim">PLAYBACK OF EXPORTED FRAMES — NOT LIVE PROCESSING</p>
      )}
    </div>
  )
}
