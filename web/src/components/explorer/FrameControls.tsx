export default function FrameControls({
  label,
  atStart,
  atEnd,
  playing,
  onPrev,
  onNext,
  onTogglePlay,
}: {
  label: string
  atStart: boolean
  atEnd: boolean
  playing: boolean
  onPrev: () => void
  onNext: () => void
  onTogglePlay: () => void
}) {
  const btn =
    'rounded-full border border-orbit-line px-4 py-2 text-sm text-orbit-text disabled:cursor-not-allowed disabled:opacity-30 hover:border-orbit-cyan/40'
  return (
    <div className="flex flex-col items-center gap-4">
      <div className="flex items-center gap-4">
        <button type="button" className={`${btn} min-w-12 font-mono text-lg`} onClick={onPrev} disabled={atStart} aria-label="Previous frame">
          −
        </button>
        <p className="min-w-[10rem] text-center font-mono text-sm tracking-[0.2em] text-orbit-cyan">
          {label}
        </p>
        <button type="button" className={`${btn} min-w-12 font-mono text-lg`} onClick={onNext} disabled={atEnd} aria-label="Next frame">
          +
        </button>
      </div>
      <div className="flex flex-wrap items-center justify-center gap-2">
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
      <p className="text-[10px] tracking-[0.2em] text-orbit-dim">PLAYBACK OF EXPORTED FRAMES — NOT LIVE PROCESSING</p>
    </div>
  )
}
