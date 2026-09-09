export default function LoadingState({
  label,
  onRetry,
  error,
}: {
  label: string
  error?: string | null
  onRetry?: () => void
}) {
  return (
    <div className="absolute inset-0 z-10 flex flex-col items-center justify-center bg-orbit-bg/80 px-6 text-center">
      <div className="relative mb-6 h-px w-48 overflow-hidden bg-orbit-line">
        <span className="absolute inset-y-0 w-16 animate-pulse bg-orbit-cyan" />
      </div>
      <p className="font-mono text-[11px] tracking-[0.32em] text-orbit-cyan">{label}</p>
      {error ? (
        <>
          <p className="mt-4 max-w-md text-sm leading-6 text-orbit-dim">{error}</p>
          {onRetry ? (
            <button
              type="button"
              onClick={onRetry}
              className="mt-6 rounded-full border border-orbit-cyan/50 px-5 py-2 text-sm text-orbit-cyan"
            >
              Retry
            </button>
          ) : null}
        </>
      ) : (
        <p className="mt-3 text-sm text-orbit-dim">Fetching exported PipelineState JSON…</p>
      )}
    </div>
  )
}
