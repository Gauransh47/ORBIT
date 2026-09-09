import type { CollectionKind, ResolvedDataset } from '../../types/datasets'

export default function DatasetExplorer({
  datasets,
  loading,
  datasetId,
  collectionId,
  onSelect,
}: {
  datasets: ResolvedDataset[]
  loading: boolean
  datasetId: string | null
  collectionId: string | null
  onSelect: (datasetId: string, collectionId: string | null, kind: CollectionKind) => void
}) {
  const selected = datasets.find((d) => d.id === datasetId)

  return (
    <div className="rounded-[1.5rem] bg-[#0a1018] px-5 py-4">
      <p className="font-mono text-[10px] tracking-[0.28em] text-orbit-dim">EXPLORE DATA</p>
      {loading ? (
        <p className="mt-3 text-sm text-orbit-dim">Reading dataset registry…</p>
      ) : (
        <div className="mt-3 flex flex-wrap gap-2">
          {datasets.map((ds) => {
            const active = ds.id === datasetId
            return (
              <button
                key={ds.id}
                type="button"
                onClick={() => {
                  const first = ds.collections.find((c) => c.available)
                  onSelect(ds.id, first?.id ?? null, ds.collection_label)
                }}
                className={`max-w-[16rem] rounded-2xl px-4 py-3 text-left ${
                  active
                    ? 'bg-orbit-cyan/12 ring-1 ring-orbit-cyan/50'
                    : 'ring-1 ring-orbit-line hover:ring-orbit-cyan/30'
                }`}
              >
                <span className="block text-sm text-orbit-text">{ds.display_name}</span>
                <span className="mt-1 block text-[11px] leading-4 text-orbit-dim">{ds.description}</span>
                {!ds.available ? (
                  <span className="mt-2 block font-mono text-[10px] tracking-wide text-orbit-warn">
                    Not exported yet
                  </span>
                ) : null}
              </button>
            )
          })}
        </div>
      )}

      {selected ? (
        <div className="mt-4">
          <p className="font-mono text-[10px] tracking-[0.22em] text-orbit-dim">
            {selected.collection_label.toUpperCase()}
          </p>
          {selected.collections.some((c) => c.available) ? (
            <div className="mt-2 flex flex-wrap gap-2">
              {selected.collections
                .filter((c) => c.available)
                .map((col) => (
                  <button
                    key={col.id}
                    type="button"
                    onClick={() => onSelect(selected.id, col.id, selected.collection_label)}
                    className={`rounded-full px-3 py-1.5 text-[12px] ${
                      col.id === collectionId
                        ? 'bg-orbit-cyan text-orbit-bg'
                        : 'border border-orbit-line text-orbit-dim hover:text-orbit-text'
                    }`}
                  >
                    {col.label}
                  </button>
                ))}
            </div>
          ) : (
            <p className="mt-2 text-sm leading-6 text-orbit-dim">
              Dataset support is planned. No exported {selected.collection_label} is available yet.
              The explorer will not invent placeholder data.
            </p>
          )}
        </div>
      ) : null}
    </div>
  )
}
