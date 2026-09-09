import { useEffect, useState } from 'react'
import { fetchJson, fetchJsonFirst } from '../lib/fetchJson'
import { candidateBaseUrls, dataFileUrl } from '../lib/dataPaths'
import { candidateDataUrls } from '../lib/orbitDataUrl'
import type {
  DatasetRegistryFile,
  DatasetSpec,
  ResolvedCollection,
  ResolvedDataset,
} from '../types/datasets'
import type { Manifest } from '../types/orbit'

async function resolveCollection(
  spec: DatasetSpec['collections'][number],
): Promise<ResolvedCollection> {
  for (const base of candidateBaseUrls(spec.paths)) {
    try {
      const manifest = await fetchJson<Manifest>(dataFileUrl(base, 'manifest.json'))
      if (Array.isArray(manifest.files)) {
        return { ...spec, available: true, baseUrl: base }
      }
    } catch {
      continue
    }
  }
  return { ...spec, available: false, baseUrl: null }
}

export function useDatasetCatalog() {
  const [datasets, setDatasets] = useState<ResolvedDataset[]>([])
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)

  useEffect(() => {
    let cancelled = false
    ;(async () => {
      setLoading(true)
      setError(null)
      try {
        const registry = await fetchJsonFirst<DatasetRegistryFile>(
          candidateDataUrls('datasets.json'),
        )
        const resolved: ResolvedDataset[] = []
        for (const ds of registry.datasets ?? []) {
          const collections: ResolvedCollection[] = []
          for (const col of ds.collections ?? []) {
            collections.push(await resolveCollection(col))
          }
          const available = collections.some((c) => c.available)
          resolved.push({ ...ds, collections, available })
        }
        if (!cancelled) setDatasets(resolved)
      } catch (err) {
        if (!cancelled) {
          setDatasets([])
          setError(err instanceof Error ? err.message : 'Unable to load dataset registry')
        }
      } finally {
        if (!cancelled) setLoading(false)
      }
    })()
    return () => {
      cancelled = true
    }
  }, [])

  return { datasets, loading, error }
}

export function findDataset(datasets: ResolvedDataset[], id: string | null) {
  if (!id) return undefined
  return datasets.find((d) => d.id === id)
}

export function findCollection(dataset: ResolvedDataset | undefined, collectionId: string | null) {
  if (!dataset || !collectionId) return undefined
  return dataset.collections.find((c) => c.id === collectionId)
}
