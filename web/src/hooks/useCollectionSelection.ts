import { useEffect, useMemo } from 'react'
import { useSearchParams } from 'react-router-dom'
import { findCollection, findDataset, useDatasetCatalog } from './useDatasetCatalog'
import { collectionIdFromParams, collectionQueryKey } from '../lib/dataPaths'
import type { CollectionKind } from '../types/datasets'

export function useCollectionSelection() {
  const [params, setSearchParams] = useSearchParams()
  const catalog = useDatasetCatalog()
  const datasetParam = params.get('dataset')?.trim() || null
  const collectionParam = collectionIdFromParams(params)

  const dataset = findDataset(catalog.datasets, datasetParam)
  const collection = findCollection(dataset, collectionParam)

  const selectionError = useMemo(() => {
    if (catalog.loading || catalog.error) return null
    if (datasetParam && !dataset) {
      return `Unknown dataset “${datasetParam}”.`
    }
    if (collectionParam && dataset && !collection) {
      return `Unknown ${dataset.collection_label} “${collectionParam}” for ${dataset.display_name}.`
    }
    if (collection && !collection.available) {
      return `${collection.label} is listed but has no exported JSON on this deployment.`
    }
    return null
  }, [catalog.error, catalog.loading, collection, collectionParam, dataset, datasetParam])

  useEffect(() => {
    if (catalog.loading || catalog.error) return
    if (datasetParam || collectionParam) return
    const firstDs = catalog.datasets.find((d) => d.available)
    const firstCol = firstDs?.collections.find((c) => c.available)
    if (!firstDs || !firstCol) return
    const next = new URLSearchParams()
    next.set('dataset', firstDs.id)
    next.set(collectionQueryKey(firstDs.collection_label), firstCol.id)
    setSearchParams(next, { replace: true })
  }, [catalog.datasets, catalog.error, catalog.loading, collectionParam, datasetParam, setSearchParams])

  useEffect(() => {
    if (catalog.loading || catalog.error) return
    if (!datasetParam || collectionParam) return
    const ds = findDataset(catalog.datasets, datasetParam)
    const first = ds?.collections.find((c) => c.available)
    if (!ds || !first) return
    const next = new URLSearchParams()
    next.set('dataset', ds.id)
    next.set(collectionQueryKey(ds.collection_label), first.id)
    setSearchParams(next, { replace: true })
  }, [catalog.datasets, catalog.error, catalog.loading, collectionParam, datasetParam, setSearchParams])

  useEffect(() => {
    if (catalog.loading || datasetParam || !collectionParam) return
    for (const ds of catalog.datasets) {
      const col = ds.collections.find((c) => c.id === collectionParam && c.available)
      if (col) {
        const next = new URLSearchParams()
        next.set('dataset', ds.id)
        next.set(collectionQueryKey(ds.collection_label), col.id)
        setSearchParams(next, { replace: true })
        return
      }
    }
  }, [catalog.datasets, catalog.loading, collectionParam, datasetParam, setSearchParams])

  const baseUrl = collection?.available ? collection.baseUrl : null

  const onSelect = (id: string, colId: string | null, kind: CollectionKind) => {
    const next = new URLSearchParams()
    next.set('dataset', id)
    if (colId) next.set(collectionQueryKey(kind), colId)
    setSearchParams(next)
  }

  return {
    catalog,
    datasetParam,
    collectionParam,
    dataset,
    collection,
    selectionError,
    baseUrl,
    onSelect,
  }
}
