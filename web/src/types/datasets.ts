export type CollectionKind = 'scene' | 'sequence' | 'environment'

export type DatasetCapabilities = {
  has_points?: boolean
  has_world_points?: boolean
  has_trajectory?: boolean
  has_adaptive_grid?: boolean
  has_tracks?: boolean
  has_objects?: boolean
  has_proposals?: boolean
  has_semantic_labels?: boolean
  has_ground_data?: boolean
  has_path_data?: boolean
}

export type DatasetCollectionSpec = {
  id: string
  label: string
  paths: string[]
}

export type DatasetSpec = {
  id: string
  display_name: string
  description: string
  source: string
  collection_label: CollectionKind
  status?: 'available' | 'planned'
  collections: DatasetCollectionSpec[]
  capabilities?: DatasetCapabilities
}

export type DatasetRegistryFile = {
  schema_version: number
  datasets: DatasetSpec[]
}

export type ResolvedCollection = DatasetCollectionSpec & {
  available: boolean
  baseUrl: string | null
}

export type ResolvedDataset = Omit<DatasetSpec, 'collections'> & {
  collections: ResolvedCollection[]
  available: boolean
}
