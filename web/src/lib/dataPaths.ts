import type { CollectionKind } from '../types/datasets'
import { DATA_PREFIX } from '../types/orbit'

export function dataFileUrl(baseUrl: string, file: string): string {
  const root = baseUrl.replace(/\/$/, '')
  return `${root}/${file}`
}

export function candidateBaseUrls(paths: string[]): string[] {
  return paths.map((p) => `${DATA_PREFIX}/${p.replace(/^\/+/, '')}`)
}

export function collectionQueryKey(kind: CollectionKind): 'scene' | 'sequence' | 'environment' {
  if (kind === 'sequence') return 'sequence'
  if (kind === 'environment') return 'environment'
  return 'scene'
}

export function collectionIdFromParams(
  params: URLSearchParams,
): string | null {
  const value =
    params.get('scene')?.trim() ||
    params.get('sequence')?.trim() ||
    params.get('environment')?.trim() ||
    ''
  return value || null
}
