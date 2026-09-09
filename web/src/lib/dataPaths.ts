import type { CollectionKind } from '../types/datasets'
import { candidateDataUrls } from './orbitDataUrl'

export function dataFileUrl(baseUrl: string, file: string): string {
  const root = baseUrl.replace(/\/$/, '')
  const name = file.replace(/^\/+/, '')
  return `${root}/${name}`
}

/** Collection roots to probe (`…/scene-0061`), remote then local when configured. */
export function candidateBaseUrls(paths: string[]): string[] {
  const seen = new Set<string>()
  const out: string[] = []
  for (const p of paths) {
    for (const url of candidateDataUrls(p)) {
      if (seen.has(url)) continue
      seen.add(url)
      out.push(url)
    }
  }
  return out
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
