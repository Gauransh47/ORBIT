import { DATA_PREFIX } from '../types/orbit'

/** Same-origin prefix; Vite/Vercel proxy this to VITE_ORBIT_DATA_URL (avoids R2 CORS). */
export const ORBIT_DATA_PROXY_PREFIX = '/orbit-data'

/** Trim and strip trailing slashes from VITE_ORBIT_DATA_URL. Empty → local /data. */
export function normalizeOrbitDataOrigin(raw: unknown): string | null {
  if (typeof raw !== 'string') return null
  const trimmed = raw.trim().replace(/\/+$/, '')
  return trimmed.length ? trimmed : null
}

export function normalizeDataRelativePath(relativePath: string): string {
  return relativePath.replace(/^\/+/, '').replace(/^data\//, '')
}

export function joinDataUrl(root: string, relativePath: string): string {
  const origin = root.replace(/\/+$/, '')
  const path = normalizeDataRelativePath(relativePath)
  if (!path) return origin
  return `${origin}/${path}`
}

/**
 * When an external origin is set: absolute host, then same-origin `/orbit-data`,
 * then `/data` (committed scene-fixture on Vercel).
 */
export function candidateDataUrlsForOrigin(
  origin: string | null,
  relativePath: string,
  localPrefix = DATA_PREFIX,
): string[] {
  const path = normalizeDataRelativePath(relativePath)
  const local = joinDataUrl(localPrefix, path)
  if (!origin) return [local]
  const urls: string[] = []
  const push = (url: string) => {
    if (!urls.includes(url)) urls.push(url)
  }
  if (/^https?:\/\//i.test(origin)) {
    push(joinDataUrl(origin, path))
    push(joinDataUrl(ORBIT_DATA_PROXY_PREFIX, path))
  } else {
    push(joinDataUrl(origin, path))
  }
  push(local)
  return urls
}

export function orbitDataOrigin(): string | null {
  return normalizeOrbitDataOrigin(import.meta.env.VITE_ORBIT_DATA_URL)
}

/** Primary URL for a registry-relative path (`scene-0061/manifest.json`). */
export function getDataUrl(relativePath: string): string {
  const urls = candidateDataUrlsForOrigin(orbitDataOrigin(), relativePath)
  return urls[0]
}

export function candidateDataUrls(relativePath: string): string[] {
  return candidateDataUrlsForOrigin(orbitDataOrigin(), relativePath)
}
