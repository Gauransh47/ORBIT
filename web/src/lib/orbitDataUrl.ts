import { DATA_PREFIX } from '../types/orbit'

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
 * Remote origin first (when set), then same-origin `/data/…` so committed
 * scene-fixture still works on Vercel if it was not uploaded to the host.
 */
export function candidateDataUrlsForOrigin(
  origin: string | null,
  relativePath: string,
  localPrefix = DATA_PREFIX,
): string[] {
  const path = normalizeDataRelativePath(relativePath)
  const local = joinDataUrl(localPrefix, path)
  if (!origin) return [local]
  const remote = joinDataUrl(origin, path)
  if (remote === local) return [local]
  return [remote, local]
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
