/** Fetch JSON and reject SPA HTML fallbacks (missing files often return 200 index.html). */
export async function fetchJson<T>(url: string): Promise<T> {
  let response: Response
  try {
    response = await fetch(url)
  } catch {
    throw new Error(`Unable to fetch ${url} (network or CORS)`)
  }
  const text = await response.text()
  const trimmed = text.trimStart()
  if (trimmed.startsWith('<!') || trimmed.startsWith('<html')) {
    throw new Error(`Not JSON: ${url}`)
  }
  if (!response.ok) {
    throw new Error(`${response.status} ${response.statusText} for ${url}`)
  }
  try {
    return JSON.parse(text) as T
  } catch {
    throw new Error(`Invalid JSON at ${url}`)
  }
}

/** Try URLs in order (external host, then local `/data`). */
export async function fetchJsonFirst<T>(urls: string[]): Promise<T> {
  let last: Error | null = null
  for (const url of urls) {
    try {
      return await fetchJson<T>(url)
    } catch (err) {
      last = err instanceof Error ? err : new Error(String(err))
    }
  }
  throw last ?? new Error('No dataset URLs to fetch')
}
