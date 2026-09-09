/** Fetch JSON and reject SPA HTML fallbacks (missing files often return 200 index.html). */
export async function fetchJson<T>(url: string): Promise<T> {
  const response = await fetch(url)
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
