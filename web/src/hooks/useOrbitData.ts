import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import { FrameCache } from '../lib/frameCache'
import { fetchJson } from '../lib/fetchJson'
import { dataFileUrl } from '../lib/dataPaths'
import type { FrameJson, Manifest, TrajectoryFile } from '../types/orbit'

export function useOrbitData(baseUrl: string | null) {
  const cacheRef = useRef(new FrameCache())
  const [manifest, setManifest] = useState<Manifest | null>(null)
  const [trajectory, setTrajectory] = useState<TrajectoryFile | null>(null)
  const [bootstrapError, setBootstrapError] = useState<string | null>(null)
  const [bootstrapping, setBootstrapping] = useState(Boolean(baseUrl))
  const [frameIndex, setFrameIndex] = useState(0)
  const [frame, setFrame] = useState<FrameJson | null>(null)
  const [frameError, setFrameError] = useState<string | null>(null)
  const [frameLoading, setFrameLoading] = useState(false)
  const [fromCache, setFromCache] = useState(false)

  const indices = useMemo(() => {
    if (manifest?.frame_indices?.length) return manifest.frame_indices
    return []
  }, [manifest])

  const cursor = Math.max(0, indices.indexOf(frameIndex))
  const firstIndex = indices[0]
  const lastIndex = indices[indices.length - 1]
  const atStart = indices.length === 0 || cursor <= 0
  const atEnd = indices.length === 0 || cursor >= indices.length - 1

  const requestRef = useRef(0)

  const loadFrame = useCallback(
    async (index: number, fileName: string) => {
      if (!baseUrl) return
      const request = ++requestRef.current
      const cached = cacheRef.current.get(index)
      if (cached) {
        setFrame(cached)
        setFrameError(null)
        setFromCache(true)
        setFrameLoading(false)
        return
      }
      setFrameLoading(true)
      setFromCache(false)
      setFrameError(null)
      try {
        const json = await fetchJson<FrameJson>(dataFileUrl(baseUrl, fileName))
        if (request !== requestRef.current) return
        cacheRef.current.set(index, json)
        setFrame(json)
      } catch (err) {
        if (request !== requestRef.current) return
        setFrame(null)
        setFrameError(err instanceof Error ? err.message : 'Unable to load frame data.')
      } finally {
        if (request === requestRef.current) setFrameLoading(false)
      }
    },
    [baseUrl],
  )

  const bootstrap = useCallback(async () => {
    if (!baseUrl) {
      setBootstrapping(false)
      setManifest(null)
      setTrajectory(null)
      setFrame(null)
      return
    }
    setBootstrapping(true)
    setBootstrapError(null)
    cacheRef.current = new FrameCache()
    try {
      const man = await fetchJson<Manifest>(dataFileUrl(baseUrl, 'manifest.json'))
      let traj: TrajectoryFile | null = null
      try {
        traj = await fetchJson<TrajectoryFile>(dataFileUrl(baseUrl, 'trajectory.json'))
      } catch {
        traj = null
      }
      setManifest(man)
      setTrajectory(traj)
      const start = man.frame_indices?.[0] ?? 0
      setFrameIndex(start)
      const file = man.files?.[0] ?? `frame_${String(start).padStart(4, '0')}.json`
      await loadFrame(start, file)
    } catch (err) {
      setManifest(null)
      setTrajectory(null)
      setFrame(null)
      setBootstrapError(err instanceof Error ? err.message : 'Unable to load manifest.json')
    } finally {
      setBootstrapping(false)
    }
  }, [loadFrame, baseUrl])

  useEffect(() => {
    void bootstrap()
  }, [bootstrap])

  const fileForIndex = useCallback(
    (index: number) => {
      if (!manifest) return `frame_${String(index).padStart(4, '0')}.json`
      const pos = manifest.frame_indices.indexOf(index)
      if (pos >= 0 && manifest.files[pos]) return manifest.files[pos]
      return `frame_${String(index).padStart(4, '0')}.json`
    },
    [manifest],
  )

  const goTo = useCallback(
    (index: number) => {
      if (!indices.length) return
      if (!indices.includes(index)) return
      setFrameIndex(index)
      void loadFrame(index, fileForIndex(index))
    },
    [fileForIndex, indices, loadFrame],
  )

  const goPrev = useCallback(() => {
    if (atStart) return
    goTo(indices[cursor - 1])
  }, [atStart, cursor, goTo, indices])

  const goNext = useCallback(() => {
    if (atEnd) return
    goTo(indices[cursor + 1])
  }, [atEnd, cursor, goTo, indices])

  const retryFrame = useCallback(() => {
    void loadFrame(frameIndex, fileForIndex(frameIndex))
  }, [fileForIndex, frameIndex, loadFrame])

  return {
    baseUrl,
    manifest,
    trajectory,
    bootstrapping,
    bootstrapError,
    retryBootstrap: bootstrap,
    frameIndex,
    frame,
    frameLoading,
    frameError,
    fromCache,
    retryFrame,
    indices,
    cursor,
    firstIndex,
    lastIndex,
    atStart,
    atEnd,
    goTo,
    goPrev,
    goNext,
  }
}
