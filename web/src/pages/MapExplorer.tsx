import { useEffect, useMemo, useState } from 'react'
import ExplorerChrome from '../components/ExplorerChrome'
import RenderErrorBoundary from '../components/RenderErrorBoundary'
import DatasetExplorer from '../components/explorer/DatasetExplorer'
import FrameControls from '../components/explorer/FrameControls'
import LoadingState from '../components/explorer/LoadingState'
import MapInspector from '../components/map/MapInspector'
import MapLegend from '../components/map/MapLegend'
import MapScene from '../components/map/MapScene'
import type { MapCameraView } from '../components/map/MapCameraRig'
import type { MapVizMode } from '../components/map/MapGrid'
import { useCollectionSelection } from '../hooks/useCollectionSelection'
import { useOrbitData } from '../hooks/useOrbitData'
import { frameHasElevation, MAX_RENDER_CELLS } from '../lib/cellVisual'
import { foveationFromExport, prototypeMetrics, uniqueResolutions } from '../lib/prototypeMetrics'
import { PLAYBACK_MS } from '../types/orbit'

function pad(n: number): string {
  return String(n).padStart(2, '0')
}

const MODES: { id: MapVizMode; label: string }[] = [
  { id: 'terrain', label: 'Terrain' },
  { id: 'semantic', label: 'Semantic' },
  { id: 'resolution', label: 'Resolution' },
  { id: 'obstacles', label: 'Obstacles' },
]

const VIEWS: { id: MapCameraView; label: string }[] = [
  { id: 'iso', label: 'Iso' },
  { id: 'top', label: 'Top' },
  { id: 'side', label: 'Side' },
]

function Stat({ k, v }: { k: string; v: string }) {
  return (
    <div className="min-w-0">
      <p className="font-mono text-[9px] tracking-[0.22em] text-orbit-dim">{k}</p>
      <p className="mt-1 truncate text-sm text-orbit-text">{v}</p>
    </div>
  )
}

export default function MapExplorer() {
  const {
    catalog,
    datasetParam,
    collectionParam,
    dataset,
    collection,
    selectionError,
    baseUrl,
    onSelect,
  } = useCollectionSelection()
  const data = useOrbitData(selectionError ? null : baseUrl)
  const [mode, setMode] = useState<MapVizMode>('terrain')
  const [cameraView, setCameraView] = useState<MapCameraView>('iso')
  const [playing, setPlaying] = useState(false)
  const [selectedIndex, setSelectedIndex] = useState<number | null>(null)
  const [showTrajectory, setShowTrajectory] = useState(true)
  const [showObjects, setShowObjects] = useState(false)
  const [showObstacleCells, setShowObstacleCells] = useState(false)
  const [showWorldPoints, setShowWorldPoints] = useState(false)
  const [showFoveation, setShowFoveation] = useState(true)

  const cells = data.frame?.adaptive_cells ?? []
  const obstacleCells = data.frame?.obstacle_cells ?? []
  const hasElevation = frameHasElevation(cells)
  const hasGrid = cells.length > 0
  const sampled = cells.length > MAX_RENDER_CELLS
  const singleFrame = (data.indices?.length ?? 0) === 1
  const hasTrajectory = Boolean(data.trajectory?.samples?.length)
  const hasObjects = Boolean((data.frame?.tracks?.length ?? 0) + (data.frame?.world_objects?.length ?? 0))
  const hasWorldPoints = Boolean(data.frame?.world_points?.length)
  const hasObstacleOverlay = obstacleCells.length > 0
  const hasSemantic = cells.some((c) => Boolean(c.semantic_class))
  const resolutions = useMemo(() => uniqueResolutions(cells), [cells])
  const foveation = useMemo(
    () => foveationFromExport(cells, data.frame?.pose.ego_xy ?? null),
    [cells, data.frame?.pose.ego_xy],
  )
  const metrics = useMemo(
    () => prototypeMetrics(data.frame, data.manifest),
    [data.frame, data.manifest],
  )

  useEffect(() => {
    setSelectedIndex(null)
  }, [data.frameIndex])

  useEffect(() => {
    setPlaying(false)
    setSelectedIndex(null)
    setShowWorldPoints(false)
  }, [baseUrl])

  useEffect(() => {
    if (!playing) return
    if (data.atEnd) {
      setPlaying(false)
      return
    }
    if (data.frameLoading) return
    const id = window.setTimeout(() => {
      data.goNext()
    }, PLAYBACK_MS)
    return () => window.clearTimeout(id)
  }, [playing, data.atEnd, data.frameLoading, data.frameIndex, data.goNext])

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      const el = e.target as HTMLElement | null
      if (el && (el.tagName === 'INPUT' || el.tagName === 'TEXTAREA' || el.isContentEditable)) return
      if (e.key === 'ArrowLeft') {
        e.preventDefault()
        data.goPrev()
      }
      if (e.key === 'ArrowRight') {
        e.preventDefault()
        data.goNext()
      }
    }
    window.addEventListener('keydown', onKey)
    return () => window.removeEventListener('keydown', onKey)
  }, [data.goPrev, data.goNext])

  const last = data.lastIndex ?? 0
  const frameLabel = `FRAME ${pad(data.frameIndex)} / ${pad(last)}`
  const overlayError = selectionError || catalog.error
  const sceneLabel = collection?.label ?? collectionParam ?? '—'
  const datasetLabel = dataset?.display_name ?? datasetParam ?? '—'
  const selectedCell = selectedIndex != null ? cells[selectedIndex] ?? null : null
  const copyHint =
    'Place exports under web/public/data/nuscenes/scene-0061/ or the legacy path web/public/data/scene-0061/.'
  const datasetNotExported = Boolean(dataset && !catalog.loading && !dataset.available && !overlayError)
  const gridMissing = Boolean(data.frame && !hasGrid && !overlayError && !data.bootstrapError && !data.frameError)

  const canvas = (
    <div className="absolute inset-0">
      {data.bootstrapping || (data.frameLoading && !data.frame && !overlayError) ? (
        <LoadingState label={`LOADING FRAME ${pad(data.frameIndex)}`} />
      ) : null}
      {overlayError ? (
        <LoadingState
          label="UNABLE TO LOAD DATASET"
          error={
            overlayError.includes('no exported JSON')
              ? `${overlayError} The map will not invent terrain, obstacles, or objects.`
              : overlayError
          }
        />
      ) : null}
      {datasetNotExported ? (
        <LoadingState
          label="THIS DATASET HAS NOT BEEN EXPORTED YET"
          error={`${dataset?.display_name ?? 'This dataset'} is listed in the registry, but no exported JSON is available on this deployment.`}
        />
      ) : null}
      {data.bootstrapError && !overlayError ? (
        <LoadingState
          label="UNABLE TO LOAD SCENE"
          error={`${data.bootstrapError} ${copyHint}`}
          onRetry={() => void data.retryBootstrap()}
        />
      ) : null}
      {data.frameError && !data.bootstrapError && !overlayError ? (
        <LoadingState
          label={`UNABLE TO LOAD FRAME ${pad(data.frameIndex)}`}
          error={data.frameError}
          onRetry={data.retryFrame}
        />
      ) : null}
      {gridMissing ? (
        <LoadingState
          label="ADAPTIVE GRID UNAVAILABLE"
          error="Adaptive grid data is not available for this export. The map does not synthesize cells."
        />
      ) : null}
      {data.frame && hasGrid && !overlayError && !data.bootstrapError ? (
        <RenderErrorBoundary
          fallback={(err) => (
            <LoadingState
              label="MAP FAILED TO RENDER"
              error={`${err.message} Large adaptive grids are subsampled for drawing only; the export is unchanged.`}
            />
          )}
        >
          <MapScene
            frame={data.frame}
            mode={mode}
            cameraView={cameraView}
            selectedIndex={selectedIndex}
            onSelectIndex={setSelectedIndex}
            trajectory={data.trajectory}
            showTrajectory={showTrajectory && hasTrajectory}
            showObjects={showObjects && hasObjects}
            showWorldPoints={showWorldPoints && hasWorldPoints}
            showObstacleCells={showObstacleCells && hasObstacleOverlay}
            showFoveation={showFoveation}
          />
        </RenderErrorBoundary>
      ) : null}
    </div>
  )

  return (
    <ExplorerChrome
      canvas={canvas}
      header={
        <div className="flex flex-col gap-3 lg:flex-row lg:items-start lg:justify-between">
          <div>
            <p className="font-mono text-[10px] tracking-[0.32em] text-orbit-cyan">2.5D MAP</p>
            <h1 className="mt-1 text-xl font-medium tracking-tight text-orbit-text md:text-2xl">
              Adaptive Grid Map
            </h1>
            <p className="mt-1 max-w-xl text-xs leading-5 text-orbit-dim">
              ORBIT demonstrates an adaptive variable-resolution 2.5D spatial representation built
              from real LiDAR-derived/exported environment data.
            </p>
            <p className="mt-1 font-mono text-[11px] tracking-[0.16em] text-orbit-dim">
              {datasetLabel}
              {sceneLabel !== '—' ? ` · ${sceneLabel}` : ''}
              {` · ${frameLabel}`}
              {singleFrame ? ' · 1 FRAME EXPORT' : ''}
            </p>
          </div>
          <div className="w-full max-w-xl lg:w-[28rem]">
            <DatasetExplorer
              compact
              datasets={catalog.datasets}
              loading={catalog.loading}
              datasetId={datasetParam}
              collectionId={collectionParam}
              onSelect={onSelect}
            />
          </div>
        </div>
      }
      left={
        hasGrid ? (
          <div className="flex flex-col gap-3">
            <div className="rounded-2xl border border-orbit-line/80 bg-orbit-bg/80 px-3 py-3 backdrop-blur-md">
              <p className="font-mono text-[10px] tracking-[0.28em] text-orbit-cyan">VIEW MODE</p>
              <div className="mt-3 flex flex-col gap-1.5">
                {MODES.map((m) => (
                  <button
                    key={m.id}
                    type="button"
                    onClick={() => setMode(m.id)}
                    className={`rounded-full px-3 py-1.5 text-left text-xs ${
                      mode === m.id
                        ? 'bg-orbit-cyan text-orbit-bg'
                        : 'border border-orbit-line text-orbit-dim hover:text-orbit-text'
                    }`}
                  >
                    {m.label}
                  </button>
                ))}
              </div>
              {mode === 'semantic' && !hasSemantic ? (
                <p className="mt-2 text-[11px] leading-4 text-orbit-dim">No semantic_class values in this frame.</p>
              ) : null}
              {mode === 'terrain' && !hasElevation ? (
                <p className="mt-2 text-[11px] leading-4 text-orbit-dim">
                  Elevation unavailable. Showing a flat adaptive grid.
                </p>
              ) : null}
            </div>
            <div className="rounded-2xl border border-orbit-line/80 bg-orbit-bg/80 px-3 py-3 backdrop-blur-md">
              <p className="font-mono text-[10px] tracking-[0.28em] text-orbit-cyan">CAMERA</p>
              <div className="mt-3 flex flex-wrap gap-1.5">
                {VIEWS.map((v) => (
                  <button
                    key={v.id}
                    type="button"
                    onClick={() => setCameraView(v.id)}
                    className={`rounded-full px-3 py-1 text-[11px] ${
                      cameraView === v.id
                        ? 'bg-orbit-cyan/15 text-orbit-cyan'
                        : 'border border-orbit-line text-orbit-dim hover:text-orbit-text'
                    }`}
                  >
                    {v.label}
                  </button>
                ))}
              </div>
              {sampled ? (
                <p className="mt-3 text-[10px] leading-4 text-orbit-dim">
                  Drawing {MAX_RENDER_CELLS.toLocaleString()} of {cells.length.toLocaleString()} exported
                  cells (evenly spaced visualization sample).
                </p>
              ) : null}
              <p className="mt-3 text-[10px] leading-4 tracking-wide text-orbit-dim">
                Drag rotate · scroll zoom · right-drag pan
              </p>
            </div>
            <div className="rounded-2xl border border-orbit-line/80 bg-orbit-bg/80 px-3 py-3 backdrop-blur-md">
              <p className="font-mono text-[10px] tracking-[0.28em] text-orbit-cyan">OVERLAYS</p>
              <label className="mt-3 flex items-center gap-2 text-xs text-orbit-dim">
                <input
                  type="checkbox"
                  className="accent-orbit-cyan"
                  checked={showFoveation}
                  onChange={(e) => setShowFoveation(e.target.checked)}
                />
                Adaptive structure
              </label>
              <label className={`mt-2 flex items-center gap-2 text-xs ${hasTrajectory ? 'text-orbit-dim' : 'text-orbit-dim/40'}`}>
                <input
                  type="checkbox"
                  className="accent-orbit-cyan"
                  checked={showTrajectory && hasTrajectory}
                  disabled={!hasTrajectory}
                  onChange={(e) => setShowTrajectory(e.target.checked)}
                />
                Ego trajectory
              </label>
              <label className={`mt-2 flex items-center gap-2 text-xs ${hasObjects ? 'text-orbit-dim' : 'text-orbit-dim/40'}`}>
                <input
                  type="checkbox"
                  className="accent-orbit-cyan"
                  checked={showObjects && hasObjects}
                  disabled={!hasObjects}
                  onChange={(e) => setShowObjects(e.target.checked)}
                />
                Tracked objects
              </label>
              <label className={`mt-2 flex items-center gap-2 text-xs ${hasObstacleOverlay ? 'text-orbit-dim' : 'text-orbit-dim/40'}`}>
                <input
                  type="checkbox"
                  className="accent-orbit-cyan"
                  checked={showObstacleCells && hasObstacleOverlay}
                  disabled={!hasObstacleOverlay}
                  onChange={(e) => setShowObstacleCells(e.target.checked)}
                />
                Obstacle cells
              </label>
              <label className={`mt-2 flex items-center gap-2 text-xs ${hasWorldPoints ? 'text-orbit-dim' : 'text-orbit-dim/40'}`}>
                <input
                  type="checkbox"
                  className="accent-orbit-cyan"
                  checked={showWorldPoints && hasWorldPoints}
                  disabled={!hasWorldPoints}
                  onChange={(e) => setShowWorldPoints(e.target.checked)}
                />
                World points
              </label>
            </div>
          </div>
        ) : undefined
      }
      right={
        hasGrid ? (
          <div className="flex flex-col gap-3">
            <div className="rounded-2xl border border-orbit-line/80 bg-orbit-bg/80 px-4 py-3 backdrop-blur-md">
              <MapLegend
                mode={mode}
                hasElevation={hasElevation}
                hasObstacles={
                  cells.some((c) => c.semantic_class === 'OBSTACLE' || (c.obstacle_count ?? 0) > 0) ||
                  hasObstacleOverlay
                }
                resolutions={resolutions}
                semanticClasses={[...new Set(cells.map((c) => c.semantic_class).filter(Boolean) as string[])]}
              />
            </div>
            <div className="rounded-2xl border border-orbit-line/80 bg-orbit-bg/80 px-4 py-3 backdrop-blur-md">
              <p className="font-mono text-[10px] tracking-[0.28em] text-orbit-cyan">ADAPTIVE STRUCTURE</p>
              <p className="mt-2 text-[11px] leading-5 text-orbit-dim">
                ORBIT allocates spatial detail non-uniformly instead of using a uniform grid. Rings
                mark the actual range extent of each exported resolution around ego — not assumed
                10 m / 100 m radii.
              </p>
              <ul className="mt-2 space-y-1 text-[11px] text-orbit-dim">
                {foveation.slice(0, 6).map((row) => (
                  <li key={row.resolution}>
                    {row.resolution < 1 ? `${Math.round(row.resolution * 100)} cm` : `${row.resolution} m`}
                    {' · '}
                    {row.count.toLocaleString()} cells · {row.minRange.toFixed(1)}–{row.maxRange.toFixed(1)} m
                  </li>
                ))}
              </ul>
            </div>
            <div className="rounded-2xl border border-orbit-line/80 bg-orbit-bg/80 px-4 py-3 backdrop-blur-md">
              <MapInspector cell={selectedCell} />
            </div>
          </div>
        ) : undefined
      }
      footer={
        data.frame ? (
          <div className="space-y-2">
            {metrics.length ? (
              <div className="grid grid-cols-2 gap-x-5 gap-y-1 sm:grid-cols-4 lg:grid-cols-6">
                {metrics.map((row) => (
                  <Stat key={row.k} k={row.k} v={row.v} />
                ))}
              </div>
            ) : null}
            <div className="flex flex-wrap items-end justify-between gap-3">
              <div className="grid min-w-0 flex-1 grid-cols-2 gap-x-5 gap-y-1 sm:grid-cols-4">
                <Stat k="DATASET" v={datasetLabel} />
                <Stat k="COLLECTION" v={sceneLabel} />
                <Stat k="FRAME" v={`${pad(data.frameIndex)} / ${pad(last)}`} />
                <Stat k="MAP FRAME" v={data.frame.world_points_frame ?? data.manifest?.world_frame ?? 'lidar_frame_0'} />
              </div>
              <FrameControls
                compact
                label={frameLabel}
                atStart={data.atStart}
                atEnd={data.atEnd}
                playing={playing}
                onPrev={data.goPrev}
                onNext={data.goNext}
                onTogglePlay={() => setPlaying((p) => !p)}
              />
            </div>
          </div>
        ) : null
      }
    />
  )
}
