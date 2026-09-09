import { useEffect, useMemo, useState } from 'react'
import DatasetExplorer from '../components/explorer/DatasetExplorer'
import FrameControls from '../components/explorer/FrameControls'
import LoadingState from '../components/explorer/LoadingState'
import RenderErrorBoundary from '../components/RenderErrorBoundary'
import type { MapCameraView } from '../components/map/MapCameraRig'
import type { MapVizMode } from '../components/map/MapGrid'
import PlanningScene from '../components/planning/PlanningScene'
import { useCollectionSelection } from '../hooks/useCollectionSelection'
import { useOrbitData } from '../hooks/useOrbitData'
import { MAX_RENDER_CELLS } from '../lib/cellVisual'
import { planPath } from '../lib/planPath'
import { PLAYBACK_MS } from '../types/orbit'

function pad(n: number): string {
  return String(n).padStart(2, '0')
}

type PickMode = 'goal' | 'start'

export default function Planning() {
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
  const [cameraView, setCameraView] = useState<MapCameraView>('iso')
  const [mode, setMode] = useState<MapVizMode>('semantic')
  const [playing, setPlaying] = useState(false)
  const [pickMode, setPickMode] = useState<PickMode>('goal')
  const [start, setStart] = useState<[number, number] | null>(null)
  const [goal, setGoal] = useState<[number, number] | null>(null)
  const [selectedIndex, setSelectedIndex] = useState<number | null>(null)

  const cells = data.frame?.adaptive_cells ?? []
  const obstacles = data.frame?.obstacle_cells ?? []

  useEffect(() => {
    setPlaying(false)
    setGoal(null)
    setSelectedIndex(null)
    setStart(data.frame?.pose.ego_xy ?? null)
  }, [baseUrl])

  useEffect(() => {
    setGoal(null)
    setSelectedIndex(null)
    setStart(data.frame?.pose.ego_xy ?? null)
  }, [data.frameIndex, data.frame])

  useEffect(() => {
    if (!playing) return
    if (data.atEnd) {
      setPlaying(false)
      return
    }
    if (data.frameLoading) return
    const id = window.setTimeout(() => data.goNext(), PLAYBACK_MS)
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

  const plan = useMemo(
    () => planPath(cells, obstacles, start, goal),
    [cells, obstacles, start, goal],
  )

  const last = data.lastIndex ?? 0
  const frameLabel = `FRAME ${pad(data.frameIndex)} / ${pad(last)}`
  const overlayError = selectionError || catalog.error
  const sceneLabel = collection?.label ?? collectionParam ?? '—'
  const datasetLabel = dataset?.display_name ?? datasetParam ?? '—'
  const hasGrid = cells.length > 0
  const sampled = cells.length > MAX_RENDER_CELLS
  const datasetNotExported = Boolean(dataset && !catalog.loading && !dataset.available && !overlayError)
  const singleFrame = (data.indices?.length ?? 0) === 1

  const applyPick = (xy: [number, number], index: number | null) => {
    if (pickMode === 'start') {
      setStart(xy)
    } else {
      setGoal(xy)
    }
    setSelectedIndex(index)
  }

  return (
    <main className="relative h-svh overflow-hidden bg-orbit-bg pt-[4.25rem]">
      <div className="absolute inset-x-0 top-[4.25rem] bottom-0">
        {data.bootstrapping || (data.frameLoading && !data.frame && !overlayError) ? (
          <LoadingState label={`LOADING FRAME ${pad(data.frameIndex)}`} />
        ) : null}
        {overlayError ? <LoadingState label="UNABLE TO LOAD DATASET" error={overlayError} /> : null}
        {datasetNotExported ? (
          <LoadingState
            label="THIS DATASET HAS NOT BEEN EXPORTED YET"
            error={`${dataset?.display_name ?? 'This dataset'} has no exported JSON. Planning will not invent an environment.`}
          />
        ) : null}
        {data.bootstrapError && !overlayError ? (
          <LoadingState
            label="UNABLE TO LOAD SCENE"
            error={data.bootstrapError}
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
        {data.frame && !hasGrid && !overlayError && !data.bootstrapError && !data.frameError ? (
          <LoadingState
            label="NO ADAPTIVE GRID"
            error="Adaptive grid data is not available for this export. The planner does not invent occupancy."
          />
        ) : null}
        {data.frame && hasGrid && !overlayError && !data.bootstrapError ? (
          <div className="absolute inset-0">
            <RenderErrorBoundary
              fallback={(err) => (
                <LoadingState label="PLANNING VIEW FAILED" error={err.message} />
              )}
            >
              <PlanningScene
                frame={data.frame}
                mode={mode}
                cameraView={cameraView}
                path={plan.status === 'ready' ? plan.path : []}
                start={start}
                goal={goal}
                selectedIndex={selectedIndex}
                onPickCell={(index) => {
                  const cell = cells[index]
                  if (!cell) return
                  applyPick(cell.center, index)
                }}
                onPickXy={(xy) => applyPick(xy, null)}
              />
            </RenderErrorBoundary>
          </div>
        ) : null}
      </div>

      <div className="pointer-events-none relative z-20 flex h-full flex-col justify-between px-3 pb-3 pt-2 md:px-5">
        <div className="flex flex-col gap-3 lg:flex-row lg:items-start lg:justify-between">
          <div className="pointer-events-auto max-w-xl">
            <p className="font-mono text-[10px] tracking-[0.32em] text-orbit-cyan">PATH PLANNING</p>
            <h1 className="mt-1 text-xl font-medium tracking-tight text-orbit-text md:text-2xl">
              Planning demonstration
            </h1>
            <p className="mt-2 max-w-md text-xs leading-5 text-orbit-dim">
              Planning demonstration using exported ORBIT environment data. Routes are computed
              in the browser with A* — they are not produced by the Python ORBIT runtime.
            </p>
            <p className="mt-1 font-mono text-[11px] tracking-[0.16em] text-orbit-dim">
              {datasetLabel}
              {sceneLabel !== '—' ? ` · ${sceneLabel}` : ''}
              {` · ${frameLabel}`}
              {singleFrame ? ' · 1 FRAME EXPORT' : ''}
            </p>
          </div>
          <div className="pointer-events-auto w-full max-w-xl lg:w-[28rem]">
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

        <div className="flex flex-1 items-stretch justify-between gap-3 py-3">
          {hasGrid ? (
            <div className="pointer-events-auto flex w-[13.5rem] flex-col justify-center gap-3 self-center">
              <div className="rounded-2xl border border-orbit-line/80 bg-orbit-bg/75 px-3 py-3 backdrop-blur-md">
                <p className="font-mono text-[10px] tracking-[0.28em] text-orbit-cyan">CLICK SETS</p>
                <div className="mt-3 flex flex-col gap-1.5">
                  {(['goal', 'start'] as PickMode[]).map((id) => (
                    <button
                      key={id}
                      type="button"
                      onClick={() => setPickMode(id)}
                      className={`rounded-full px-3 py-1.5 text-left text-xs ${
                        pickMode === id
                          ? 'bg-orbit-cyan text-orbit-bg'
                          : 'border border-orbit-line text-orbit-dim hover:text-orbit-text'
                      }`}
                    >
                      {id === 'goal' ? 'Destination' : 'Start'}
                    </button>
                  ))}
                </div>
                <button
                  type="button"
                  className="mt-3 text-[11px] text-orbit-cyan"
                  onClick={() => {
                    setStart(data.frame?.pose.ego_xy ?? null)
                    setGoal(null)
                    setSelectedIndex(null)
                  }}
                >
                  Reset to ego start
                </button>
              </div>
              <div className="rounded-2xl border border-orbit-line/80 bg-orbit-bg/75 px-3 py-3 backdrop-blur-md">
                <p className="font-mono text-[10px] tracking-[0.28em] text-orbit-cyan">VIEW</p>
                <div className="mt-2 flex flex-wrap gap-1.5">
                  {(['semantic', 'obstacles', 'resolution', 'terrain'] as MapVizMode[]).map((id) => (
                    <button
                      key={id}
                      type="button"
                      onClick={() => setMode(id)}
                      className={`rounded-full px-3 py-1 text-[11px] ${
                        mode === id ? 'bg-orbit-cyan/15 text-orbit-cyan' : 'border border-orbit-line text-orbit-dim'
                      }`}
                    >
                      {id}
                    </button>
                  ))}
                </div>
                <div className="mt-2 flex flex-wrap gap-1.5">
                  {(['iso', 'top', 'side'] as MapCameraView[]).map((id) => (
                    <button
                      key={id}
                      type="button"
                      onClick={() => setCameraView(id)}
                      className={`rounded-full px-3 py-1 text-[11px] ${
                        cameraView === id ? 'bg-orbit-cyan/15 text-orbit-cyan' : 'border border-orbit-line text-orbit-dim'
                      }`}
                    >
                      {id}
                    </button>
                  ))}
                </div>
              </div>
            </div>
          ) : (
            <div />
          )}

          {hasGrid ? (
            <div className="pointer-events-auto hidden w-[16rem] flex-col justify-center gap-3 self-center sm:flex">
              <div className="rounded-2xl border border-orbit-line/80 bg-orbit-bg/75 px-4 py-3 backdrop-blur-md">
                <p className="font-mono text-[10px] tracking-[0.28em] text-orbit-cyan">REAL ORBIT DATA</p>
                <ul className="mt-2 space-y-1 text-[11px] leading-4 text-orbit-dim">
                  <li>adaptive_cells</li>
                  <li>obstacle_cells</li>
                  <li>pose.ego_xy</li>
                  <li>tracks / objects (not used as cost)</li>
                </ul>
                <p className="mt-3 font-mono text-[10px] tracking-[0.28em] text-orbit-warn">COMPUTED IN WEBSITE</p>
                <ul className="mt-2 space-y-1 text-[11px] leading-4 text-orbit-dim">
                  <li>GROUND vs blocked occupancy</li>
                  <li>A* search</li>
                  <li>route polyline</li>
                </ul>
              </div>
              <div className="rounded-2xl border border-orbit-line/80 bg-orbit-bg/75 px-4 py-3 backdrop-blur-md">
                <p className="font-mono text-[10px] tracking-[0.28em] text-orbit-cyan">PLAN</p>
                <p className="mt-2 text-sm text-orbit-text">
                  {plan.status === 'ready'
                    ? `${plan.lengthM.toFixed(1)} m · ${plan.path.length} cells · ${plan.explored} explored`
                    : plan.note || plan.status}
                </p>
                <p className="mt-2 font-mono text-[10px] text-orbit-dim">
                  TRAVERSABLE {plan.traversable} · BLOCKED {plan.blocked}
                </p>
                {sampled ? (
                  <p className="mt-2 text-[11px] leading-4 text-orbit-dim">
                    Drawing {MAX_RENDER_CELLS.toLocaleString()} of {cells.length.toLocaleString()} exported
                    cells (even sample). Search still uses the full export.
                  </p>
                ) : null}
                <p className="mt-2 text-[11px] leading-4 text-orbit-dim">
                  Legend: teal GROUND is traversable. Orange OBSTACLE / MIXED / obstacle_cells are blocked.
                </p>
              </div>
            </div>
          ) : (
            <div />
          )}
        </div>

        <div className="pointer-events-auto mx-auto w-full max-w-3xl space-y-3">
          {data.frame ? (
            <div className="grid grid-cols-2 gap-x-6 gap-y-3 rounded-2xl border border-orbit-line/80 bg-orbit-bg/80 px-5 py-3 backdrop-blur-md sm:grid-cols-3 lg:grid-cols-6">
              <div>
                <p className="font-mono text-[9px] tracking-[0.22em] text-orbit-dim">START</p>
                <p className="mt-1 text-sm text-orbit-text">
                  {start ? `${start[0].toFixed(1)}, ${start[1].toFixed(1)}` : '—'}
                </p>
              </div>
              <div>
                <p className="font-mono text-[9px] tracking-[0.22em] text-orbit-dim">GOAL</p>
                <p className="mt-1 text-sm text-orbit-text">
                  {goal ? `${goal[0].toFixed(1)}, ${goal[1].toFixed(1)}` : 'click map'}
                </p>
              </div>
              <div>
                <p className="font-mono text-[9px] tracking-[0.22em] text-orbit-dim">STATUS</p>
                <p className="mt-1 text-sm text-orbit-text">{plan.status}</p>
              </div>
              <div>
                <p className="font-mono text-[9px] tracking-[0.22em] text-orbit-dim">CELLS</p>
                <p className="mt-1 text-sm text-orbit-text">{cells.length.toLocaleString()}</p>
              </div>
              <div>
                <p className="font-mono text-[9px] tracking-[0.22em] text-orbit-dim">MAP FRAME</p>
                <p className="mt-1 text-sm text-orbit-text">
                  {data.frame.world_points_frame ?? data.manifest?.world_frame ?? 'lidar_frame_0'}
                </p>
              </div>
              <div>
                <p className="font-mono text-[9px] tracking-[0.22em] text-orbit-dim">SOURCE</p>
                <p className="mt-1 text-sm text-orbit-text">website A*</p>
              </div>
            </div>
          ) : null}
          {data.frame ? (
            <div className="rounded-2xl border border-orbit-line/80 bg-orbit-bg/80 px-4 py-3 backdrop-blur-md">
              <FrameControls
                label={frameLabel}
                atStart={data.atStart}
                atEnd={data.atEnd}
                playing={playing}
                onPrev={data.goPrev}
                onNext={data.goNext}
                onTogglePlay={() => setPlaying((p) => !p)}
              />
            </div>
          ) : null}
        </div>
      </div>
    </main>
  )
}
