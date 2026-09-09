import { useCallback, useEffect, useMemo, useRef, useState } from 'react'
import ExplorerChrome from '../components/ExplorerChrome'
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
import { isBlockedCell, planPath, planStatusLabel, pointAlongPath, type PlanResult } from '../lib/planPath'
import { prototypeMetrics } from '../lib/prototypeMetrics'
import type { AdaptiveCellRecord } from '../types/orbit'
import { PLAYBACK_MS } from '../types/orbit'

function pad(n: number): string {
  return String(n).padStart(2, '0')
}

type PickMode = 'start' | 'goal' | null

function emptyPlan(): PlanResult | null {
  return null
}

function xyLabel(xy: [number, number] | null): string {
  if (!xy) return 'not selected'
  return `(${xy[0].toFixed(1)}, ${xy[1].toFixed(1)})`
}

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
  const [framePlaying, setFramePlaying] = useState(false)
  const [pickMode, setPickMode] = useState<PickMode>(null)
  const [start, setStart] = useState<[number, number] | null>(null)
  const [goal, setGoal] = useState<[number, number] | null>(null)
  const [startCell, setStartCell] = useState<AdaptiveCellRecord | null>(null)
  const [goalCell, setGoalCell] = useState<AdaptiveCellRecord | null>(null)
  const [selectedIndex, setSelectedIndex] = useState<number | null>(null)
  const [plan, setPlan] = useState<PlanResult | null>(null)
  const [routePlaying, setRoutePlaying] = useState(false)
  const [playbackT, setPlaybackT] = useState(0)
  const playRaf = useRef(0)

  const cells = data.frame?.adaptive_cells ?? []
  const obstacles = data.frame?.obstacle_cells ?? []

  const clearPlan = useCallback(() => {
    setPlan(emptyPlan())
    setRoutePlaying(false)
    setPlaybackT(0)
  }, [])

  useEffect(() => {
    setFramePlaying(false)
    setPickMode(null)
    setStart(null)
    setGoal(null)
    setStartCell(null)
    setGoalCell(null)
    setSelectedIndex(null)
    clearPlan()
  }, [baseUrl, clearPlan])

  useEffect(() => {
    setPickMode(null)
    setStart(null)
    setGoal(null)
    setStartCell(null)
    setGoalCell(null)
    setSelectedIndex(null)
    clearPlan()
  }, [data.frameIndex, clearPlan])

  useEffect(() => {
    if (!framePlaying) return
    if (data.atEnd) {
      setFramePlaying(false)
      return
    }
    if (data.frameLoading) return
    const id = window.setTimeout(() => data.goNext(), PLAYBACK_MS)
    return () => window.clearTimeout(id)
  }, [framePlaying, data.atEnd, data.frameLoading, data.frameIndex, data.goNext])

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

  const locatePath = useCallback(() => {
    if (!start || !goal) return
    setRoutePlaying(false)
    setPlaybackT(0)
    if (startCell && isBlockedCell(startCell, obstacles)) {
      setPlan({
        status: 'blocked-start',
        path: [],
        lengthM: 0,
        explored: 0,
        start,
        goal,
        startIndex: null,
        goalIndex: null,
        traversable: 0,
        blocked: 0,
        note: 'Selected start is not a traversable exported GROUND cell.',
      })
      return
    }
    if (goalCell && isBlockedCell(goalCell, obstacles)) {
      setPlan({
        status: 'blocked-goal',
        path: [],
        lengthM: 0,
        explored: 0,
        start,
        goal,
        startIndex: null,
        goalIndex: null,
        traversable: 0,
        blocked: 0,
        note: 'Selected destination is not a traversable exported GROUND cell.',
      })
      return
    }
    setPlan(planPath(cells, obstacles, start, goal))
  }, [cells, obstacles, start, goal, startCell, goalCell])

  useEffect(() => {
    if (!routePlaying || plan?.status !== 'ready' || plan.path.length < 2) return
    let last = performance.now()
    const tick = (now: number) => {
      const dt = Math.min(0.05, (now - last) / 1000)
      last = now
      setPlaybackT((t) => {
        const next = t + dt / 8
        if (next >= 1) {
          setRoutePlaying(false)
          return 1
        }
        return next
      })
      playRaf.current = requestAnimationFrame(tick)
    }
    playRaf.current = requestAnimationFrame(tick)
    return () => cancelAnimationFrame(playRaf.current)
  }, [routePlaying, plan])

  const playbackXy = useMemo(
    () => (plan?.status === 'ready' ? pointAlongPath(plan.path, playbackT) : null),
    [plan, playbackT],
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
  const metrics = useMemo(() => prototypeMetrics(data.frame, data.manifest), [data.frame, data.manifest])

  const statusLabel = plan
    ? planStatusLabel(plan.status)
    : !start
      ? 'WAITING FOR START'
      : !goal
        ? 'WAITING FOR DESTINATION'
        : 'READY TO LOCATE PATH'

  const applyPick = (xy: [number, number], index: number | null) => {
    const cell = index != null ? cells[index] ?? null : null
    if (pickMode === 'start') {
      setStart(xy)
      setStartCell(cell)
      clearPlan()
      setPickMode('goal')
    } else if (pickMode === 'goal') {
      setGoal(xy)
      setGoalCell(cell)
      clearPlan()
      setPickMode(null)
    }
    setSelectedIndex(index)
  }

  const canvas = (
    <div className="absolute inset-0">
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
        <RenderErrorBoundary fallback={(err) => <LoadingState label="PLANNING VIEW FAILED" error={err.message} />}>
          <PlanningScene
            frame={data.frame}
            mode={mode}
            cameraView={cameraView}
            path={plan?.status === 'ready' ? plan.path : []}
            start={start}
            goal={goal}
            playbackXy={playbackXy}
            selectedIndex={selectedIndex}
            onPickCell={(index) => {
              const cell = cells[index]
              if (!cell) return
              applyPick(cell.center, index)
            }}
            onPickXy={(xy) => {
              if (!pickMode) return
              applyPick(xy, null)
            }}
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
            <p className="font-mono text-[10px] tracking-[0.32em] text-orbit-cyan">PATH PLANNING</p>
            <h1 className="mt-1 text-xl font-medium tracking-tight text-orbit-text md:text-2xl">
              Planning demonstration
            </h1>
            <p className="mt-2 max-w-md text-xs leading-5 text-orbit-dim">
              Select START, then DESTINATION, then Locate path. A* runs in the browser on exported
              occupancy — not the Python ORBIT runtime planner, and not live vehicle control.
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
              <p className="font-mono text-[10px] tracking-[0.28em] text-orbit-cyan">WORKFLOW</p>
              <div className="mt-3 flex flex-col gap-1.5">
                <button
                  type="button"
                  onClick={() => setPickMode('start')}
                  className={`rounded-full px-3 py-1.5 text-left text-xs ${
                    pickMode === 'start'
                      ? 'bg-emerald-400 text-orbit-bg'
                      : 'border border-orbit-line text-orbit-dim hover:text-orbit-text'
                  }`}
                >
                  1. Start
                </button>
                <button
                  type="button"
                  onClick={() => setPickMode('goal')}
                  className={`rounded-full px-3 py-1.5 text-left text-xs ${
                    pickMode === 'goal'
                      ? 'bg-orange-400 text-orbit-bg'
                      : 'border border-orbit-line text-orbit-dim hover:text-orbit-text'
                  }`}
                >
                  2. Destination
                </button>
              </div>
              <p className="mt-3 text-[11px] text-orbit-dim">
                Start: <span className="text-orbit-text">{xyLabel(start)}</span>
              </p>
              <p className="text-[11px] text-orbit-dim">
                Destination: <span className="text-orbit-text">{xyLabel(goal)}</span>
              </p>
              {data.frame?.pose.ego_xy ? (
                <button
                  type="button"
                  className="mt-2 text-left font-mono text-[10px] tracking-widest text-orbit-cyan underline"
                  onClick={() => {
                    const xy = data.frame!.pose.ego_xy
                    const nearest = cells.reduce(
                      (best, c) => {
                        const d = Math.hypot(c.center[0] - xy[0], c.center[1] - xy[1])
                        return d < best.d ? { c, d } : best
                      },
                      { c: null as AdaptiveCellRecord | null, d: Infinity },
                    )
                    setStart(nearest.c?.center ?? xy)
                    setStartCell(nearest.c)
                    clearPlan()
                  }}
                >
                  Use exported ego as start
                </button>
              ) : null}
              <button
                type="button"
                disabled={!start || !goal}
                onClick={locatePath}
                className="mt-3 w-full rounded-full bg-orbit-cyan py-2 font-mono text-[11px] tracking-[0.18em] text-orbit-bg disabled:cursor-not-allowed disabled:opacity-35"
              >
                Locate path
              </button>
              <p className="mt-3 font-mono text-xs tracking-wide text-orbit-text">{statusLabel}</p>
              {plan?.status === 'ready' ? (
                <p className="mt-1 text-[11px] text-orbit-dim">
                  {plan.path.length} cells · {plan.lengthM.toFixed(1)} m along A* waypoints
                </p>
              ) : null}
              {plan && plan.status !== 'ready' ? (
                <p className="mt-1 text-[11px] leading-4 text-orbit-dim">{plan.note}</p>
              ) : null}
            </div>
            <div className="rounded-2xl border border-orbit-line/80 bg-orbit-bg/80 px-3 py-3 backdrop-blur-md">
              <p className="font-mono text-[10px] tracking-[0.28em] text-orbit-cyan">PATH PLAYBACK</p>
              <p className="mt-2 text-[11px] leading-4 text-orbit-dim">
                Route playback visualization of the already-computed browser A* path. Not live
                driving and not Python ORBIT vehicle control.
              </p>
              <div className="mt-3 flex flex-wrap gap-1.5">
                <button
                  type="button"
                  disabled={plan?.status !== 'ready'}
                  onClick={() => setRoutePlaying(true)}
                  className="rounded-full border border-orbit-line px-3 py-1 text-[11px] text-orbit-text disabled:opacity-30"
                >
                  Play route
                </button>
                <button
                  type="button"
                  disabled={plan?.status !== 'ready'}
                  onClick={() => setRoutePlaying(false)}
                  className="rounded-full border border-orbit-line px-3 py-1 text-[11px] text-orbit-text disabled:opacity-30"
                >
                  Pause
                </button>
                <button
                  type="button"
                  disabled={plan?.status !== 'ready'}
                  onClick={() => {
                    setRoutePlaying(false)
                    setPlaybackT(0)
                  }}
                  className="rounded-full border border-orbit-line px-3 py-1 text-[11px] text-orbit-text disabled:opacity-30"
                >
                  Reset
                </button>
              </div>
            </div>
            <div className="rounded-2xl border border-orbit-line/80 bg-orbit-bg/80 px-3 py-3 backdrop-blur-md">
              <p className="font-mono text-[10px] tracking-[0.28em] text-orbit-cyan">CAMERA</p>
              <div className="mt-3 flex flex-wrap gap-1.5">
                {(
                  [
                    ['iso', 'Iso'],
                    ['top', 'Top'],
                    ['side', 'Side'],
                  ] as const
                ).map(([id, label]) => (
                  <button
                    key={id}
                    type="button"
                    onClick={() => setCameraView(id)}
                    className={`rounded-full px-3 py-1 text-[11px] ${
                      cameraView === id
                        ? 'bg-orbit-cyan/15 text-orbit-cyan'
                        : 'border border-orbit-line text-orbit-dim hover:text-orbit-text'
                    }`}
                  >
                    {label}
                  </button>
                ))}
              </div>
              <div className="mt-3 flex flex-col gap-1.5">
                {(
                  [
                    ['semantic', 'Semantic'],
                    ['obstacles', 'Obstacles'],
                    ['resolution', 'Resolution'],
                    ['terrain', 'Terrain'],
                  ] as const
                ).map(([id, label]) => (
                  <button
                    key={id}
                    type="button"
                    onClick={() => setMode(id)}
                    className={`rounded-full px-3 py-1.5 text-left text-xs ${
                      mode === id
                        ? 'bg-orbit-cyan text-orbit-bg'
                        : 'border border-orbit-line text-orbit-dim hover:text-orbit-text'
                    }`}
                  >
                    {label}
                  </button>
                ))}
              </div>
              {sampled ? (
                <p className="mt-3 text-[10px] leading-4 text-orbit-dim">
                  Drawing {MAX_RENDER_CELLS.toLocaleString()} of {cells.length.toLocaleString()} cells
                  (visualization sample). Planning uses the full export.
                </p>
              ) : null}
            </div>
          </div>
        ) : undefined
      }
      right={
        hasGrid ? (
          <div className="flex flex-col gap-3">
            <div className="rounded-2xl border border-orbit-line/80 bg-orbit-bg/80 px-4 py-3 backdrop-blur-md">
              <p className="font-mono text-[10px] tracking-[0.28em] text-orbit-cyan">OCCUPANCY</p>
              <ul className="mt-3 space-y-2 text-xs text-orbit-dim">
                <li className="flex items-center gap-2">
                  <span className="h-2 w-2 rounded-sm bg-emerald-400" /> GROUND — traversable
                </li>
                <li className="flex items-center gap-2">
                  <span className="h-2 w-2 rounded-sm bg-sky-300" /> MIXED — blocked
                </li>
                <li className="flex items-center gap-2">
                  <span className="h-2 w-2 rounded-sm bg-orange-400" /> OBSTACLE — blocked
                </li>
              </ul>
              <p className="mt-3 text-[11px] leading-5 text-orbit-dim">
                Neighbors: exported cells whose footprints touch. MIXED, OBSTACLE, obstacle_count&gt;0,
                and obstacle_cells are blocked. Unmapped space is not free.
              </p>
            </div>
            {metrics.length ? (
              <div className="rounded-2xl border border-orbit-line/80 bg-orbit-bg/80 px-4 py-3 backdrop-blur-md">
                <p className="font-mono text-[10px] tracking-[0.28em] text-orbit-cyan">EXPORT METRICS</p>
                <ul className="mt-3 space-y-2">
                  {metrics.slice(0, 8).map((row) => (
                    <li key={row.k} className="min-w-0">
                      <p className="font-mono text-[9px] tracking-[0.22em] text-orbit-dim">{row.k}</p>
                      <p className="mt-0.5 truncate text-sm text-orbit-text">{row.v}</p>
                    </li>
                  ))}
                </ul>
              </div>
            ) : null}
          </div>
        ) : undefined
      }
      footer={
        data.frame ? (
          <div className="space-y-3">
            <div className="rounded-2xl border border-orbit-line/80 bg-orbit-bg/85 px-4 py-3 backdrop-blur-md">
              <p className="font-mono text-[10px] tracking-[0.22em] text-orbit-cyan">{statusLabel}</p>
              {plan?.status === 'ready' ? (
                <p className="mt-1 text-sm text-orbit-text">
                  {plan.path.length} cells · {plan.lengthM.toFixed(1)} m
                </p>
              ) : null}
            </div>
            <div className="rounded-2xl border border-orbit-line/80 bg-orbit-bg/85 px-4 py-3 backdrop-blur-md">
              <FrameControls
                compact
                label={frameLabel}
                atStart={data.atStart}
                atEnd={data.atEnd}
                playing={framePlaying}
                onPrev={data.goPrev}
                onNext={data.goNext}
                onTogglePlay={() => setFramePlaying((p) => !p)}
              />
            </div>
          </div>
        ) : null
      }
    />
  )
}
