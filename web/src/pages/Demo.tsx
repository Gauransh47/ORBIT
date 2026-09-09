import { useEffect, useMemo, useState } from 'react'
import { useSearchParams } from 'react-router-dom'
import ExplorerScene from '../components/explorer/ExplorerScene'
import FrameControls from '../components/explorer/FrameControls'
import InformationPanel from '../components/explorer/InformationPanel'
import LoadingState from '../components/explorer/LoadingState'
import PipelineContext from '../components/explorer/PipelineContext'
import ViewModeSelector from '../components/explorer/ViewModeSelector'
import { useOrbitData } from '../hooks/useOrbitData'
import {
  DEFAULT_SCENE,
  PLAYBACK_MS,
  type TrackRecord,
  type ViewMode,
  type WorldObjectRecord,
} from '../types/orbit'

function pad(n: number): string {
  return String(n).padStart(2, '0')
}

function selectedDetails(
  track: TrackRecord | undefined,
  world: WorldObjectRecord | undefined,
): { title: string; rows: { k: string; v: string }[] } | null {
  const src = track ?? world
  if (!src) return null
  const rows: { k: string; v: string }[] = []
  rows.push({ k: 'CLASS', v: src.class_name })
  if (src.motion_state) rows.push({ k: 'MOTION', v: src.motion_state })
  if (src.confidence !== undefined) rows.push({ k: 'CONFIDENCE', v: src.confidence.toFixed(3) })
  if (src.age !== undefined) rows.push({ k: 'AGE', v: String(src.age) })
  if (src.hits !== undefined) rows.push({ k: 'HITS', v: String(src.hits) })
  if (src.missed !== undefined) rows.push({ k: 'MISSED', v: String(src.missed) })
  if ('confirmed' in src && src.confirmed !== undefined) {
    rows.push({ k: 'CONFIRMED', v: src.confirmed ? 'true' : 'false' })
  }
  rows.push({
    k: 'POSITION XY',
    v: `${src.position[0].toFixed(2)}, ${src.position[1].toFixed(2)}`,
  })
  const dims = src.dimensions_xy
  if (dims && dims.length >= 2) {
    rows.push({ k: 'DIMENSIONS XY', v: `${dims[0].toFixed(2)} × ${dims[1].toFixed(2)}` })
  }
  if (track?.velocity_xy && track.velocity_xy.length >= 2) {
    rows.push({
      k: 'VELOCITY XY',
      v: `${track.velocity_xy[0].toFixed(3)}, ${track.velocity_xy[1].toFixed(3)}`,
    })
  }
  if (src.frame) rows.push({ k: 'FRAME', v: src.frame })
  return { title: `TRACK #${src.track_id}`, rows }
}

function modeTitle(mode: ViewMode, frame: { points_frame?: string; world_points_frame?: string }) {
  if (mode === 'lidar') {
    return {
      kicker: 'CURRENT LIDAR FRAME',
      note: frame.points_frame ?? 'current_lidar',
    }
  }
  if (mode === 'world') {
    return {
      kicker: 'WORLD FRAME — lidar_frame_0',
      note: frame.world_points_frame ?? 'lidar_frame_0',
    }
  }
  if (mode === 'grid') {
    return {
      kicker: 'ADAPTIVE 2.5D TERRAIN MAP',
      note: 'Cells from this frame’s grid (current LiDAR XY). Geometric class GROUND / MIXED / OBSTACLE.',
    }
  }
  return {
    kicker: 'OBJECTS & TRACKING',
    note: 'Ground-plane footprints from exported XY and dimensions_xy. Not 3D bounding boxes.',
  }
}

export default function Demo() {
  const [params] = useSearchParams()
  const sceneId = params.get('scene')?.trim() || DEFAULT_SCENE
  const data = useOrbitData(sceneId)
  const [mode, setMode] = useState<ViewMode>('lidar')
  const [playing, setPlaying] = useState(false)
  const [showTrajectory, setShowTrajectory] = useState(true)
  const [selectedTrackId, setSelectedTrackId] = useState<number | null>(null)

  useEffect(() => {
    setSelectedTrackId(null)
  }, [data.frameIndex])

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
      if (el && (el.tagName === 'INPUT' || el.tagName === 'TEXTAREA' || el.isContentEditable)) {
        return
      }
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
  const titles = modeTitle(mode, data.frame ?? {})
  const selected = useMemo(() => {
    if (selectedTrackId === null || !data.frame) return null
    const track = data.frame.tracks?.find((t) => t.track_id === selectedTrackId)
    const world = data.frame.world_objects?.find((o) => o.track_id === selectedTrackId)
    return selectedDetails(track, world)
  }, [data.frame, selectedTrackId])

  const copyHint = `Copy exported_data/${sceneId}/ into web/public/data/${sceneId}/ (manifest.json, trajectory.json, frame_*.json).`

  return (
    <main className="mx-auto max-w-[1400px] px-4 pb-16 pt-24 md:px-8">
      <header className="flex flex-col gap-4 md:flex-row md:items-end md:justify-between">
        <div>
          <p className="font-mono text-[11px] tracking-[0.32em] text-orbit-cyan">ORBIT EXPLORER</p>
          <h1 className="mt-2 text-3xl font-medium tracking-tight text-orbit-text md:text-4xl">
            Interactive ORBIT Explorer
          </h1>
          <p className="mt-3 max-w-2xl text-sm leading-6 text-orbit-dim">
            Playback of exported PipelineState from{' '}
            <span className="text-orbit-text">{data.manifest?.scene_id ?? sceneId}</span>
            . The browser does not run perception, mapping, or tracking.
          </p>
        </div>
        <p className="font-mono text-[11px] tracking-[0.22em] text-orbit-dim">
          {(data.manifest?.scene_id ?? sceneId).toUpperCase()}
          {data.manifest?.source ? ` · ${data.manifest.source}` : ''}
        </p>
      </header>

      <div className="mt-8 grid gap-6 lg:grid-cols-[minmax(0,1fr)_minmax(16rem,19rem)] lg:items-stretch">
        <section className="relative h-[min(58vh,38rem)] min-h-[22rem] overflow-hidden rounded-[1.75rem] bg-[#0a1018] md:min-h-[28rem]">
          <div className="pointer-events-none absolute left-5 top-5 z-10 max-w-[22rem]">
            <p className="font-mono text-[10px] tracking-[0.28em] text-orbit-cyan">{titles.kicker}</p>
            <p className="mt-2 text-xs leading-5 text-orbit-dim">{titles.note}</p>
          </div>
          <p className="pointer-events-none absolute bottom-4 left-5 z-10 font-mono text-[10px] tracking-[0.18em] text-orbit-dim">
            DRAG — ROTATE · SCROLL — ZOOM · RIGHT DRAG — PAN
          </p>
          {data.bootstrapping || (data.frameLoading && !data.frame) ? (
            <LoadingState label={`LOADING FRAME ${pad(data.frameIndex)}`} />
          ) : null}
          {data.bootstrapError ? (
            <LoadingState
              label="UNABLE TO LOAD SCENE"
              error={`${data.bootstrapError} ${copyHint}`}
              onRetry={() => void data.retryBootstrap()}
            />
          ) : null}
          {data.frameError && !data.bootstrapError ? (
            <LoadingState
              label={`UNABLE TO LOAD FRAME ${pad(data.frameIndex)}`}
              error={data.frameError}
              onRetry={data.retryFrame}
            />
          ) : null}
          {data.frame && !data.bootstrapError ? (
            <div className={`absolute inset-0 ${data.frameLoading ? 'opacity-70' : ''}`}>
              <ExplorerScene
                frame={data.frame}
                mode={mode}
                trajectory={data.trajectory}
                showTrajectory={showTrajectory}
                selectedTrackId={selectedTrackId}
                onSelectTrack={setSelectedTrackId}
              />
            </div>
          ) : null}
          {data.frameLoading && data.frame ? (
            <p className="pointer-events-none absolute right-5 top-5 z-10 font-mono text-[10px] tracking-[0.2em] text-orbit-cyan">
              LOADING FRAME {pad(data.frameIndex)}
            </p>
          ) : null}
        </section>

        <div className="rounded-[1.75rem] bg-[#0a1018] px-6 py-6">
          {data.frame ? (
            <InformationPanel
              frameIndex={data.frame.frame_index}
              lastIndex={data.lastIndex}
              frameLabel={frameLabel}
              pointsFrame={data.frame.points_frame}
              worldFrame={data.frame.world_points_frame ?? data.manifest?.world_frame}
              poseSource={data.frame.pose.pose_source}
              egoXy={data.frame.pose.ego_xy}
              headingRad={data.frame.pose.heading_rad}
              pointCountFull={data.frame.metrics.point_count_full}
              pointCountExported={data.frame.metrics.point_count_exported}
              adaptiveCells={data.frame.metrics.adaptive_cells}
              obstacleCells={data.frame.metrics.obstacle_cells}
              liveTracks={data.frame.metrics.live_tracks}
              confirmedTracks={data.frame.metrics.confirmed_tracks}
              groundMethod={data.frame.ground?.method}
              groundInliers={data.frame.ground?.inlier_count}
              selected={selected}
            />
          ) : (
            <p className="text-sm leading-6 text-orbit-dim">
              Waiting for exported JSON. {copyHint}
            </p>
          )}
        </div>
      </div>

      <div className="mt-6 flex flex-col gap-4 md:flex-row md:items-center md:justify-between">
        <div>
          <p className="mb-3 font-mono text-[10px] tracking-[0.28em] text-orbit-dim">VIEW MODE</p>
          <ViewModeSelector mode={mode} onChange={setMode} />
        </div>
        <label className="flex items-center gap-3 text-sm text-orbit-dim">
          <input
            type="checkbox"
            checked={showTrajectory}
            onChange={(e) => setShowTrajectory(e.target.checked)}
            className="accent-orbit-cyan"
          />
          Show trajectory
        </label>
      </div>

      <div className="mt-6">
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

      <section className="mt-14">
        <p className="mb-3 font-mono text-[10px] tracking-[0.28em] text-orbit-dim">PIPELINE CONTEXT</p>
        <PipelineContext mode={mode} />
      </section>
    </main>
  )
}
