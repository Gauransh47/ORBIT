import type { AdaptiveCellRecord, FrameJson, ObstacleCellRecord, Pose } from '../types/orbit'
import { isRenderableCell } from './cellVisual'

function asArray<T>(value: unknown): T[] {
  return Array.isArray(value) ? (value as T[]) : []
}

function sanitizePose(pose: unknown): Pose | null {
  if (!pose || typeof pose !== 'object') return null
  const p = pose as Pose
  if (!Array.isArray(p.ego_xy) || p.ego_xy.length < 2) return null
  if (!Number.isFinite(Number(p.ego_xy[0])) || !Number.isFinite(Number(p.ego_xy[1]))) return null
  return {
    ...p,
    ego_xy: [Number(p.ego_xy[0]), Number(p.ego_xy[1])],
    heading_rad: Number.isFinite(p.heading_rad) ? p.heading_rad : 0,
    pose_source: p.pose_source || 'unknown',
  }
}

export function normalizeFrameJson(raw: FrameJson): FrameJson {
  const pose = sanitizePose(raw.pose)
  if (!pose) {
    throw new Error('Frame JSON is missing a valid pose.ego_xy. The map will not invent a pose.')
  }
  return {
    ...raw,
    frame_index: Number.isFinite(raw.frame_index) ? raw.frame_index : 0,
    pose,
    metrics: raw.metrics && typeof raw.metrics === 'object' ? raw.metrics : {},
    points: asArray(raw.points),
    world_points: asArray(raw.world_points),
    tracks: asArray(raw.tracks),
    adaptive_cells: asArray<AdaptiveCellRecord>(raw.adaptive_cells).filter(isRenderableCell),
    obstacle_cells: asArray<ObstacleCellRecord>(raw.obstacle_cells).filter((cell) => {
      return (
        Array.isArray(cell.center) &&
        cell.center.length >= 2 &&
        Number.isFinite(cell.center[0]) &&
        Number.isFinite(cell.center[1]) &&
        Number.isFinite(cell.resolution) &&
        cell.resolution > 0
      )
    }),
    proposals: asArray(raw.proposals),
    world_objects: asArray(raw.world_objects),
    retired_ids: asArray(raw.retired_ids),
  }
}
