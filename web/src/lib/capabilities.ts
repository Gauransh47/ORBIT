import type { DatasetCapabilities } from '../types/datasets'
import type { FrameJson, ViewMode } from '../types/orbit'

export function inferCapabilities(
  frame: FrameJson | null,
  hasTrajectoryFile: boolean,
  declared?: DatasetCapabilities,
): DatasetCapabilities {
  const inferred: DatasetCapabilities = {
    has_points: Array.isArray(frame?.points),
    has_world_points: Array.isArray(frame?.world_points),
    has_trajectory: hasTrajectoryFile,
    has_adaptive_grid: Array.isArray(frame?.adaptive_cells),
    has_tracks: Array.isArray(frame?.tracks),
    has_objects: Array.isArray(frame?.world_objects),
    has_proposals: Array.isArray(frame?.proposals),
    has_semantic_labels: Array.isArray((frame as { semantic_labels?: unknown } | null)?.semantic_labels),
    has_ground_data: Boolean(frame?.ground),
    has_path_data: Array.isArray((frame as { path?: unknown } | null)?.path),
  }
  return { ...declared, ...inferred }
}

export function modeAvailable(mode: ViewMode, caps: DatasetCapabilities): boolean {
  if (mode === 'lidar') return caps.has_points !== false
  if (mode === 'world') return caps.has_world_points === true
  if (mode === 'grid') return caps.has_adaptive_grid === true
  if (mode === 'objects') {
    return caps.has_tracks === true || caps.has_objects === true || caps.has_proposals === true
  }
  return false
}

export function firstAvailableMode(caps: DatasetCapabilities): ViewMode {
  const order: ViewMode[] = ['lidar', 'world', 'grid', 'objects']
  return order.find((m) => modeAvailable(m, caps)) ?? 'lidar'
}

export function modeUnavailableLabel(mode: ViewMode): string {
  if (mode === 'objects') return 'Object tracking is not available for this exported dataset.'
  if (mode === 'grid') return 'Adaptive grid is not available for this exported dataset.'
  if (mode === 'world') return 'World-frame points are not available for this exported dataset.'
  return 'This visualization is not available for this exported dataset.'
}
