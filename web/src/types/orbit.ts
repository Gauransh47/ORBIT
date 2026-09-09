export type ViewMode = 'lidar' | 'world' | 'grid' | 'objects'

export type TrajectorySample = {
  frame_index: number
  ego_xy: [number, number]
  heading_rad: number
  pose_source: string
  file: string
}

export type TrajectoryFile = {
  world_frame: string
  samples: TrajectorySample[]
}

export type Manifest = {
  schema_version: number
  scene_id: string
  source: string
  frame_count: number
  frame_indices: number[]
  world_frame: string
  world_reference_frame_index: number
  coordinate_system?: {
    world?: string
    note?: string
    lidar_forward?: string
  }
  downsample?: {
    method?: string
    max_points?: number
    description?: string
  }
  pose_source?: string
  files: string[]
  website_note?: string
}

export type Pose = {
  ego_xy: [number, number]
  heading_rad: number
  pose_source: string
  T_ego_to_world?: number[][]
  world_frame?: string
  world_reference_frame_index?: number
}

export type Metrics = {
  latency_ms?: number
  input_points?: number
  mapped_points?: number
  ground_points?: number
  adaptive_cells?: number
  obstacle_cells?: number
  proposals?: number
  live_tracks?: number
  confirmed_tracks?: number
  world_objects?: number
  world_frame?: string
  pose_source?: string
  point_count_full?: number
  point_count_exported?: number
  adaptive_cells_exported?: number
  world_points_exported?: number
  [key: string]: unknown
}

export type GroundInfo = {
  method?: string
  inlier_count?: number
  plane_model?: number[] | null
}

export type TrackRecord = {
  track_id: number
  class_name: string
  position: [number, number]
  frame?: string
  dimensions_xy?: number[]
  velocity_xy?: number[]
  hits?: number
  missed?: number
  age?: number
  confirmed?: boolean
  first_frame?: number
  last_frame?: number
  motion_state?: string
  confidence?: number
  motion_history_speed?: number[]
}

export type AdaptiveCellRecord = {
  ix?: number
  iy?: number
  level?: number
  center: [number, number]
  resolution: number
  semantic_class?: string
  ground_elevation?: number | null
  obstacle_elevation?: number | null
  z_mean?: number | null
  point_count?: number
  ground_count?: number
  obstacle_count?: number
  z_min?: number
  z_max?: number
}

export type ObstacleCellRecord = {
  ix?: number
  iy?: number
  level?: number
  center: [number, number]
  resolution: number
  ground_elevation?: number | null
  obstacle_elevation?: number | null
  obstacle_height?: number | null
  obstacle_count?: number
  ground_source?: string
}

export type ProposalRecord = {
  proposal_id?: number
  center?: [number, number]
  width?: number
  length?: number
  height?: number
  classification?: string
  confidence?: number
  frame?: string
  [key: string]: unknown
}

export type WorldObjectRecord = {
  track_id: number
  class_name: string
  position: [number, number]
  frame?: string
  hits?: number
  confirmed?: boolean
  motion_state?: string
  confidence?: number
  history_xy?: [number, number][]
  dimensions_xy?: number[]
  age?: number
  missed?: number
}

export type FrameJson = {
  schema_version?: number
  frame_index: number
  points_frame?: string
  world_points_frame?: string
  pose: Pose
  metrics: Metrics
  ground?: GroundInfo
  points: number[][]
  world_points: number[][]
  tracks: TrackRecord[]
  adaptive_cells: AdaptiveCellRecord[]
  obstacle_cells?: ObstacleCellRecord[]
  proposals: ProposalRecord[]
  world_objects: WorldObjectRecord[]
  retired_ids?: number[]
}

export const DATA_PREFIX = '/data'
export const FRAME_CACHE_LIMIT = 12
export const PLAYBACK_MS = 450
