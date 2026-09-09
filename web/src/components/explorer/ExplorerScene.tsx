import { Canvas } from '@react-three/fiber'
import { OrbitControls } from '@react-three/drei'
import type { FrameJson, TrajectoryFile, ViewMode } from '../../types/orbit'
import LidarPoints from './LidarPoints'
import WorldPoints from './WorldPoints'
import AdaptiveGridMesh from './AdaptiveGrid'
import Trajectory from './Trajectory'
import ObjectTracks, { EgoMarker, ProposalFootprints } from './ObjectTracks'

export default function ExplorerScene({
  frame,
  mode,
  trajectory,
  showTrajectory,
  selectedTrackId,
  onSelectTrack,
  showObjectIds,
}: {
  frame: FrameJson
  mode: ViewMode
  trajectory: TrajectoryFile | null
  showTrajectory: boolean
  selectedTrackId: number | null
  onSelectTrack: (id: number | null) => void
  showObjectIds?: boolean
}) {
  return (
    <Canvas
      className="h-full w-full"
      camera={{ position: [28, 22, 28], fov: 50, near: 0.1, far: 400 }}
      dpr={[1, 1.5]}
      gl={{ antialias: true, alpha: false }}
      onCreated={({ gl }) => {
        gl.setClearColor('#0b1522')
      }}
    >
      <ambientLight intensity={0.95} />
      <directionalLight position={[30, 50, 12]} intensity={1.25} />
      <hemisphereLight args={['#c8e8ff', '#163028', 0.5]} />
      <gridHelper args={[120, 48, '#1c3348', '#0f1a24']} />
      <OrbitControls makeDefault enableDamping dampingFactor={0.08} maxDistance={180} minDistance={4} />

      {mode === 'lidar' ? (
        <>
          <LidarPoints points={frame.points ?? []} />
          <ProposalFootprints proposals={frame.proposals ?? []} />
        </>
      ) : null}

      {mode === 'world' ? (
        <>
          <WorldPoints points={frame.world_points ?? []} />
          <EgoMarker xy={frame.pose.ego_xy} />
          {showTrajectory && trajectory ? (
            <Trajectory samples={trajectory.samples} currentIndex={frame.frame_index} />
          ) : null}
        </>
      ) : null}

      {mode === 'grid' ? (
        <AdaptiveGridMesh key={frame.frame_index} cells={frame.adaptive_cells ?? []} />
      ) : null}

      {mode === 'objects' ? (
        <>
          {showTrajectory && trajectory ? (
            <Trajectory samples={trajectory.samples} currentIndex={frame.frame_index} />
          ) : null}
          <ObjectTracks
            tracks={frame.tracks ?? []}
            worldObjects={frame.world_objects ?? []}
            selectedId={selectedTrackId}
            onSelect={onSelectTrack}
            showIds={Boolean(showObjectIds)}
          />
          <EgoMarker xy={frame.pose.ego_xy} />
        </>
      ) : null}
    </Canvas>
  )
}
