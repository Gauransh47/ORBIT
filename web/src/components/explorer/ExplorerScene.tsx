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
}: {
  frame: FrameJson
  mode: ViewMode
  trajectory: TrajectoryFile | null
  showTrajectory: boolean
  selectedTrackId: number | null
  onSelectTrack: (id: number | null) => void
}) {
  return (
    <Canvas
      className="h-full w-full"
      camera={{ position: [28, 22, 28], fov: 50, near: 0.1, far: 400 }}
      dpr={[1, 1.5]}
      gl={{ antialias: true, alpha: false }}
      onCreated={({ gl }) => {
        gl.setClearColor('#07090e')
      }}
    >
      <ambientLight intensity={0.55} />
      <directionalLight position={[20, 40, 10]} intensity={0.85} />
      <gridHelper args={[120, 48, '#243044', '#151b24']} />
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
          />
          <EgoMarker xy={frame.pose.ego_xy} />
        </>
      ) : null}
    </Canvas>
  )
}
