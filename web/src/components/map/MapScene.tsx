import { useMemo } from 'react'
import { Canvas } from '@react-three/fiber'
import { OrbitControls } from '@react-three/drei'
import { gridFocus } from '../../lib/cellVisual'
import type { FrameJson, TrajectoryFile } from '../../types/orbit'
import FoveationRings from './FoveationRings'
import MapCameraRig, { type MapCameraView } from './MapCameraRig'
import MapGrid, { type MapVizMode } from './MapGrid'
import MapObstacleCells from './MapObstacleCells'
import Trajectory from '../explorer/Trajectory'
import ObjectTracks, { EgoMarker } from '../explorer/ObjectTracks'
import WorldPoints from '../explorer/WorldPoints'

export default function MapScene({
  frame,
  mode,
  cameraView,
  selectedIndex,
  onSelectIndex,
  trajectory,
  showTrajectory,
  showObjects,
  showWorldPoints,
  showObstacleCells,
  showFoveation,
  showObjectIds,
}: {
  frame: FrameJson
  mode: MapVizMode
  cameraView: MapCameraView
  selectedIndex: number | null
  onSelectIndex: (index: number | null) => void
  trajectory: TrajectoryFile | null
  showTrajectory: boolean
  showObjects: boolean
  showWorldPoints: boolean
  showObstacleCells: boolean
  showFoveation: boolean
  showObjectIds?: boolean
}) {
  const cells = frame.adaptive_cells ?? []
  const focus = useMemo(() => gridFocus(cells), [cells])
  const markerRadius = Math.min(1.2, Math.max(0.04, focus.radius * 0.06))

  return (
    <Canvas
      className="h-full w-full"
      camera={{ position: [42, 38, 42], fov: 46, near: 0.1, far: 800 }}
      dpr={[1, 1.5]}
      gl={{ antialias: true, alpha: false }}
      onCreated={({ gl }) => {
        gl.setClearColor('#0b1522')
      }}
      onPointerMissed={() => onSelectIndex(null)}
    >
      <ambientLight intensity={0.95} />
      <directionalLight position={[30, 50, 12]} intensity={1.35} />
      <hemisphereLight args={['#c8e8ff', '#163028', 0.55]} />
      <OrbitControls makeDefault enableDamping dampingFactor={0.08} />
      <MapCameraRig view={cameraView} cells={frame.adaptive_cells ?? []} />

      <MapGrid
        key={`${frame.frame_index}-${(frame.adaptive_cells ?? []).length}`}
        cells={frame.adaptive_cells ?? []}
        mode={mode}
        selectedIndex={selectedIndex}
        onSelectIndex={onSelectIndex}
      />

      {showObstacleCells ? <MapObstacleCells cells={frame.obstacle_cells ?? []} /> : null}
      {showWorldPoints ? <WorldPoints points={frame.world_points ?? []} /> : null}
      {showTrajectory && trajectory ? (
        <Trajectory samples={trajectory.samples} currentIndex={frame.frame_index} markerRadius={markerRadius} />
      ) : null}
      {showObjects ? (
        <ObjectTracks
          tracks={frame.tracks ?? []}
          worldObjects={frame.world_objects ?? []}
          selectedId={null}
          onSelect={() => undefined}
          showIds={Boolean(showObjectIds)}
        />
      ) : null}
      {showFoveation && frame.pose?.ego_xy ? (
        <FoveationRings cells={cells} egoXy={frame.pose.ego_xy} />
      ) : null}
      <EgoMarker xy={frame.pose.ego_xy} radius={markerRadius} />
    </Canvas>
  )
}
