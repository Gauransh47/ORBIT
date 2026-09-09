import { useMemo } from 'react'
import { Canvas } from '@react-three/fiber'
import { Line, OrbitControls } from '@react-three/drei'
import MapCameraRig, { type MapCameraView } from '../map/MapCameraRig'
import MapGrid, { type MapVizMode } from '../map/MapGrid'
import MapObstacleCells from '../map/MapObstacleCells'
import { EgoMarker } from '../explorer/ObjectTracks'
import { gridFocus } from '../../lib/cellVisual'
import { orbitToThree } from '../../lib/orbitCoords'
import type { FrameJson } from '../../types/orbit'

function threeToOrbit(point: { x: number; y: number; z: number }): [number, number] {
  return [point.x, -point.z]
}

export default function PlanningScene({
  frame,
  mode,
  cameraView,
  path,
  start,
  goal,
  selectedIndex,
  onPickCell,
  onPickXy,
}: {
  frame: FrameJson
  mode: MapVizMode
  cameraView: MapCameraView
  path: [number, number][]
  start: [number, number] | null
  goal: [number, number] | null
  selectedIndex: number | null
  onPickCell: (index: number) => void
  onPickXy: (xy: [number, number]) => void
}) {
  const cells = frame.adaptive_cells ?? []
  const focus = useMemo(() => gridFocus(cells), [cells])
  const markerRadius = Math.min(1.4, Math.max(0.08, focus.radius * 0.04))
  const pathPts = useMemo(() => {
    if (path.length < 2) return null
    return path.map(([x, y]) => orbitToThree(x, y, 0.35))
  }, [path])

  return (
    <Canvas
      className="h-full w-full"
      camera={{ position: [42, 58, 42], fov: 46, near: 0.1, far: 800 }}
      dpr={[1, 1.5]}
      gl={{ antialias: true, alpha: false }}
      onCreated={({ gl }) => {
        gl.setClearColor('#07090e')
      }}
    >
      <ambientLight intensity={0.55} />
      <directionalLight position={[30, 50, 12]} intensity={1.05} />
      <OrbitControls makeDefault enableDamping dampingFactor={0.08} />
      <MapCameraRig view={cameraView} cells={cells} />

      <mesh
        rotation={[-Math.PI / 2, 0, 0]}
        position={[focus.target[0], 0, focus.target[2]]}
        onClick={(e) => {
          e.stopPropagation()
          onPickXy(threeToOrbit(e.point))
        }}
      >
        <planeGeometry args={[Math.max(80, focus.radius * 8), Math.max(80, focus.radius * 8)]} />
        <meshBasicMaterial transparent opacity={0} />
      </mesh>

      <MapGrid
        key={`${frame.frame_index}-${cells.length}`}
        cells={cells}
        mode={mode}
        selectedIndex={selectedIndex}
        onSelectIndex={(index) => {
          if (index == null) return
          onPickCell(index)
        }}
      />
      <MapObstacleCells cells={frame.obstacle_cells ?? []} />

      {pathPts ? <Line points={pathPts} color="#f4f1de" lineWidth={2.4} /> : null}

      {start ? (
        <EgoMarker xy={start} radius={markerRadius} />
      ) : (
        <EgoMarker xy={frame.pose.ego_xy} radius={markerRadius} />
      )}
      {goal ? (
        <mesh position={orbitToThree(goal[0], goal[1], markerRadius)}>
          <sphereGeometry args={[markerRadius * 0.85, 16, 16]} />
          <meshStandardMaterial color="#e08a3c" emissive="#e08a3c" emissiveIntensity={0.25} />
        </mesh>
      ) : null}
    </Canvas>
  )
}
