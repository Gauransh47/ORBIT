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
  playbackXy,
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
  playbackXy: [number, number] | null
  selectedIndex: number | null
  onPickCell: (index: number) => void
  onPickXy: (xy: [number, number]) => void
}) {
  const cells = frame.adaptive_cells ?? []
  const focus = useMemo(() => gridFocus(cells), [cells])
  const markerRadius = Math.min(1.4, Math.max(0.12, focus.radius * 0.035))
  const pathPts = useMemo(() => {
    if (path.length < 2) return null
    return path.map(([x, y]) => orbitToThree(x, y, 0.42))
  }, [path])
  const ticks = useMemo(() => {
    if (path.length < 2) return []
    const out: [number, number, number][][] = []
    const step = Math.max(1, Math.floor(path.length / 12))
    for (let i = step; i < path.length; i += step) {
      const [x0, y0] = path[i - 1]
      const [x1, y1] = path[i]
      const dx = x1 - x0
      const dy = y1 - y0
      const len = Math.hypot(dx, dy) || 1
      const px = (-dy / len) * markerRadius * 0.6
      const py = (dx / len) * markerRadius * 0.6
      out.push([
        orbitToThree(x1 - dx * 0.4 + px, y1 - dy * 0.4 + py, 0.5),
        orbitToThree(x1, y1, 0.5),
        orbitToThree(x1 - dx * 0.4 - px, y1 - dy * 0.4 - py, 0.5),
      ])
    }
    return out
  }, [path, markerRadius])

  return (
    <Canvas
      className="h-full w-full"
      camera={{ position: [42, 58, 42], fov: 46, near: 0.1, far: 800 }}
      dpr={[1, 1.5]}
      gl={{ antialias: true, alpha: false }}
      onCreated={({ gl }) => {
        gl.setClearColor('#0b1522')
      }}
    >
      <ambientLight intensity={0.95} />
      <directionalLight position={[30, 50, 12]} intensity={1.35} />
      <hemisphereLight args={['#c8e8ff', '#163028', 0.5]} />
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

      {pathPts ? (
        <>
          <Line points={pathPts} color="#0b1522" lineWidth={8} />
          <Line points={pathPts} color="#ffe566" lineWidth={3.5} />
        </>
      ) : null}
      {ticks.map((pts, i) => (
        <Line key={i} points={pts} color="#fff4b0" lineWidth={2} />
      ))}

      {start ? <EgoMarker xy={start} radius={markerRadius} /> : null}
      {goal ? (
        <mesh position={orbitToThree(goal[0], goal[1], markerRadius)}>
          <sphereGeometry args={[markerRadius * 0.9, 16, 16]} />
          <meshStandardMaterial color="#ff6a2c" emissive="#ff6a2c" emissiveIntensity={0.4} />
        </mesh>
      ) : null}
      {playbackXy ? (
        <mesh position={orbitToThree(playbackXy[0], playbackXy[1], markerRadius * 1.2)}>
          <sphereGeometry args={[markerRadius * 0.7, 16, 16]} />
          <meshStandardMaterial color="#f4f1de" emissive="#ffe566" emissiveIntensity={0.5} />
        </mesh>
      ) : null}
    </Canvas>
  )
}
