import { Line } from '@react-three/drei'
import { orbitToThree } from '../../lib/orbitCoords'
import type { TrajectorySample } from '../../types/orbit'

export default function Trajectory({
  samples,
  currentIndex,
  markerRadius = 0.35,
}: {
  samples: TrajectorySample[]
  currentIndex: number
  markerRadius?: number
}) {
  if (samples.length < 1) return null
  const points = samples.map((s) => {
    const [x, y, z] = orbitToThree(s.ego_xy[0], s.ego_xy[1], markerRadius * 0.15)
    return [x, y, z] as [number, number, number]
  })
  const current = samples.find((s) => s.frame_index === currentIndex) ?? samples[0]
  const [cx, cy, cz] = orbitToThree(current.ego_xy[0], current.ego_xy[1], markerRadius * 0.6)

  return (
    <group>
      {points.length >= 2 ? (
        <Line points={points} color="#3ddcff" lineWidth={1.6} transparent opacity={0.85} />
      ) : null}
      <mesh position={[cx, cy, cz]}>
        <sphereGeometry args={[markerRadius, 16, 16]} />
        <meshStandardMaterial color="#e08a3c" emissive="#e08a3c" emissiveIntensity={0.2} />
      </mesh>
    </group>
  )
}
