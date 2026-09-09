import { Line } from '@react-three/drei'
import { orbitToThree } from '../../lib/orbitCoords'
import type { AdaptiveCellRecord } from '../../types/orbit'
import { foveationFromExport } from '../../lib/prototypeMetrics'

const RING_COLORS = ['#7af0ff', '#5ee0b8', '#7aa6ff', '#c4b5fd']

export default function FoveationRings({
  cells,
  egoXy,
}: {
  cells: AdaptiveCellRecord[]
  egoXy: [number, number]
}) {
  const levels = foveationFromExport(cells, egoXy)
  if (!levels.length) return null
  const [ex, ey, ez] = orbitToThree(egoXy[0], egoXy[1], 0.08)

  return (
    <group>
      <mesh position={[ex, ey, ez]}>
        <sphereGeometry args={[0.18, 12, 12]} />
        <meshBasicMaterial color="#f4f1de" />
      </mesh>
      {levels.map((level, i) => {
        const radius = Math.max(level.maxRange, 0.2)
        const segs = 64
        const pts: [number, number, number][] = []
        for (let s = 0; s <= segs; s++) {
          const a = (s / segs) * Math.PI * 2
          const x = egoXy[0] + Math.cos(a) * radius
          const y = egoXy[1] + Math.sin(a) * radius
          pts.push(orbitToThree(x, y, 0.12))
        }
        return (
          <Line
            key={level.resolution}
            points={pts}
            color={RING_COLORS[i % RING_COLORS.length]}
            lineWidth={1.2}
            transparent
            opacity={0.55}
          />
        )
      })}
    </group>
  )
}
