import LidarPoints from './LidarPoints'
import { evenPickIndices } from '../../lib/cellVisual'
import { useMemo } from 'react'

export default function WorldPoints({
  points,
  maxPoints = 8000,
}: {
  points: number[][]
  maxPoints?: number
}) {
  const sampled = useMemo(() => {
    const idx = evenPickIndices(points.length, maxPoints)
    return idx.map((i) => points[i]).filter(Boolean)
  }, [points, maxPoints])
  return <LidarPoints points={sampled} color="#7dd3c7" />
}
