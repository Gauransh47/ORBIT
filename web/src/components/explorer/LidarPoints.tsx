import { useEffect, useMemo } from 'react'
import * as THREE from 'three'
import { orbitToThree } from '../../lib/orbitCoords'

export default function LidarPoints({
  points,
  color = '#9fdcff',
}: {
  points: number[][]
  color?: string
}) {
  const geometry = useMemo(() => {
    const geom = new THREE.BufferGeometry()
    const arr = new Float32Array(Math.max(points.length, 0) * 3)
    for (let i = 0; i < points.length; i++) {
      const p = points[i]
      if (!p || p.length < 3) continue
      const [x, y, z] = orbitToThree(p[0], p[1], p[2])
      arr[i * 3] = x
      arr[i * 3 + 1] = y
      arr[i * 3 + 2] = z
    }
    geom.setAttribute('position', new THREE.BufferAttribute(arr, 3))
    return geom
  }, [points])

  useEffect(() => {
    return () => {
      geometry.dispose()
    }
  }, [geometry])

  if (!points.length) return null

  return (
    <points geometry={geometry}>
      <pointsMaterial
        color={color}
        size={0.11}
        sizeAttenuation
        transparent
        opacity={0.88}
        depthWrite={false}
      />
    </points>
  )
}
