import { useEffect, useMemo } from 'react'
import * as THREE from 'three'
import { orbitToThree } from '../../lib/orbitCoords'
import { isFiniteNumber, pointsFocus } from '../../lib/cellVisual'

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
      if (![p[0], p[1], p[2]].every(isFiniteNumber)) continue
      const [x, y, z] = orbitToThree(p[0], p[1], p[2])
      arr[i * 3] = x
      arr[i * 3 + 1] = y
      arr[i * 3 + 2] = z
    }
    geom.setAttribute('position', new THREE.BufferAttribute(arr, 3))
    return geom
  }, [points])

  const { size, sizeAttenuation } = useMemo(() => {
    const sparse = points.length > 0 && points.length <= 64
    if (sparse) return { size: 10, sizeAttenuation: false }
    const radius = pointsFocus(points).radius
    return { size: Math.min(0.28, Math.max(0.08, radius * 0.004)), sizeAttenuation: true }
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
        size={size}
        sizeAttenuation={sizeAttenuation}
        transparent
        opacity={0.92}
        depthWrite={false}
      />
    </points>
  )
}
