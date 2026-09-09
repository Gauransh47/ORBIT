import { useLayoutEffect, useMemo, useRef } from 'react'
import * as THREE from 'three'
import { orbitToThree } from '../../lib/orbitCoords'
import type { AdaptiveCellRecord } from '../../types/orbit'

function classColor(name: string | undefined): THREE.Color {
  if (name === 'GROUND') return new THREE.Color('#2fbf9a')
  if (name === 'MIXED') return new THREE.Color('#3ddcff')
  if (name === 'OBSTACLE') return new THREE.Color('#e08a3c')
  return new THREE.Color('#5b6b7c')
}

export default function AdaptiveGridMesh({ cells }: { cells: AdaptiveCellRecord[] }) {
  const meshRef = useRef<THREE.InstancedMesh>(null)
  const geometry = useMemo(() => new THREE.BoxGeometry(1, 1, 1), [])
  const material = useMemo(
    () =>
      new THREE.MeshStandardMaterial({
        vertexColors: true,
        transparent: true,
        opacity: 0.82,
        roughness: 0.55,
        metalness: 0.08,
      }),
    [],
  )

  useLayoutEffect(() => {
    const mesh = meshRef.current
    if (!mesh) return
    const dummy = new THREE.Object3D()
    const color = new THREE.Color()
    for (let i = 0; i < cells.length; i++) {
      const cell = cells[i]
      const res = Math.max(cell.resolution ?? 0.05, 0.03)
      const elev =
        cell.ground_elevation ??
        cell.z_mean ??
        0
      const top =
        cell.obstacle_elevation ??
        cell.z_max ??
        elev
      const h = Math.max(0.04, Math.abs(top - elev) * 0.35 + res * 0.18)
      const [x, y, z] = orbitToThree(cell.center[0], cell.center[1], elev)
      dummy.position.set(x, y + h / 2, z)
      dummy.scale.set(res * 0.92, h, res * 0.92)
      dummy.updateMatrix()
      mesh.setMatrixAt(i, dummy.matrix)
      color.copy(classColor(cell.semantic_class))
      mesh.setColorAt(i, color)
    }
    mesh.instanceMatrix.needsUpdate = true
    if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true
  }, [cells])

  useLayoutEffect(() => {
    return () => {
      geometry.dispose()
      material.dispose()
    }
  }, [geometry, material])

  if (!cells.length) return null

  return (
    <instancedMesh
      ref={meshRef}
      args={[geometry, material, cells.length]}
      frustumCulled={false}
    />
  )
}
