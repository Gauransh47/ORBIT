import { useLayoutEffect, useMemo, useRef } from 'react'
import * as THREE from 'three'
import { orbitToThree } from '../../lib/orbitCoords'
import type { ObstacleCellRecord } from '../../types/orbit'

export default function MapObstacleCells({ cells }: { cells: ObstacleCellRecord[] }) {
  const meshRef = useRef<THREE.InstancedMesh>(null)
  const geometry = useMemo(() => new THREE.BoxGeometry(1, 1, 1), [])
  const material = useMemo(
    () =>
      new THREE.MeshStandardMaterial({
        color: '#e08a3c',
        transparent: true,
        opacity: 0.55,
        roughness: 0.45,
        metalness: 0.08,
        depthWrite: false,
      }),
    [],
  )

  useLayoutEffect(() => {
    const mesh = meshRef.current
    if (!mesh) return
    const dummy = new THREE.Object3D()
    for (let i = 0; i < cells.length; i++) {
      const cell = cells[i]
      const res = Math.max(cell.resolution ?? 0.05, 0.03)
      const ground =
        cell.ground_elevation != null && Number.isFinite(cell.ground_elevation)
          ? cell.ground_elevation
          : 0
      let top = ground
      if (cell.obstacle_elevation != null && Number.isFinite(cell.obstacle_elevation)) {
        top = cell.obstacle_elevation
      } else if (cell.obstacle_height != null && Number.isFinite(cell.obstacle_height)) {
        top = ground + cell.obstacle_height
      }
      const thickness = Math.max(0.06, Math.abs(top - ground) || res * 0.22)
      const [x, y, z] = orbitToThree(cell.center[0], cell.center[1], ground)
      dummy.position.set(x, y + thickness / 2, z)
      dummy.scale.set(res * 0.98, thickness, res * 0.98)
      dummy.updateMatrix()
      mesh.setMatrixAt(i, dummy.matrix)
    }
    mesh.instanceMatrix.needsUpdate = true
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
      raycast={() => undefined}
    />
  )
}
