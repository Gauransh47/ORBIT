import { useLayoutEffect, useMemo, useRef } from 'react'
import * as THREE from 'three'
import { orbitToThree } from '../../lib/orbitCoords'
import {
  cellElevation,
  elevationRange,
  evenPickIndices,
  isRenderableCell,
  MAX_RENDER_CELLS,
  semanticColor,
  terrainColor,
} from '../../lib/cellVisual'
import type { AdaptiveCellRecord } from '../../types/orbit'

function colorFor(
  cell: AdaptiveCellRecord,
  range: { min: number; max: number } | null,
): THREE.Color {
  const c = new THREE.Color()
  if (cell.semantic_class === 'OBSTACLE' || (cell.obstacle_count ?? 0) > 0) {
    const [r, g, b] = semanticColor('OBSTACLE')
    c.setRGB(r, g, b)
    return c
  }
  if (cell.semantic_class === 'MIXED') {
    const [r, g, b] = semanticColor('MIXED')
    c.setRGB(r, g, b)
    return c
  }
  const [r, g, b] = terrainColor(cellElevation(cell), range)
  c.setRGB(r, g, b)
  return c
}

/** Interactive Demo grid: sampled, Lambert-lit, cool terrain + warm obstacles. */
export default function AdaptiveGridMesh({ cells }: { cells: AdaptiveCellRecord[] }) {
  const meshRef = useRef<THREE.InstancedMesh>(null)
  const geometry = useMemo(() => new THREE.BoxGeometry(1, 1, 1), [])
  const material = useMemo(
    () =>
      new THREE.MeshLambertMaterial({
        vertexColors: true,
      }),
    [],
  )
  const pick = useMemo(() => evenPickIndices(cells.length, MAX_RENDER_CELLS), [cells.length])
  const range = useMemo(() => elevationRange(cells.filter(isRenderableCell)), [cells])

  useLayoutEffect(() => {
    const mesh = meshRef.current
    if (!mesh) return
    const dummy = new THREE.Object3D()
    for (let i = 0; i < pick.length; i++) {
      const cell = cells[pick[i]]
      if (!cell || !isRenderableCell(cell)) continue
      const res = Math.max(cell.resolution, 0.03)
      const elevRaw = cellElevation(cell)
      const elev = elevRaw != null && Number.isFinite(elevRaw) ? elevRaw : 0
      const obstacleTop =
        cell.obstacle_elevation != null && Number.isFinite(cell.obstacle_elevation)
          ? cell.obstacle_elevation
          : null
      const thickness =
        obstacleTop != null
          ? Math.max(0.1, Math.abs(obstacleTop - elev))
          : Math.max(0.1, res * 0.32)
      const [x, y, z] = orbitToThree(cell.center[0], cell.center[1], elev)
      if (![x, y, z, res, thickness].every((v) => Number.isFinite(v))) continue
      dummy.position.set(x, y + thickness / 2, z)
      dummy.scale.set(res * 0.94, thickness, res * 0.94)
      dummy.updateMatrix()
      mesh.setMatrixAt(i, dummy.matrix)
      mesh.setColorAt(i, colorFor(cell, range))
    }
    mesh.instanceMatrix.needsUpdate = true
    if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true
  }, [cells, pick, range])

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
      args={[geometry, material, Math.max(pick.length, 1)]}
      frustumCulled={false}
    />
  )
}
