import { useLayoutEffect, useMemo, useRef } from 'react'
import * as THREE from 'three'
import { orbitToThree } from '../../lib/orbitCoords'
import {
  cellElevation,
  elevationRange,
  resolutionColor,
  semanticColor,
  terrainColor,
} from '../../lib/cellVisual'
import type { AdaptiveCellRecord } from '../../types/orbit'

export type MapVizMode = 'terrain' | 'semantic' | 'resolution' | 'obstacles'

function colorFor(
  cell: AdaptiveCellRecord,
  mode: MapVizMode,
  range: { min: number; max: number } | null,
  selected: boolean,
): THREE.Color {
  const c = new THREE.Color()
  if (selected) {
    c.set('#f4f1de')
    return c
  }
  if (mode === 'semantic') {
    const [r, g, b] = semanticColor(cell.semantic_class)
    c.setRGB(r, g, b)
    return c
  }
  if (mode === 'resolution') {
    const [r, g, b] = resolutionColor(cell.resolution)
    c.setRGB(r, g, b)
    return c
  }
  if (mode === 'obstacles') {
    if (cell.semantic_class === 'OBSTACLE') c.set('#e08a3c')
    else if (cell.semantic_class === 'MIXED') c.set('#c4a574')
    else c.set('#1a3030')
    return c
  }
  const [r, g, b] = terrainColor(cellElevation(cell), range)
  c.setRGB(r, g, b)
  return c
}

export default function MapGrid({
  cells,
  mode,
  selectedIndex,
  onSelectIndex,
}: {
  cells: AdaptiveCellRecord[]
  mode: MapVizMode
  selectedIndex: number | null
  onSelectIndex: (index: number | null) => void
}) {
  const meshRef = useRef<THREE.InstancedMesh>(null)
  const geometry = useMemo(() => new THREE.BoxGeometry(1, 1, 1), [])
  const material = useMemo(
    () =>
      new THREE.MeshStandardMaterial({
        vertexColors: true,
        roughness: 0.62,
        metalness: 0.04,
      }),
    [],
  )
  const range = useMemo(() => elevationRange(cells), [cells])

  useLayoutEffect(() => {
    const mesh = meshRef.current
    if (!mesh) return
    const dummy = new THREE.Object3D()
    for (let i = 0; i < cells.length; i++) {
      const cell = cells[i]
      const res = Math.max(cell.resolution ?? 0.05, 0.03)
      const elev = cellElevation(cell) ?? 0
      const obstacleTop =
        cell.obstacle_elevation != null && Number.isFinite(cell.obstacle_elevation)
          ? cell.obstacle_elevation
          : null
      const thickness =
        obstacleTop != null
          ? Math.max(0.05, Math.abs(obstacleTop - elev))
          : Math.max(0.05, res * 0.22)
      const [x, y, z] = orbitToThree(cell.center[0], cell.center[1], elev)
      dummy.position.set(x, y + thickness / 2, z)
      dummy.scale.set(res * 0.94, thickness, res * 0.94)
      dummy.updateMatrix()
      mesh.setMatrixAt(i, dummy.matrix)
      mesh.setColorAt(i, colorFor(cell, mode, range, i === selectedIndex))
    }
    mesh.instanceMatrix.needsUpdate = true
    if (mesh.instanceColor) mesh.instanceColor.needsUpdate = true
  }, [cells, mode, range, selectedIndex])

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
      onClick={(e) => {
        e.stopPropagation()
        const id = e.instanceId
        if (id == null) return
        onSelectIndex(id === selectedIndex ? null : id)
      }}
    />
  )
}
