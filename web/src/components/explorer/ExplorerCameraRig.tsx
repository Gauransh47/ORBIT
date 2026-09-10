import { useEffect, useMemo, useRef } from 'react'
import { useThree } from '@react-three/fiber'
import type { PerspectiveCamera } from 'three'
import { gridFocus, pointsFocus } from '../../lib/cellVisual'
import type { AdaptiveCellRecord } from '../../types/orbit'

type OrbitLike = {
  target: { set: (x: number, y: number, z: number) => void }
  minDistance: number
  maxDistance: number
  update: () => void
}

/** Fit the demo camera once when the viewer mounts. Frame playback must not move it. */
export default function ExplorerCameraRig({
  points,
  cells,
}: {
  points: number[][]
  cells: AdaptiveCellRecord[]
}) {
  const camera = useThree((s) => s.camera)
  const controls = useThree((s) => s.controls) as OrbitLike | null
  const hasFitted = useRef(false)
  const focus = useMemo(() => {
    if (points.length) return pointsFocus(points)
    if (cells.length) return gridFocus(cells)
    return { target: [0, 0, 0] as [number, number, number], radius: 40 }
  }, [points, cells])

  useEffect(() => {
    if (hasFitted.current) return
    if (!controls) return
    const [tx, ty, tz] = focus.target
    const r = Math.max(focus.radius, 0.5)
    const cam = camera as PerspectiveCamera
    if (![tx, ty, tz, r].every((v) => Number.isFinite(v))) return
    cam.position.set(tx + r * 1.8, ty + r * 1.4, tz + r * 1.8)
    cam.near = Math.max(0.02, r / 200)
    cam.far = Math.max(400, r * 40)
    cam.updateProjectionMatrix()
    cam.lookAt(tx, ty, tz)
    controls.target.set(tx, ty, tz)
    controls.minDistance = Math.max(0.08, r * 0.12)
    controls.maxDistance = Math.max(80, r * 18)
    controls.update()
    hasFitted.current = true
  }, [camera, controls, focus])

  return null
}
