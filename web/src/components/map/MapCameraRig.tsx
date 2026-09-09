import { useEffect, useMemo } from 'react'
import { useThree } from '@react-three/fiber'
import type { PerspectiveCamera } from 'three'
import { gridFocus } from '../../lib/cellVisual'
import type { AdaptiveCellRecord } from '../../types/orbit'

export type MapCameraView = 'iso' | 'top' | 'side'

type OrbitLike = {
  target: { set: (x: number, y: number, z: number) => void }
  minDistance: number
  maxDistance: number
  update: () => void
}

export default function MapCameraRig({
  view,
  cells,
}: {
  view: MapCameraView
  cells: AdaptiveCellRecord[]
}) {
  const camera = useThree((s) => s.camera)
  const controls = useThree((s) => s.controls) as OrbitLike | null
  const focus = useMemo(() => gridFocus(cells), [cells])

  useEffect(() => {
    const [tx, ty, tz] = focus.target
    const r = Math.max(focus.radius, 0.6)
    const cam = camera as PerspectiveCamera
    if (![tx, ty, tz, r].every((v) => Number.isFinite(v))) return
    if (view === 'top') cam.position.set(tx + 0.01, ty + r * 2.6, tz + 0.01)
    else if (view === 'side') cam.position.set(tx + r * 2.4, ty + r * 0.4, tz)
    else cam.position.set(tx + r * 1.7, ty + r * 1.35, tz + r * 1.7)
    if (![cam.position.x, cam.position.y, cam.position.z].every((v) => Number.isFinite(v))) return
    cam.near = Math.max(0.02, r / 200)
    cam.far = Math.max(400, r * 40)
    cam.updateProjectionMatrix()
    cam.lookAt(tx, ty, tz)
    if (controls) {
      controls.target.set(tx, ty, tz)
      controls.minDistance = Math.max(0.12, r * 0.15)
      controls.maxDistance = Math.max(80, r * 18)
      controls.update()
    }
  }, [view, camera, controls, focus])

  return null
}
