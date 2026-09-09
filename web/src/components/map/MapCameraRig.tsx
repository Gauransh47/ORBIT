import { useEffect } from 'react'
import { useThree } from '@react-three/fiber'

export type MapCameraView = 'iso' | 'top' | 'side'

type OrbitLike = {
  target: { set: (x: number, y: number, z: number) => void }
  update: () => void
}

export default function MapCameraRig({ view }: { view: MapCameraView }) {
  const camera = useThree((s) => s.camera)
  const controls = useThree((s) => s.controls) as OrbitLike | null

  useEffect(() => {
    if (view === 'top') camera.position.set(0.2, 92, 0.2)
    else if (view === 'side') camera.position.set(78, 14, 8)
    else camera.position.set(42, 38, 42)
    camera.lookAt(0, 0, 0)
    if (controls) {
      controls.target.set(0, 0, 0)
      controls.update()
    }
  }, [view, camera, controls])

  return null
}
