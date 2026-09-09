import LidarPoints from './LidarPoints'

export default function WorldPoints({ points }: { points: number[][] }) {
  return <LidarPoints points={points} color="#7dd3c7" />
}
