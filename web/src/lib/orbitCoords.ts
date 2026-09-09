/** Map ORBIT LiDAR (x forward, y left, z up) into Three.js (y up). */
export function orbitToThree(x: number, y: number, z: number): [number, number, number] {
  return [x, z, -y]
}
