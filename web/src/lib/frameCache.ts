import type { FrameJson } from '../types/orbit'
import { FRAME_CACHE_LIMIT } from '../types/orbit'

/** LRU cache of fetched frame JSON. Does not run ORBIT algorithms. */
export class FrameCache {
  private readonly map = new Map<number, FrameJson>()
  private readonly order: number[] = []
  private readonly max: number

  constructor(max = FRAME_CACHE_LIMIT) {
    this.max = max
  }

  get(frameIndex: number): FrameJson | undefined {
    const hit = this.map.get(frameIndex)
    if (!hit) return undefined
    const at = this.order.indexOf(frameIndex)
    if (at >= 0) {
      this.order.splice(at, 1)
      this.order.push(frameIndex)
    }
    return hit
  }

  set(frameIndex: number, frame: FrameJson): void {
    if (this.map.has(frameIndex)) {
      this.map.set(frameIndex, frame)
      this.get(frameIndex)
      return
    }
    while (this.order.length >= this.max) {
      const evict = this.order.shift()
      if (evict !== undefined) this.map.delete(evict)
    }
    this.map.set(frameIndex, frame)
    this.order.push(frameIndex)
  }

  has(frameIndex: number): boolean {
    return this.map.has(frameIndex)
  }

  get size(): number {
    return this.map.size
  }
}
