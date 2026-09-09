import type { AdaptiveCellRecord, ObstacleCellRecord } from '../types/orbit'
import { isRenderableCell } from './cellVisual'

export type PlanStatus = 'idle' | 'ready' | 'no-grid' | 'no-start' | 'no-goal' | 'no-path' | 'blocked-start' | 'blocked-goal'

export type PlanResult = {
  status: PlanStatus
  path: [number, number][]
  lengthM: number
  explored: number
  start: [number, number] | null
  goal: [number, number] | null
  startIndex: number | null
  goalIndex: number | null
  traversable: number
  blocked: number
  note: string
}

export type PlanningGraph = {
  xs: Float64Array
  ys: Float64Array
  res: Float64Array
  blocked: Uint8Array
  neighbors: number[][]
  traversable: number
  blockedCount: number
}

function hashObstacles(obstacles: ObstacleCellRecord[]): Map<string, ObstacleCellRecord[]> {
  const buckets = new Map<string, ObstacleCellRecord[]>()
  for (const obs of obstacles) {
    if (!Array.isArray(obs.center) || obs.center.length < 2) continue
    const key = `${Math.floor(obs.center[0])},${Math.floor(obs.center[1])}`
    const list = buckets.get(key)
    if (list) list.push(obs)
    else buckets.set(key, [obs])
  }
  return buckets
}

function obstacleMarksCell(
  cell: AdaptiveCellRecord,
  buckets: Map<string, ObstacleCellRecord[]>,
): boolean {
  const ix = Math.floor(cell.center[0])
  const iy = Math.floor(cell.center[1])
  for (let dx = -1; dx <= 1; dx++) {
    for (let dy = -1; dy <= 1; dy++) {
      const list = buckets.get(`${ix + dx},${iy + dy}`)
      if (!list) continue
      for (const obs of list) {
        const lim = (cell.resolution + obs.resolution) * 0.5 + 1e-4
        if (Math.abs(cell.center[0] - obs.center[0]) <= lim && Math.abs(cell.center[1] - obs.center[1]) <= lim) {
          return true
        }
      }
    }
  }
  return false
}

export function isBlockedCell(
  cell: AdaptiveCellRecord,
  obstacles: ObstacleCellRecord[],
  obstacleHash?: Map<string, ObstacleCellRecord[]>,
): boolean {
  if (cell.semantic_class === 'OBSTACLE' || cell.semantic_class === 'MIXED') return true
  if ((cell.obstacle_count ?? 0) > 0) return true
  if (obstacles.length && obstacleMarksCell(cell, obstacleHash ?? hashObstacles(obstacles))) return true
  return false
}

function binKey(x: number, y: number, bin: number): string {
  return `${Math.floor(x / bin)},${Math.floor(y / bin)}`
}

export function buildPlanningGraph(
  cells: AdaptiveCellRecord[],
  obstacles: ObstacleCellRecord[],
): PlanningGraph | null {
  const usable = cells.filter(isRenderableCell)
  if (!usable.length) return null
  const n = usable.length
  const xs = new Float64Array(n)
  const ys = new Float64Array(n)
  const res = new Float64Array(n)
  const blocked = new Uint8Array(n)
  const obstacleHash = hashObstacles(obstacles)
  let blockedCount = 0
  for (let i = 0; i < n; i++) {
    xs[i] = usable[i].center[0]
    ys[i] = usable[i].center[1]
    res[i] = usable[i].resolution
    const b = isBlockedCell(usable[i], obstacles, obstacleHash)
    blocked[i] = b ? 1 : 0
    if (b) blockedCount++
  }
  const bin = 1
  const buckets = new Map<string, number[]>()
  for (let i = 0; i < n; i++) {
    const key = binKey(xs[i], ys[i], bin)
    const list = buckets.get(key)
    if (list) list.push(i)
    else buckets.set(key, [i])
  }
  const neighbors: number[][] = Array.from({ length: n }, () => [])
  for (let i = 0; i < n; i++) {
    if (blocked[i]) continue
    const ix = Math.floor(xs[i] / bin)
    const iy = Math.floor(ys[i] / bin)
    for (let dx = -1; dx <= 1; dx++) {
      for (let dy = -1; dy <= 1; dy++) {
        const list = buckets.get(`${ix + dx},${iy + dy}`)
        if (!list) continue
        for (const j of list) {
          if (j <= i || blocked[j]) continue
          const lim = (res[i] + res[j]) * 0.5 + 1e-3
          if (Math.abs(xs[i] - xs[j]) <= lim && Math.abs(ys[i] - ys[j]) <= lim) {
            neighbors[i].push(j)
            neighbors[j].push(i)
          }
        }
      }
    }
  }
  return {
    xs,
    ys,
    res,
    blocked,
    neighbors,
    traversable: n - blockedCount,
    blockedCount,
  }
}

function nearestIndex(
  graph: PlanningGraph,
  xy: [number, number],
  requireFree: boolean,
): number | null {
  let best = -1
  let bestD = Infinity
  for (let i = 0; i < graph.xs.length; i++) {
    if (requireFree && graph.blocked[i]) continue
    const d = (graph.xs[i] - xy[0]) ** 2 + (graph.ys[i] - xy[1]) ** 2
    if (d < bestD) {
      bestD = d
      best = i
    }
  }
  if (best < 0) return null
  const snap = Math.max(graph.res[best] * 2, 1.5)
  if (Math.sqrt(bestD) > snap) return null
  return best
}

function astar(graph: PlanningGraph, start: number, goal: number): { path: number[]; explored: number } | null {
  const n = graph.xs.length
  const gScore = new Float64Array(n)
  gScore.fill(Infinity)
  gScore[start] = 0
  const came = new Int32Array(n)
  came.fill(-1)
  const h = (i: number) => Math.hypot(graph.xs[i] - graph.xs[goal], graph.ys[i] - graph.ys[goal])
  const heap: { i: number; f: number }[] = []
  const bubbleUp = (idx: number) => {
    while (idx > 0) {
      const p = (idx - 1) >> 1
      if (heap[p].f <= heap[idx].f) break
      const tmp = heap[p]
      heap[p] = heap[idx]
      heap[idx] = tmp
      idx = p
    }
  }
  const sink = (idx: number) => {
    for (;;) {
      let m = idx
      const l = idx * 2 + 1
      const r = l + 1
      if (l < heap.length && heap[l].f < heap[m].f) m = l
      if (r < heap.length && heap[r].f < heap[m].f) m = r
      if (m === idx) break
      const tmp = heap[m]
      heap[m] = heap[idx]
      heap[idx] = tmp
      idx = m
    }
  }
  heap.push({ i: start, f: h(start) })
  const closed = new Uint8Array(n)
  let explored = 0
  while (heap.length) {
    const current = heap[0]
    const last = heap.pop()!
    if (heap.length) {
      heap[0] = last
      sink(0)
    } else if (current !== last) {
      /* popped the only item */
    }
    if (closed[current.i]) continue
    closed[current.i] = 1
    explored++
    if (current.i === goal) {
      const path = [current.i]
      let c = current.i
      while (came[c] >= 0) {
        c = came[c]
        path.push(c)
      }
      path.reverse()
      return { path, explored }
    }
    for (const nb of graph.neighbors[current.i]) {
      if (closed[nb]) continue
      const step = Math.hypot(graph.xs[nb] - graph.xs[current.i], graph.ys[nb] - graph.ys[current.i])
      const tentative = gScore[current.i] + step
      if (tentative < gScore[nb]) {
        came[nb] = current.i
        gScore[nb] = tentative
        heap.push({ i: nb, f: tentative + h(nb) })
        bubbleUp(heap.length - 1)
      }
    }
  }
  return { path: [], explored }
}

export function planPath(
  cells: AdaptiveCellRecord[],
  obstacles: ObstacleCellRecord[],
  startXy: [number, number] | null,
  goalXy: [number, number] | null,
): PlanResult {
  const empty: PlanResult = {
    status: 'idle',
    path: [],
    lengthM: 0,
    explored: 0,
    start: startXy,
    goal: goalXy,
    startIndex: null,
    goalIndex: null,
    traversable: 0,
    blocked: 0,
    note: '',
  }
  const graph = buildPlanningGraph(cells, obstacles)
  if (!graph) {
    return { ...empty, status: 'no-grid', note: 'No exported adaptive cells to plan on.' }
  }
  empty.traversable = graph.traversable
  empty.blocked = graph.blockedCount
  if (!startXy) {
    return { ...empty, status: 'no-start', note: 'Start is not set.' }
  }
  if (!goalXy) {
    return { ...empty, status: 'no-goal', note: 'Click the map to set a destination on exported ground.' }
  }
  const startIndex = nearestIndex(graph, startXy, true)
  if (startIndex == null) {
    return { ...empty, status: 'blocked-start', note: 'Ego / start does not lie on a traversable exported GROUND cell.' }
  }
  const goalIndex = nearestIndex(graph, goalXy, true)
  if (goalIndex == null) {
    return {
      ...empty,
      status: 'blocked-goal',
      start: [graph.xs[startIndex], graph.ys[startIndex]],
      startIndex,
      note: 'Goal is not on a traversable exported GROUND cell. Obstacles and MIXED cells are treated as blocked.',
    }
  }
  const search = astar(graph, startIndex, goalIndex)
  if (!search || !search.path.length) {
    return {
      ...empty,
      status: 'no-path',
      start: [graph.xs[startIndex], graph.ys[startIndex]],
      goal: [graph.xs[goalIndex], graph.ys[goalIndex]],
      startIndex,
      goalIndex,
      explored: search?.explored ?? 0,
      note: 'No route through exported GROUND cells. Nothing was drawn.',
    }
  }
  const path: [number, number][] = search.path.map((i) => [graph.xs[i], graph.ys[i]])
  let lengthM = 0
  for (let i = 1; i < path.length; i++) {
    lengthM += Math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
  }
  return {
    status: 'ready',
    path,
    lengthM,
    explored: search.explored,
    start: path[0],
    goal: path[path.length - 1],
    startIndex,
    goalIndex,
    traversable: graph.traversable,
    blocked: graph.blockedCount,
    note: 'A* on exported adaptive GROUND cells. Not an ORBIT runtime planner.',
  }
}

export function pointAlongPath(path: [number, number][], t: number): [number, number] | null {
  if (path.length === 0) return null
  if (path.length === 1) return path[0]
  const clamped = Math.min(1, Math.max(0, t))
  let total = 0
  const segs: number[] = []
  for (let i = 1; i < path.length; i++) {
    const d = Math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
    segs.push(d)
    total += d
  }
  if (total <= 0) return path[path.length - 1]
  let remain = clamped * total
  for (let i = 0; i < segs.length; i++) {
    if (remain <= segs[i] || i === segs.length - 1) {
      const u = segs[i] <= 0 ? 1 : remain / segs[i]
      return [
        path[i][0] + (path[i + 1][0] - path[i][0]) * u,
        path[i][1] + (path[i + 1][1] - path[i][1]) * u,
      ]
    }
    remain -= segs[i]
  }
  return path[path.length - 1]
}

export function planStatusLabel(status: PlanStatus): string {
  if (status === 'ready') return 'PATH FOUND'
  if (status === 'no-path') return 'NO PATH AVAILABLE'
  if (status === 'blocked-start') return 'START BLOCKED'
  if (status === 'blocked-goal') return 'DESTINATION BLOCKED'
  if (status === 'no-start') return 'WAITING FOR START'
  if (status === 'no-goal') return 'WAITING FOR DESTINATION'
  if (status === 'no-grid') return 'NO GRID'
  return 'IDLE'
}
