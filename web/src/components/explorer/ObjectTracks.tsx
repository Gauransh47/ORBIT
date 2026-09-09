import { DoubleSide } from 'three'
import { orbitToThree } from '../../lib/orbitCoords'
import type { ProposalRecord, TrackRecord, WorldObjectRecord } from '../../types/orbit'

function Footprint({
  x,
  y,
  width,
  length,
  color,
  selected,
  onSelect,
}: {
  x: number
  y: number
  width: number
  length: number
  color: string
  selected?: boolean
  onSelect?: () => void
}) {
  const [px, py, pz] = orbitToThree(x, y, 0.04)
  const w = Math.max(width, 0.4)
  const l = Math.max(length, 0.4)
  return (
    <mesh
      position={[px, py, pz]}
      rotation={[-Math.PI / 2, 0, 0]}
      onClick={(e) => {
        e.stopPropagation()
        onSelect?.()
      }}
    >
      <planeGeometry args={[l, w]} />
      <meshStandardMaterial
        color={color}
        transparent
        opacity={selected ? 0.9 : 0.55}
        side={DoubleSide}
      />
    </mesh>
  )
}

export default function ObjectTracks({
  tracks,
  worldObjects,
  selectedId,
  onSelect,
}: {
  tracks: TrackRecord[]
  worldObjects: WorldObjectRecord[]
  selectedId: number | null
  onSelect: (id: number | null) => void
}) {
  const trackIds = new Set(tracks.map((t) => t.track_id))
  const extras = worldObjects.filter((o) => !trackIds.has(o.track_id))

  return (
    <group
      onPointerMissed={() => {
        onSelect(null)
      }}
    >
      {tracks.map((track) => {
        const dims = track.dimensions_xy ?? [1.2, 2.4]
        return (
          <Footprint
            key={`t-${track.track_id}`}
            x={track.position[0]}
            y={track.position[1]}
            width={dims[0] ?? 1.2}
            length={dims[1] ?? 2.4}
            color={track.confirmed ? '#e08a3c' : '#f0c27a'}
            selected={selectedId === track.track_id}
            onSelect={() => onSelect(track.track_id)}
          />
        )
      })}
      {extras.map((obj) => {
        const dims = obj.dimensions_xy ?? [0.8, 0.8]
        return (
          <Footprint
            key={`w-${obj.track_id}`}
            x={obj.position[0]}
            y={obj.position[1]}
            width={dims[0] ?? 0.8}
            length={dims[1] ?? 0.8}
            color="#3ddcff"
            selected={selectedId === obj.track_id}
            onSelect={() => onSelect(obj.track_id)}
          />
        )
      })}
    </group>
  )
}

export function ProposalFootprints({ proposals }: { proposals: ProposalRecord[] }) {
  return (
    <group>
      {proposals.map((p, i) => {
        if (!p.center || p.center.length < 2) return null
        return (
          <Footprint
            key={p.proposal_id ?? i}
            x={p.center[0]}
            y={p.center[1]}
            width={p.width ?? 1}
            length={p.length ?? 1}
            color="#7dd3fc"
          />
        )
      })}
    </group>
  )
}

export function EgoMarker({ xy }: { xy: [number, number] }) {
  const [x, y, z] = orbitToThree(xy[0], xy[1], 0.15)
  return (
    <mesh position={[x, y, z]}>
      <coneGeometry args={[0.28, 0.7, 8]} />
      <meshStandardMaterial color="#3ddcff" />
    </mesh>
  )
}
