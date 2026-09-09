import { DoubleSide } from 'three'
import { Html } from '@react-three/drei'
import { orbitToThree } from '../../lib/orbitCoords'
import type { ProposalRecord, TrackRecord, WorldObjectRecord } from '../../types/orbit'

function Footprint({
  x,
  y,
  width,
  length,
  color,
  selected,
  label,
  onSelect,
}: {
  x: number
  y: number
  width: number
  length: number
  color: string
  selected?: boolean
  label?: string
  onSelect?: () => void
}) {
  const [px, py, pz] = orbitToThree(x, y, 0.12)
  const w = Math.max(width, 0.5)
  const l = Math.max(length, 0.5)
  return (
    <group>
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
          emissive={color}
          emissiveIntensity={selected ? 0.45 : 0.22}
          transparent
          opacity={selected ? 0.95 : 0.78}
          side={DoubleSide}
        />
      </mesh>
      <mesh position={[px, py + 0.02, pz]} rotation={[-Math.PI / 2, 0, 0]}>
        <ringGeometry args={[Math.max(l, w) * 0.42, Math.max(l, w) * 0.5, 20]} />
        <meshBasicMaterial color="#f4f1de" transparent opacity={0.85} side={DoubleSide} />
      </mesh>
      {label ? (
        <Html position={[px, py + 0.45, pz]} center distanceFactor={28}>
          <span className="rounded bg-black/70 px-1.5 py-0.5 font-mono text-[10px] text-orbit-cyan whitespace-nowrap">
            {label}
          </span>
        </Html>
      ) : null}
    </group>
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
  const labelAll = tracks.length + extras.length <= 40

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
            color={track.confirmed ? '#ff8a3c' : '#ffd36a'}
            selected={selectedId === track.track_id}
            label={labelAll || selectedId === track.track_id ? `#${track.track_id}` : undefined}
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
            color="#5ee0ff"
            selected={selectedId === obj.track_id}
            label={labelAll || selectedId === obj.track_id ? `W#${obj.track_id}` : undefined}
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
            color="#8ee4ff"
          />
        )
      })}
    </group>
  )
}

export function EgoMarker({ xy, radius = 0.28 }: { xy: [number, number]; radius?: number }) {
  const [x, y, z] = orbitToThree(xy[0], xy[1], radius * 0.55)
  return (
    <mesh position={[x, y, z]}>
      <coneGeometry args={[radius, radius * 2.5, 8]} />
      <meshStandardMaterial color="#7af0ff" emissive="#3ddcff" emissiveIntensity={0.35} />
    </mesh>
  )
}
