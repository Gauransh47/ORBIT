import { useState } from 'react'
import { Link } from 'react-router-dom'
import Section from '../Section'

export default function MapPreview() {
  const [tilt, setTilt] = useState({ x: 58, y: -18 })

  return (
    <Section
      kicker="ADAPTIVE 2.5D TERRAIN MAPPING"
      title="Resolution that follows range, not a uniform voxel soup."
    >
      <div className="grid items-start gap-10 lg:grid-cols-2">
        <div>
          <p className="text-sm leading-7 text-orbit-dim">
            Near the sensor, cells are small. Farther out they coarsen. Heights
            come from ground inliers. The explorer on the next page will rotate
            a real exported grid after Phase 3; this panel is a schematic you
            can tilt with the cursor.
          </p>
          <Link
            to="/map"
            className="mt-8 inline-flex rounded-full border border-orbit-cyan/50 px-6 py-2.5 text-sm text-orbit-cyan hover:bg-orbit-cyan/10"
          >
            Explore the map →
          </Link>
        </div>
        <div
          className="overflow-hidden rounded-2xl border border-orbit-line bg-orbit-panel p-6"
          onMouseMove={(e) => {
            const r = e.currentTarget.getBoundingClientRect()
            const nx = (e.clientX - r.left) / r.width - 0.5
            const ny = (e.clientY - r.top) / r.height - 0.5
            setTilt({ x: 58 + ny * 14, y: -18 + nx * 22 })
          }}
          onMouseLeave={() => setTilt({ x: 58, y: -18 })}
        >
          <p className="font-mono text-[10px] tracking-widest text-orbit-cyan">
            ADAPTIVE 2.5D TERRAIN MAP
          </p>
          <p className="mt-1 text-xs leading-5 text-orbit-dim">
            conceptual cell sizes · move cursor to rotate · not exported data
          </p>
          <div className="mt-8 flex h-48 items-center justify-center [perspective:900px]">
            <div
              className="grid grid-cols-8 gap-1 transition-transform duration-150 ease-out"
              style={{
                transform: `rotateX(${tilt.x}deg) rotateZ(${tilt.y}deg)`,
                transformStyle: 'preserve-3d',
              }}
            >
              {Array.from({ length: 64 }).map((_, i) => {
                const col = i % 8
                const row = Math.floor(i / 8)
                const dist = Math.hypot(col - 3.5, row - 3.5)
                const size = dist < 2 ? 10 : dist < 3.2 ? 12 : 14
                const h = 8 + ((col * 7 + row * 11) % 28)
                return (
                  <span
                    key={i}
                    className="block rounded-[2px] bg-orbit-ground/70"
                    style={{
                      width: size,
                      height: h,
                      transform: `translateZ(${h / 2}px)`,
                      opacity: 0.45 + (1 - dist / 6) * 0.5,
                    }}
                  />
                )
              })}
            </div>
          </div>
        </div>
      </div>
    </Section>
  )
}
