/** Conceptual SVG sketches — not exported pipeline data. */

export function LidarSketch({ className = '' }: { className?: string }) {
  const dots = [
    [40, 42],
    [62, 28],
    [88, 50],
    [110, 22],
    [132, 44],
    [70, 62],
    [98, 70],
    [48, 74],
    [150, 36],
    [168, 58],
    [28, 54],
    [120, 78],
  ]
  return (
    <svg viewBox="0 0 200 100" className={className} aria-hidden>
      <ellipse cx="100" cy="52" rx="78" ry="28" fill="none" stroke="rgba(61,220,255,0.25)" />
      <ellipse cx="100" cy="52" rx="48" ry="16" fill="none" stroke="rgba(61,220,255,0.18)" />
      <line x1="100" y1="52" x2="168" y2="30" stroke="rgba(61,220,255,0.45)" strokeWidth="1">
        <animateTransform
          attributeName="transform"
          type="rotate"
          from="0 100 52"
          to="360 100 52"
          dur="8s"
          repeatCount="indefinite"
        />
      </line>
      {dots.map(([x, y], i) => (
        <circle key={i} cx={x} cy={y} r={i % 3 === 0 ? 1.8 : 1.2} fill="#7dd3fc" opacity={0.55 + (i % 4) * 0.1} />
      ))}
    </svg>
  )
}

export function TerrainSketch({ className = '' }: { className?: string }) {
  return (
    <svg viewBox="0 0 200 100" className={className} aria-hidden>
      <path d="M8 72 Q 50 68 90 70 T 192 64" fill="none" stroke="#2fbf9a" strokeWidth="1.4" />
      <path d="M8 78 Q 70 86 140 76 T 192 80" fill="none" stroke="rgba(47,191,154,0.35)" />
      {[20, 50, 80, 110, 140, 170].map((x, i) => (
        <rect key={x} x={x} y={48 - i * 3} width={14} height={22 + i * 2} rx="1" fill="rgba(47,191,154,0.35)" />
      ))}
    </svg>
  )
}

export function GridSketch({ className = '' }: { className?: string }) {
  const cells = [
    { x: 20, y: 40, s: 10, h: 18 },
    { x: 32, y: 38, s: 10, h: 22 },
    { x: 44, y: 34, s: 12, h: 28 },
    { x: 60, y: 30, s: 16, h: 36 },
    { x: 80, y: 28, s: 22, h: 40 },
    { x: 108, y: 36, s: 28, h: 30 },
    { x: 142, y: 42, s: 36, h: 24 },
  ]
  return (
    <svg viewBox="0 0 200 100" className={className} aria-hidden>
      {cells.map((c, i) => (
        <rect
          key={i}
          x={c.x}
          y={c.y}
          width={c.s}
          height={c.h}
          rx="1.5"
          fill="rgba(61,220,255,0.12)"
          stroke="rgba(61,220,255,0.45)"
        />
      ))}
    </svg>
  )
}

export function ObjectsSketch({ className = '' }: { className?: string }) {
  return (
    <svg viewBox="0 0 200 100" className={className} aria-hidden>
      <rect x="36" y="42" width="48" height="28" fill="none" stroke="#e08a3c" />
      <circle cx="112" cy="58" r="12" fill="none" stroke="#3ddcff" />
      <rect x="148" y="30" width="8" height="44" fill="rgba(224,86,86,0.7)" />
    </svg>
  )
}

export function TrackSketch({ className = '' }: { className?: string }) {
  return (
    <svg viewBox="0 0 200 100" className={className} aria-hidden>
      <path d="M16 70 C 60 20, 100 86, 186 34" fill="none" stroke="#3ddcff" strokeWidth="1.4" />
      <circle cx="42" cy="52" r="4" fill="#3ddcff" />
      <circle cx="108" cy="64" r="4" fill="#3ddcff" />
      <circle cx="170" cy="38" r="4" fill="#e08a3c" />
    </svg>
  )
}

export function WorldSketch({ className = '' }: { className?: string }) {
  return (
    <svg viewBox="0 0 200 100" className={className} aria-hidden>
      <rect x="24" y="18" width="152" height="64" fill="none" stroke="rgba(61,220,255,0.28)" strokeDasharray="4 4" />
      <text x="32" y="34" fill="#3ddcff" fontSize="8" fontFamily="monospace">
        START
      </text>
      <circle cx="48" cy="58" r="3" fill="#3ddcff" />
      <circle cx="92" cy="50" r="3" fill="#7dd3fc" />
      <circle cx="140" cy="62" r="3" fill="#e08a3c" />
      <text x="132" y="74" fill="#e7eef6" fontSize="8" fontFamily="monospace">
        CURRENT
      </text>
    </svg>
  )
}

export function RouteSketch({ className = '' }: { className?: string }) {
  return (
    <svg viewBox="0 0 200 100" className={className} aria-hidden>
      <rect x="70" y="28" width="36" height="22" fill="rgba(224,86,86,0.25)" stroke="#e05656" />
      <path
        d="M18 78 L 58 78 L 88 52 L 150 52 L 184 28"
        fill="none"
        stroke="#2fbf9a"
        strokeWidth="2"
        strokeDasharray="7 5"
      >
        <animate attributeName="stroke-dashoffset" from="0" to="-48" dur="2.4s" repeatCount="indefinite" />
      </path>
      <circle cx="18" cy="78" r="3" fill="#2fbf9a" />
      <circle cx="184" cy="28" r="3" fill="#3ddcff" />
    </svg>
  )
}

export function CloudField({ className = '' }: { className?: string }) {
  const pts = Array.from({ length: 48 }, (_, i) => ({
    x: 10 + ((i * 37) % 180),
    y: 12 + ((i * 19) % 76),
    r: 0.8 + (i % 3) * 0.4,
  }))
  return (
    <svg viewBox="0 0 200 100" className={className} aria-hidden>
      {pts.map((p, i) => (
        <circle key={i} cx={p.x} cy={p.y} r={p.r} fill="#7dd3fc" opacity={0.25 + (i % 5) * 0.12} />
      ))}
    </svg>
  )
}
