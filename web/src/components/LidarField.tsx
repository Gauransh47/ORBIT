import { useEffect, useRef } from 'react'

type Particle = {
  x: number
  y: number
  z: number
  kind: 'cloud' | 'ground'
}

function project(x: number, y: number, z: number, w: number, h: number, cx: number, cy: number) {
  const persp = 280 / (280 + z)
  return {
    px: w / 2 + (x - cx) * persp,
    py: h * 0.42 + (y - cy) * persp + z * 0.12,
    persp,
  }
}

export default function LidarField() {
  const canvasRef = useRef<HTMLCanvasElement>(null)

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext('2d', { alpha: true })
    if (!ctx) return

    const reduced = window.matchMedia('(prefers-reduced-motion: reduce)').matches
    let raf = 0
    let t = 0
    let mouseX = 0
    let mouseY = 0
    let targetMx = 0
    let targetMy = 0
    let visible = true

    const particles: Particle[] = []
    const nCloud = reduced ? 140 : 420
    const nGround = reduced ? 40 : 90
    for (let i = 0; i < nCloud; i++) {
      particles.push({
        x: (Math.random() - 0.5) * 920,
        y: (Math.random() - 0.5) * 520,
        z: Math.random() * 520,
        kind: 'cloud',
      })
    }
    for (let i = 0; i < nGround; i++) {
      particles.push({
        x: (Math.random() - 0.5) * 880,
        y: 170 + Math.random() * 70,
        z: Math.random() * 480,
        kind: 'ground',
      })
    }

    const resize = () => {
      const dpr = Math.min(window.devicePixelRatio || 1, 2)
      const rect = canvas.getBoundingClientRect()
      canvas.width = Math.max(1, Math.floor(rect.width * dpr))
      canvas.height = Math.max(1, Math.floor(rect.height * dpr))
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
    }
    resize()

    const onMove = (e: MouseEvent) => {
      const rect = canvas.getBoundingClientRect()
      targetMx = ((e.clientX - rect.left) / rect.width - 0.5) * 2
      targetMy = ((e.clientY - rect.top) / rect.height - 0.5) * 2
    }

    const observer = new IntersectionObserver(
      ([entry]) => {
        visible = entry.isIntersecting
      },
      { threshold: 0.05 },
    )
    observer.observe(canvas)

    const draw = () => {
      const w = canvas.clientWidth
      const h = canvas.clientHeight
      ctx.clearRect(0, 0, w, h)

      mouseX += (targetMx - mouseX) * 0.06
      mouseY += (targetMy - mouseY) * 0.06
      const camX = mouseX * 42
      const camY = mouseY * 22

      const grd = ctx.createRadialGradient(w * 0.5, h * 0.38, 20, w * 0.5, h * 0.5, Math.max(w, h) * 0.72)
      grd.addColorStop(0, 'rgba(8, 22, 36, 0.15)')
      grd.addColorStop(1, 'rgba(4, 8, 14, 0.55)')
      ctx.fillStyle = grd
      ctx.fillRect(0, 0, w, h)

      ctx.strokeStyle = 'rgba(94, 234, 212, 0.07)'
      ctx.lineWidth = 1
      const gy = h * 0.62 + camY * 0.15
      for (let i = -8; i <= 8; i++) {
        ctx.beginPath()
        ctx.moveTo(w / 2 + camX * 0.3, gy - 40)
        ctx.lineTo(i * (w / 10) + w / 2 + camX, h + 20)
        ctx.stroke()
      }
      for (let j = 0; j < 10; j++) {
        const y = gy + j * 28
        ctx.beginPath()
        ctx.moveTo(0, y)
        ctx.lineTo(w, y + 8)
        ctx.stroke()
      }

      ctx.strokeStyle = 'rgba(56, 189, 248, 0.18)'
      ctx.beginPath()
      ctx.ellipse(w / 2 + camX * 0.2, h * 0.46 + camY * 0.1, 210, 58, 0.05, 0, Math.PI * 2)
      ctx.stroke()
      ctx.strokeStyle = 'rgba(94, 234, 212, 0.12)'
      ctx.beginPath()
      ctx.ellipse(w / 2 + camX * 0.15, h * 0.5 + camY * 0.08, 320, 88, -0.04, 0, Math.PI * 2)
      ctx.stroke()

      const sweep = ((t * 0.35) % (Math.PI * 2)) - Math.PI
      ctx.save()
      ctx.translate(w / 2 + camX * 0.25, h * 0.48)
      ctx.rotate(sweep)
      const beam = ctx.createLinearGradient(0, 0, 380, 0)
      beam.addColorStop(0, 'rgba(94, 234, 212, 0)')
      beam.addColorStop(0.5, 'rgba(94, 234, 212, 0.07)')
      beam.addColorStop(1, 'rgba(94, 234, 212, 0)')
      ctx.fillStyle = beam
      ctx.beginPath()
      ctx.moveTo(0, 0)
      ctx.lineTo(360, -36)
      ctx.lineTo(360, 36)
      ctx.closePath()
      ctx.fill()
      ctx.restore()

      ctx.strokeStyle = 'rgba(251, 146, 60, 0.28)'
      ctx.lineWidth = 1.4
      ctx.beginPath()
      for (let s = 0; s < 28; s++) {
        const u = s / 27
        const x = (u - 0.5) * 640 + Math.sin(u * 6 + t * 0.12) * 24 + camX * 0.4
        const y = h * 0.52 + Math.cos(u * 4) * 36 + camY * 0.3
        if (s === 0) ctx.moveTo(x, y)
        else ctx.lineTo(x, y)
      }
      ctx.stroke()

      for (const p of particles) {
        if (!reduced) {
          p.z -= p.kind === 'ground' ? 0.55 : 1.05
          if (p.z < -40) {
            p.z = 520
            p.x = (Math.random() - 0.5) * 920
            p.y = p.kind === 'ground' ? 170 + Math.random() * 70 : (Math.random() - 0.5) * 520
          }
        }
        const { px, py, persp } = project(p.x, p.y, p.z, w, h, camX, camY)
        const alpha = Math.max(0.08, Math.min(0.85, persp * 0.85))
        ctx.fillStyle =
          p.kind === 'ground' ? `rgba(52, 211, 153, ${alpha * 0.55})` : `rgba(125, 211, 252, ${alpha})`
        const size = p.kind === 'ground' ? 1.4 * persp : 1.15 * persp
        ctx.fillRect(px, py, size, size)
      }

      ctx.fillStyle = 'rgba(4, 8, 14, 0.72)'
      ctx.fillRect(0, h * 0.78, w, h * 0.22)
      const veil = ctx.createLinearGradient(0, h * 0.55, 0, h)
      veil.addColorStop(0, 'rgba(4, 8, 14, 0)')
      veil.addColorStop(1, 'rgba(4, 8, 14, 0.88)')
      ctx.fillStyle = veil
      ctx.fillRect(0, h * 0.55, w, h * 0.45)

      const side = ctx.createLinearGradient(0, 0, w, 0)
      side.addColorStop(0, 'rgba(4, 8, 14, 0.55)')
      side.addColorStop(0.22, 'rgba(4, 8, 14, 0)')
      side.addColorStop(0.78, 'rgba(4, 8, 14, 0)')
      side.addColorStop(1, 'rgba(4, 8, 14, 0.55)')
      ctx.fillStyle = side
      ctx.fillRect(0, 0, w, h)
    }

    const loop = () => {
      if (visible) {
        t += 1
        draw()
      }
      raf = requestAnimationFrame(loop)
    }

    window.addEventListener('resize', resize)
    window.addEventListener('mousemove', onMove, { passive: true })
    draw()
    if (!reduced) raf = requestAnimationFrame(loop)

    return () => {
      cancelAnimationFrame(raf)
      window.removeEventListener('resize', resize)
      window.removeEventListener('mousemove', onMove)
      observer.disconnect()
    }
  }, [])

  return <canvas ref={canvasRef} className="absolute inset-0 h-full w-full" aria-hidden />
}
