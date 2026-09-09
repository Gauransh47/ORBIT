import { useEffect, useRef } from 'react'

type Point = { x: number; y: number; r: number; a: number }

export default function LidarField() {
  const canvasRef = useRef<HTMLCanvasElement | null>(null)

  useEffect(() => {
    const canvas = canvasRef.current
    if (!canvas) return
    const ctx = canvas.getContext('2d')
    if (!ctx) return

    let frame = 0
    let raf = 0
    const points: Point[] = Array.from({ length: 420 }, () => {
      const angle = Math.random() * Math.PI * 2
      const radius = 40 + Math.random() * 280
      return { x: 0, y: 0, r: radius, a: angle }
    })

    const resize = () => {
      const dpr = Math.min(window.devicePixelRatio || 1, 2)
      canvas.width = Math.floor(canvas.clientWidth * dpr)
      canvas.height = Math.floor(canvas.clientHeight * dpr)
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0)
    }
    resize()
    window.addEventListener('resize', resize)

    const draw = () => {
      const w = canvas.clientWidth
      const h = canvas.clientHeight
      ctx.fillStyle = '#07090e'
      ctx.fillRect(0, 0, w, h)

      const cx = w * 0.58
      const cy = h * 0.52
      const scan = (frame * 0.012) % (Math.PI * 2)

      ctx.strokeStyle = 'rgba(61, 220, 255, 0.08)'
      ctx.lineWidth = 1
      for (let i = 1; i <= 6; i += 1) {
        ctx.beginPath()
        ctx.arc(cx, cy, i * 55, 0, Math.PI * 2)
        ctx.stroke()
      }
      for (let i = 0; i < 8; i += 1) {
        const a = (i / 8) * Math.PI * 2
        ctx.beginPath()
        ctx.moveTo(cx, cy)
        ctx.lineTo(cx + Math.cos(a) * 340, cy + Math.sin(a) * 340)
        ctx.stroke()
      }

      const gradient = ctx.createRadialGradient(cx, cy, 0, cx, cy, 320)
      gradient.addColorStop(0, 'rgba(61, 220, 255, 0.18)')
      gradient.addColorStop(1, 'rgba(61, 220, 255, 0)')
      ctx.fillStyle = gradient
      ctx.beginPath()
      ctx.moveTo(cx, cy)
      ctx.arc(cx, cy, 320, scan - 0.22, scan + 0.22)
      ctx.closePath()
      ctx.fill()

      for (const p of points) {
        const x = cx + Math.cos(p.a) * p.r
        const y = cy + Math.sin(p.a) * p.r * 0.62
        const delta = Math.abs(((p.a - scan + Math.PI * 3) % (Math.PI * 2)) - Math.PI)
        const lit = Math.max(0.12, 1 - delta * 1.6)
        ctx.fillStyle = `rgba(61, 220, 255, ${0.15 + lit * 0.7})`
        ctx.fillRect(x, y, 1.7, 1.7)
      }

      ctx.fillStyle = '#3ddcff'
      ctx.beginPath()
      ctx.arc(cx, cy, 4, 0, Math.PI * 2)
      ctx.fill()
      ctx.strokeStyle = '#3ddcff'
      ctx.beginPath()
      ctx.moveTo(cx, cy)
      ctx.lineTo(cx + 28, cy)
      ctx.stroke()

      frame += 1
      raf = requestAnimationFrame(draw)
    }
    raf = requestAnimationFrame(draw)
    return () => {
      cancelAnimationFrame(raf)
      window.removeEventListener('resize', resize)
    }
  }, [])

  return <canvas ref={canvasRef} className="absolute inset-0 h-full w-full" />
}
