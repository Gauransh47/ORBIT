import { defineConfig, loadEnv } from 'vite'
import react from '@vitejs/plugin-react'
import tailwindcss from '@tailwindcss/vite'

export default defineConfig(({ mode }) => {
  const env = loadEnv(mode, process.cwd(), '')
  const dataUrl = (env.VITE_ORBIT_DATA_URL || '').trim().replace(/\/+$/, '')
  const orbitDataProxy = dataUrl
    ? {
        '/orbit-data': {
          target: dataUrl,
          changeOrigin: true,
          rewrite: (path: string) => path.replace(/^\/orbit-data/, ''),
        },
      }
    : undefined

  return {
    plugins: [react(), tailwindcss()],
    server: orbitDataProxy ? { proxy: orbitDataProxy } : undefined,
    preview: orbitDataProxy ? { proxy: orbitDataProxy } : undefined,
  }
})
