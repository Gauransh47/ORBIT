import { lazy, Suspense } from 'react'
import { Route, Routes } from 'react-router-dom'
import Nav from './components/Nav'
import Home from './pages/Home'
import About from './pages/About'
import Pipeline from './pages/Pipeline'
import Planning from './pages/Planning'
import Technology from './pages/Technology'

const Demo = lazy(() => import('./pages/Demo'))
const MapExplorer = lazy(() => import('./pages/MapExplorer'))

export default function App() {
  return (
    <div className="min-h-svh">
      <Nav />
      <Suspense
        fallback={
          <p className="px-6 pt-28 font-mono text-[11px] tracking-[0.28em] text-orbit-cyan">
            LOADING EXPLORER
          </p>
        }
      >
        <Routes>
          <Route path="/" element={<Home />} />
          <Route path="/about" element={<About />} />
          <Route path="/pipeline" element={<Pipeline />} />
          <Route path="/demo" element={<Demo />} />
          <Route path="/map" element={<MapExplorer />} />
          <Route path="/planning" element={<Planning />} />
          <Route path="/technology" element={<Technology />} />
        </Routes>
      </Suspense>
    </div>
  )
}
