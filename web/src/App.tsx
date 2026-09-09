import { Route, Routes } from 'react-router-dom'
import Nav from './components/Nav'
import Home from './pages/Home'
import About from './pages/About'
import Pipeline from './pages/Pipeline'
import Demo from './pages/Demo'
import MapExplorer from './pages/MapExplorer'
import Planning from './pages/Planning'
import Technology from './pages/Technology'

export default function App() {
  return (
    <div className="min-h-svh">
      <Nav />
      <Routes>
        <Route path="/" element={<Home />} />
        <Route path="/about" element={<About />} />
        <Route path="/pipeline" element={<Pipeline />} />
        <Route path="/demo" element={<Demo />} />
        <Route path="/map" element={<MapExplorer />} />
        <Route path="/planning" element={<Planning />} />
        <Route path="/technology" element={<Technology />} />
      </Routes>
    </div>
  )
}
