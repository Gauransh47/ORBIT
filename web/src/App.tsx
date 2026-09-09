import { NavLink, Route, Routes } from 'react-router-dom'
import Home from './pages/Home'
import About from './pages/About'
import Pipeline from './pages/Pipeline'
import Demo from './pages/Demo'
import MapExplorer from './pages/MapExplorer'
import Planning from './pages/Planning'
import Technology from './pages/Technology'

const links = [
  { to: '/', label: 'Home' },
  { to: '/about', label: 'About' },
  { to: '/pipeline', label: 'Pipeline' },
  { to: '/demo', label: 'Interactive Demo' },
  { to: '/map', label: '2.5D Map' },
  { to: '/planning', label: 'Path Planning' },
  { to: '/technology', label: 'Technology' },
]

function Nav() {
  return (
    <header className="pointer-events-none fixed inset-x-0 top-0 z-50 flex justify-center p-4">
      <nav className="pointer-events-auto flex max-w-[1100px] flex-wrap items-center justify-between gap-3 rounded-full border border-orbit-line bg-orbit-bg/80 px-4 py-2 backdrop-blur-md">
        <NavLink to="/" className="font-mono text-sm tracking-[0.28em] text-orbit-cyan">
          ORBIT
        </NavLink>
        <ul className="flex flex-wrap items-center gap-1">
          {links.map((link) => (
            <li key={link.to}>
              <NavLink
                to={link.to}
                end={link.to === '/'}
                className={({ isActive }) =>
                  `rounded-full px-3 py-1.5 text-xs tracking-wide ${
                    isActive
                      ? 'bg-orbit-cyan/15 text-orbit-cyan'
                      : 'text-orbit-dim hover:text-orbit-text'
                  }`
                }
              >
                {link.label}
              </NavLink>
            </li>
          ))}
        </ul>
      </nav>
    </header>
  )
}

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
