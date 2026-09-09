import { NavLink } from 'react-router-dom'

export const links = [
  { to: '/', label: 'Home' },
  { to: '/about', label: 'About' },
  { to: '/pipeline', label: 'Pipeline' },
  { to: '/demo', label: 'Interactive Demo' },
  { to: '/map', label: '2.5D Map' },
  { to: '/planning', label: 'Path Planning' },
  { to: '/technology', label: 'Technology' },
]

export default function Nav() {
  return (
    <header className="pointer-events-none fixed inset-x-0 top-0 z-50 p-3 md:flex md:justify-center md:p-4">
      <nav className="pointer-events-auto flex w-full max-w-[1120px] items-center gap-3 rounded-2xl border border-orbit-line/80 bg-orbit-bg/80 px-3 py-2 backdrop-blur-md md:rounded-full md:px-5">
        <NavLink
          to="/"
          className="shrink-0 font-mono text-[11px] tracking-[0.32em] text-orbit-cyan"
        >
          ORBIT
        </NavLink>
        <ul className="flex min-w-0 flex-1 items-center gap-0.5 overflow-x-auto pb-0.5 [-ms-overflow-style:none] [scrollbar-width:none] [&::-webkit-scrollbar]:hidden">
          {links.map((link) => (
            <li key={link.to} className="shrink-0">
              <NavLink
                to={link.to}
                end={link.to === '/'}
                className={({ isActive }) =>
                  `block rounded-full px-2.5 py-1.5 text-[11px] tracking-wide whitespace-nowrap md:px-3 ${
                    isActive
                      ? 'bg-orbit-cyan/12 text-orbit-cyan'
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
