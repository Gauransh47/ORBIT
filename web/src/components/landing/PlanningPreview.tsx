import { Link } from 'react-router-dom'
import Section from '../Section'
import { RouteSketch } from './sketches'

const steps = ['Start', 'Destination', 'Locate path', 'Route if found', 'Path playback']

export default function PlanningPreview() {
  return (
    <Section kicker="PLANNING DEMONSTRATION" title="A* on exported ORBIT terrain, not a runtime planner.">
      <p className="max-w-2xl text-sm leading-7 text-orbit-dim">
        The Python pipeline does not compute a driving path. The /planning page
        runs a website-side A* demonstration on exported adaptive_cells and
        obstacle_cells. Geometric GROUND / MIXED / OBSTACLE is not a learned
        drivability model. The sketch below is still conceptual.
      </p>
      <div className="mt-8 overflow-hidden rounded-2xl border border-orbit-line bg-[#0a1018] p-4">
        <p className="font-mono text-[10px] tracking-widest text-orbit-dim">CONCEPTUAL ROUTE</p>
        <RouteSketch className="h-36 w-full" />
      </div>
      <ol className="mt-6 grid gap-3 sm:grid-cols-2 lg:grid-cols-5">
        {steps.map((step, i) => (
          <li
            key={step}
            className="flex min-h-[5.5rem] flex-col rounded-2xl border border-orbit-line bg-orbit-panel p-4"
          >
            <span className="font-mono text-[10px] text-orbit-cyan">
              {String(i + 1).padStart(2, '0')}
              {i < steps.length - 1 ? '  →' : ''}
            </span>
            <span className="mt-2 text-sm leading-5 text-orbit-text">{step}</span>
          </li>
        ))}
      </ol>
      <Link to="/planning" className="mt-8 inline-block text-sm text-orbit-cyan hover:underline">
        Open the planner →
      </Link>
    </Section>
  )
}
