import PageShell from '../components/PageShell'

export default function About() {
  return (
    <PageShell kicker="ABOUT ORBIT" title="A geometric mapping prototype">
      <p>
        ORBIT is a research prototype for adaptive variable-resolution 2.5D
        LiDAR mapping. It runs on CPU-side Python: RANSAC ground, an adaptive
        terrain grid, geometric object proposals, multi-frame tracking, and a
        world model in <strong className="text-orbit-text">LiDAR frame 0</strong>.
      </p>
      <p>
        Dataset backends today are SemanticKITTI / KITTI and nuScenes v1.0-mini
        (LIDAR_TOP keyframes). The website is a separate presentation layer. It
        will consume exported JSON from the same pipeline — it does not replace
        the Matplotlib dashboard.
      </p>
      <p>
        What ORBIT does <em>not</em> currently do: learned semantic segmentation,
        PointNet++, or sparse CNN inference. Those remain planned future work
        toward the intended DRDO/iDEX system. The website can run a browser A*
        demonstration on exported occupancy; that is not the Python ORBIT
        runtime planner.
      </p>
    </PageShell>
  )
}
