import open3d as o3d
import numpy as np


# ============================================================
# ORBIT v0.2
# 100m Synthetic LiDAR Environment
# ============================================================

# Semantic classes
DRIVABLE = 0
STATIC = 1
DYNAMIC = 2


def color_point_cloud(point_cloud, color):
    """Assign one RGB color to every point."""

    points = np.asarray(point_cloud.points)

    colors = np.tile(
        color,
        (len(points), 1)
    )

    point_cloud.colors = o3d.utility.Vector3dVector(
        colors
    )

    return point_cloud


def create_ground(size=120, resolution=0.15):
    """
    Create a large terrain surface.

    The terrain is slightly uneven to make the
    elevation map more realistic.
    """

    x = np.arange(
        -10,
        size,
        resolution
    )

    y = np.arange(
        -60,
        60,
        resolution
    )

    xx, yy = np.meshgrid(x, y)

    # Slight terrain variation
    zz = (
        0.10 * np.sin(xx / 12.0)
        + 0.06 * np.sin(yy / 9.0)
    )

    points = np.column_stack(
        (
            xx.ravel(),
            yy.ravel(),
            zz.ravel()
        )
    )

    ground = o3d.geometry.PointCloud()

    ground.points = o3d.utility.Vector3dVector(
        points
    )

    return color_point_cloud(
        ground,
        [0.2, 0.8, 0.2]
    )


def create_box(
    width,
    depth,
    height,
    position,
    points=1500
):
    """
    Create a box and sample its surfaces.
    """

    mesh = o3d.geometry.TriangleMesh.create_box(
        width=width,
        height=depth,
        depth=height
    )

    mesh.translate(position)

    cloud = mesh.sample_points_uniformly(
        number_of_points=points
    )

    return cloud


def create_wall(x, y, width=0.4, length=8, height=3):
    """
    Create a wall.

    The wall's bottom sits on the terrain plane.
    """

    wall = create_box(
        width=width,
        depth=length,
        height=height,
        position=(x, y, 0)
    )

    return color_point_cloud(
        wall,
        [0.9, 0.1, 0.1]
    )


def create_pole(x, y, height=3):
    """
    Create a vertical pole.

    The cylinder is centered around its origin,
    so we place its center at height / 2.
    """

    mesh = o3d.geometry.TriangleMesh.create_cylinder(
        radius=0.18,
        height=height
    )

    mesh.translate(
        (
            x,
            y,
            height / 2
        )
    )

    cloud = mesh.sample_points_uniformly(
        number_of_points=1000
    )

    return color_point_cloud(
        cloud,
        [1.0, 0.7, 0.1]
    )


def create_vehicle(x, y):
    """
    Create a simplified vehicle.
    """

    body = create_box(
        width=2.0,
        depth=4.0,
        height=1.2,
        position=(x, y, 0),
        points=1200
    )

    roof = create_box(
        width=1.6,
        depth=2.0,
        height=0.7,
        position=(x + 0.2, y + 1.0, 1.2),
        points=800
    )

    vehicle = body + roof

    return color_point_cloud(
        vehicle,
        [0.1, 0.3, 1.0]
    )


def create_noise(number_of_points=2000):
    """
    Add sparse random outliers.
    """

    points = np.random.uniform(
        low=[-10, -60, 0],
        high=[110, 60, 5],
        size=(number_of_points, 3)
    )

    cloud = o3d.geometry.PointCloud()

    cloud.points = o3d.utility.Vector3dVector(
        points
    )

    return color_point_cloud(
        cloud,
        [0.5, 0.5, 0.5]
    )


def main():

    print("=" * 60)
    print("ORBIT - 100m SYNTHETIC ENVIRONMENT")
    print("=" * 60)

    np.random.seed(42)

    # --------------------------------------------------------
    # Ground
    # --------------------------------------------------------

    ground = create_ground()

    # --------------------------------------------------------
    # Objects at different distances
    # --------------------------------------------------------

    pole_near = create_pole(
        5,
        2
    )

    vehicle_12m = create_vehicle(
        12,
        -4
    )

    wall_20m = create_wall(
        20,
        3,
        length=10
    )

    pole_35m = create_pole(
        35,
        -6
    )

    vehicle_60m = create_vehicle(
        60,
        8
    )

    wall_90m = create_wall(
        90,
        -5,
        length=12
    )

    # --------------------------------------------------------
    # Combine environment
    # --------------------------------------------------------

    scene = (
        ground
        + pole_near
        + vehicle_12m
        + wall_20m
        + pole_35m
        + vehicle_60m
        + wall_90m
    )

    # Add noise
    noise = create_noise()

    scene += noise

    # --------------------------------------------------------
    # Save
    # --------------------------------------------------------

    output_file = (
        "data/synthetic_scene.ply"
    )

    o3d.io.write_point_cloud(
        output_file,
        scene
    )

    print(
        f"\nGenerated "
        f"{len(scene.points):,} points."
    )

    print(
        f"Saved to: {output_file}"
    )

    # --------------------------------------------------------
    # Visualize
    # --------------------------------------------------------

    o3d.visualization.draw_geometries(
        [scene],
        window_name=(
            "ORBIT - 100m Synthetic Environment"
        )
    )


if __name__ == "__main__":
    main()