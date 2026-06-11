#!/usr/bin/env python3

import math
from dataclasses import dataclass
from collections import deque

import numpy as np
from scipy.ndimage import binary_erosion, label

from delta_control.kinematics import DeltaKinematics2


@dataclass
class WorkspaceMesh:
    x: np.ndarray
    y: np.ndarray
    z: np.ndarray
    volume_x: np.ndarray
    volume_y: np.ndarray
    volume_z: np.ndarray
    scan_path: np.ndarray
    scan_voxel_count: int
    volume_voxel_count: int
    bounds_mm: tuple[float, float, float, float, float, float]
    grid_step_mm: float


def compress_collinear_path(path, tolerance=1e-9):
    """Merge straight voxel runs without changing their geometric route."""
    points = np.asarray(path, dtype=float)
    if len(points) <= 2:
        return points.copy()

    compressed = [points[0]]
    previous_direction = None

    for index in range(1, len(points)):
        delta = points[index] - points[index - 1]
        length = float(np.linalg.norm(delta))
        direction = None if length <= tolerance else delta / length

        if (
            previous_direction is not None
            and (
                direction is None
                or not np.allclose(
                    direction,
                    previous_direction,
                    atol=tolerance,
                    rtol=0.0,
                )
            )
        ):
            compressed.append(points[index - 1])

        if direction is not None:
            previous_direction = direction

    if not np.allclose(compressed[-1], points[-1], atol=tolerance, rtol=0.0):
        compressed.append(points[-1])
    return np.asarray(compressed)


def generate_limited_joint_segment(
    solver,
    p0,
    p1,
    duration,
    n_waypoints,
    preferred_start=None,
    max_joint_speed=math.radians(60.0),
    max_joint_acceleration=math.radians(150.0),
):
    """Stretch a Cartesian segment until joint speed and acceleration fit."""
    adjusted_duration = float(duration)
    for _ in range(4):
        segment = solver.generate_joint_trajectory(
            p0=p0,
            p1=p1,
            duration=adjusted_duration,
            n_waypoints=n_waypoints,
            preferred_start=preferred_start,
        )
        peak_speed = max(
            abs(value)
            for point in segment
            for value in point["q_dot"]
        )
        peak_acceleration = max(
            abs(value)
            for point in segment
            for value in point["q_ddot"]
        )
        scale = max(
            1.0,
            peak_speed / max_joint_speed,
            math.sqrt(peak_acceleration / max_joint_acceleration),
        )
        if scale <= 1.001:
            return segment
        adjusted_duration *= 1.02 * scale

    return segment


def calculate_workspace_surface(
    solver: DeltaKinematics2,
    grid_step: float = 0.004,
    scan_divisions: int = 12,
    full_scan: bool = True,
) -> WorkspaceMesh:
    """Return surface voxels of the IK workspace connected to HOME."""
    g = solver.g
    x_values = np.arange(-0.35, 0.35 + grid_step * 0.5, grid_step)
    y_values = np.arange(-0.33, 0.33 + grid_step * 0.5, grid_step)
    z_values = np.arange(-0.50, -0.18 + grid_step * 0.5, grid_step)
    x_grid, y_grid = np.meshgrid(x_values, y_values, indexing="xy")

    valid = np.zeros(
        (len(z_values), len(y_values), len(x_values)),
        dtype=bool,
    )

    for z_index, z_world in enumerate(z_values):
        z = z_world - g.motor_plane_z
        layer = np.ones(x_grid.shape, dtype=bool)

        for phi in solver.phis:
            ux = np.cos(phi)
            uy = np.sin(phi)
            vx = -np.sin(phi)
            vy = np.cos(phi)

            s = ux * x_grid + uy * y_grid - g.d_base
            h = vx * x_grid + vy * y_grid
            a = -s
            c = (
                g.l2 * g.l2
                - s * s
                - h * h
                - z * z
                - g.l1 * g.l1
            ) / (2.0 * g.l1)
            radius = np.sqrt(a * a + z * z)
            layer &= (radius > 1e-12) & (np.abs(c / radius) <= 1.0)

        valid[z_index] = layer

    labels, _ = label(valid)
    home_index = (
        int(np.argmin(np.abs(z_values - g.home_z))),
        int(np.argmin(np.abs(y_values - g.home_y))),
        int(np.argmin(np.abs(x_values - g.home_x))),
    )
    home_label = labels[home_index]
    if home_label == 0:
        raise RuntimeError("HOME is not inside the calculated workspace grid.")

    connected = labels == home_label
    surface = connected & ~binary_erosion(connected)
    z_index, y_index, x_index = np.where(surface)

    inside_z, inside_y, inside_x = np.where(connected)
    volume_sample = connected[::2, ::2, ::2]
    sampled_x = x_values[::2]
    sampled_y = y_values[::2]
    sampled_z = z_values[::2]
    for iz, iy, ix in np.argwhere(volume_sample):
        if not solver.is_within_motor_limits(
            float(sampled_x[ix]),
            float(sampled_y[iy]),
            float(sampled_z[iz]),
        ):
            volume_sample[iz, iy, ix] = False
    volume_z_index, volume_y_index, volume_x_index = np.where(volume_sample)
    bounds_mm = (
        float(x_values[inside_x].min() * 1000.0),
        float(x_values[inside_x].max() * 1000.0),
        float(y_values[inside_y].min() * 1000.0),
        float(y_values[inside_y].max() * 1000.0),
        float(z_values[inside_z].min() * 1000.0),
        float(z_values[inside_z].max() * 1000.0),
    )

    scan_path = _create_volume_scan_path(
        volume_sample,
        sampled_x,
        sampled_y,
        sampled_z,
        solver,
        scan_divisions,
        full_scan,
    )
    volume_voxel_count = int(np.count_nonzero(volume_sample))

    return WorkspaceMesh(
        x=x_values[x_index] * 1000.0,
        y=y_values[y_index] * 1000.0,
        z=z_values[z_index] * 1000.0,
        volume_x=sampled_x[volume_x_index] * 1000.0,
        volume_y=sampled_y[volume_y_index] * 1000.0,
        volume_z=sampled_z[volume_z_index] * 1000.0,
        scan_path=scan_path,
        scan_voxel_count=volume_voxel_count if full_scan else len(scan_path) - 3,
        volume_voxel_count=volume_voxel_count,
        bounds_mm=bounds_mm,
        grid_step_mm=grid_step * 1000.0,
    )


def _point_is_reachable(solver, x, y, z_world):
    g = solver.g
    z = z_world - g.motor_plane_z
    for phi in solver.phis:
        ux = np.cos(phi)
        uy = np.sin(phi)
        vx = -np.sin(phi)
        vy = np.cos(phi)
        s = ux * x + uy * y - g.d_base
        h = vx * x + vy * y
        a = -s
        c = (
            g.l2 * g.l2
            - s * s
            - h * h
            - z * z
            - g.l1 * g.l1
        ) / (2.0 * g.l1)
        radius = np.hypot(a, z)
        if radius <= 1e-12 or abs(c / radius) > 1.0:
            return False
    return True


def _create_volume_scan_path(
    volume,
    x_values,
    y_values,
    z_values,
    solver,
    divisions,
    full_scan,
):
    """Order points from the same voxel volume used by the 3D plot."""
    home = np.array(
        [solver.g.home_x, solver.g.home_y, solver.g.home_z],
        dtype=float,
    )
    valid = volume.copy() if full_scan else binary_erosion(volume, iterations=1)
    if not valid.any():
        valid = volume.copy()

    home_index = (
        int(np.argmin(np.abs(z_values - home[2]))),
        int(np.argmin(np.abs(y_values - home[1]))),
        int(np.argmin(np.abs(x_values - home[0]))),
    )
    if full_scan:
        connectivity = np.ones((3, 3, 3), dtype=int)
    else:
        connectivity = np.zeros((3, 3, 3), dtype=int)
        connectivity[1, 1, 1] = 1
        connectivity[0, 1, 1] = 1
        connectivity[2, 1, 1] = 1
        connectivity[1, 0, 1] = 1
        connectivity[1, 2, 1] = 1
        connectivity[1, 1, 0] = 1
        connectivity[1, 1, 2] = 1
    labels, _ = label(valid, structure=connectivity)
    home_label = labels[home_index]
    if home_label == 0:
        nearest = np.argwhere(valid)
        if not len(nearest):
            raise RuntimeError("No reachable workspace cells were found.")
        home_index = tuple(
            nearest[
                np.argmin(
                    np.sum((nearest - np.asarray(home_index)) ** 2, axis=1)
                )
            ]
        )
        home_label = labels[home_index]

    connected = labels == home_label
    if full_scan:
        path_indices = _walk_all_connected_voxels(connected, home_index)
        points = [home]
        points.extend(
            np.array([x_values[ix], y_values[iy], z_values[iz]])
            for iz, iy, ix in path_indices
        )
        points.append(home)
        return np.asarray(points)

    divisions = max(5, min(24, int(divisions)))
    z_selection = np.unique(
        np.linspace(0, len(z_values) - 1, divisions, dtype=int)
    )
    y_selection = np.unique(
        np.linspace(0, len(y_values) - 1, divisions, dtype=int)
    )
    x_selection = np.unique(
        np.linspace(0, len(x_values) - 1, divisions, dtype=int)
    )

    cells = []
    for layer_index, iz in enumerate(z_selection):
        y_order = (
            y_selection
            if layer_index % 2 == 0
            else y_selection[::-1]
        )
        for row_index, iy in enumerate(y_order):
            x_indices = np.asarray(
                [ix for ix in x_selection if connected[iz, iy, ix]],
                dtype=int,
            )
            if not len(x_indices):
                continue
            if (layer_index + row_index) % 2:
                x_indices = x_indices[::-1]
            cells.extend((iz, iy, int(ix)) for ix in x_indices)

    path_indices = [home_index]
    for target in cells:
        if target == path_indices[-1]:
            continue
        path_indices.extend(
            _grid_shortest_path(connected, path_indices[-1], target)[1:]
        )
    path_indices.extend(
        _grid_shortest_path(connected, path_indices[-1], home_index)[1:]
    )

    points = [home]
    points.extend(
        np.array([x_values[ix], y_values[iy], z_values[iz]])
        for iz, iy, ix in path_indices
    )
    points.append(home)
    return np.asarray(points)


def _walk_all_connected_voxels(valid, start):
    """Return a continuous depth-first walk that visits every valid voxel."""
    visited = np.zeros(valid.shape, dtype=bool)
    visited[start] = True
    path = [start]
    stack = [(start, 0)]

    while stack:
        current, neighbor_index = stack[-1]
        neighbors = _ordered_neighbors(current)

        if neighbor_index >= len(neighbors):
            stack.pop()
            if stack:
                path.append(stack[-1][0])
            continue

        stack[-1] = (current, neighbor_index + 1)
        nxt = neighbors[neighbor_index]
        if any(nxt[i] < 0 or nxt[i] >= valid.shape[i] for i in range(3)):
            continue
        if not valid[nxt] or visited[nxt]:
            continue

        visited[nxt] = True
        path.append(nxt)
        stack.append((nxt, 0))

    if int(np.count_nonzero(visited)) != int(np.count_nonzero(valid)):
        raise RuntimeError("The full workspace voxel walk is not connected.")
    return path


def _ordered_neighbors(cell):
    iz, iy, _ = cell
    x_direction = 1 if (iz + iy) % 2 == 0 else -1
    y_direction = 1 if iz % 2 == 0 else -1
    axis_neighbors = [
        (cell[0], cell[1], cell[2] + x_direction),
        (cell[0], cell[1] + y_direction, cell[2]),
        (cell[0] + 1, cell[1], cell[2]),
        (cell[0], cell[1], cell[2] - x_direction),
        (cell[0], cell[1] - y_direction, cell[2]),
        (cell[0] - 1, cell[1], cell[2]),
    ]
    axis_deltas = {
        (neighbor[0] - cell[0], neighbor[1] - cell[1], neighbor[2] - cell[2])
        for neighbor in axis_neighbors
    }
    diagonal_neighbors = []
    for dz in (-1, 0, 1):
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                delta = (dz, dy, dx)
                if delta == (0, 0, 0) or delta in axis_deltas:
                    continue
                diagonal_neighbors.append(
                    (cell[0] + dz, cell[1] + dy, cell[2] + dx)
                )
    return axis_neighbors + diagonal_neighbors


def _grid_shortest_path(valid, start, goal):
    if start == goal:
        return [start]

    queue = deque([start])
    previous = {start: None}
    shape = valid.shape
    neighbors = (
        (-1, 0, 0),
        (1, 0, 0),
        (0, -1, 0),
        (0, 1, 0),
        (0, 0, -1),
        (0, 0, 1),
    )

    while queue:
        current = queue.popleft()
        for delta in neighbors:
            nxt = tuple(current[i] + delta[i] for i in range(3))
            if any(nxt[i] < 0 or nxt[i] >= shape[i] for i in range(3)):
                continue
            if not valid[nxt] or nxt in previous:
                continue
            previous[nxt] = current
            if nxt == goal:
                path = [goal]
                while path[-1] != start:
                    path.append(previous[path[-1]])
                return path[::-1]
            queue.append(nxt)

    raise RuntimeError("Unable to connect two neighboring workspace cells.")
