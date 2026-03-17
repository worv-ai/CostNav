import numpy as np


def interpolate_trajectory(trajectory: list | np.ndarray, distance_threshold: float = 0.2) -> list[np.ndarray]:
    """Resample a 2-D (x, y) trajectory so that consecutive waypoints are no
    more than `distance_threshold` metres apart.
    """
    if len(trajectory) < 2:
        return [np.asarray(p, dtype=float) for p in trajectory]

    points = [np.asarray(p, dtype=float) for p in trajectory]

    interpolated: list[np.ndarray] = [points[0]]
    for p0, p1 in zip(points[:-1], points[1:]):
        seg_len = float(np.linalg.norm(p1[:2] - p0[:2]))
        n_steps = max(1, int(np.ceil(seg_len / distance_threshold)))

        for step in range(1, n_steps + 1):
            t = step / n_steps
            interp = (1.0 - t) * p0 + t * p1
            interpolated.append(interp)

    return interpolated
