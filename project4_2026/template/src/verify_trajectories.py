#!/usr/bin/env python3
"""Independently integrate exported controls with SciPy and verify all solutions."""
import argparse
import json
from pathlib import Path
import numpy as np
from scipy.integrate import solve_ivp


def angle_difference(a, b):
    return (a - b + np.pi) % (2 * np.pi) - np.pi


def verify(base, obstacles):
    info = json.loads(base.with_suffix('.json').read_text())
    states = np.loadtxt(base.with_suffix('.txt'), ndmin=2)
    data = np.loadtxt(base.with_name(base.name + '_controls.txt'), ndmin=2)
    pendulum = info['system'] == 'pendulum'
    n = 2 if pendulum else 4
    angle_index = 0 if pendulum else 2
    bound = info['torque'] if pendulum else 1.5
    assert data.shape[1] == (4 if pendulum else 7), base
    assert np.isfinite(data).all() and np.isfinite(states).all(), base
    assert np.all(np.abs(data[:, n:-1]) <= bound + 1e-12), base
    assert np.all(data[1:, -1] > 0) and np.allclose(data[0, n:], 0), base
    durations = data[1:, -1]
    assert np.allclose(durations / 0.05, np.round(durations / 0.05), atol=1e-10), base
    start = np.array([-np.pi / 2, 0] if pendulum else [1, 5, 0, 0])
    goal = np.array([np.pi / 2, 0] if pendulum else [11, 5, 0, 0])
    assert np.allclose(states[0], start) and np.allclose(data[0, :n], start), base
    assert np.allclose(states[-1], data[-1, :n]), base
    delta = states[-1] - goal
    delta[angle_index] = angle_difference(states[-1, angle_index], goal[angle_index])
    distance = (abs(delta[0]) + 0.5 * abs(delta[1]) if pendulum else
                np.linalg.norm(delta[:2]) + 0.5 * abs(delta[2]) + 0.5 * abs(delta[3]))
    assert distance < (0.1 if pendulum else 0.5), (base, distance)
    assert abs(distance - info['goal_distance']) < 1e-8, base

    def valid(q):
        if pendulum:
            return np.all(np.abs(q[:, 1]) <= 10 + 1e-8)
        if not (np.all(q[:, :2] >= -1e-8) and np.all(q[:, 0] <= 12 + 1e-8)
                and np.all(q[:, 1] <= 10 + 1e-8) and np.all(np.abs(q[:, 3]) <= 2 + 1e-8)):
            return False
        for x, y, w, h in obstacles:
            if np.any((q[:, 0] >= x) & (q[:, 0] <= x + w) &
                      (q[:, 1] >= y) & (q[:, 1] <= y + h)):
                return False
        return True

    assert valid(states) and valid(data[:, :n]), base
    assert np.all(np.abs(states[:, angle_index]) <= np.pi + 1e-12), base
    max_error = 0.0
    geometric_index = 1
    for i, row in enumerate(data[1:], start=1):
        u, duration = row[n:-1], row[-1]
        if pendulum:
            def ode(t, q):
                return [q[1], -9.81 * np.cos(q[0]) + u[0]]
        else:
            def ode(t, q):
                return [q[3] * np.cos(q[2]), q[3] * np.sin(q[2]), u[0], u[1]]
        times = np.linspace(0, duration, int(np.ceil(duration / 0.0025)) + 1)
        integration = solve_ivp(ode, [0, duration], data[i-1, :n], method='DOP853',
                                rtol=1e-11, atol=1e-12, t_eval=times, dense_output=True)
        assert integration.success and valid(integration.y.T), base
        error = integration.y[:, -1] - row[:n]
        error[angle_index] = angle_difference(integration.y[angle_index, -1], row[angle_index])
        max_error = max(max_error, float(np.max(np.abs(error))))
        # Check every plotted state against the independently integrated controls,
        # including states added by PathControl::asGeometric().
        count = int(round(duration / 0.05))
        reference = integration.sol(np.arange(1, count + 1) * (duration / count)).T
        exported = states[geometric_index:geometric_index + count]
        assert exported.shape == reference.shape, base
        differences = reference - exported
        differences[:, angle_index] = angle_difference(reference[:, angle_index], exported[:, angle_index])
        max_error = max(max_error, float(np.max(np.abs(differences))))
        geometric_index += count
    assert geometric_index == len(states), base
    assert max_error < 1e-5, (base, max_error)
    return dict(file=base.name, system=info['system'], planner=info['planner'],
                torque=info['torque'] if pendulum else None, seed=info['seed'],
                goal_distance=float(distance), independent_max_error=max_error, passed=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('output'))
    args = parser.parse_args()
    obstacles = np.loadtxt(args.output / 'street.txt', ndmin=2)
    records = []
    for metadata in sorted((args.output / 'trajectories').glob('*.json')):
        result = verify(metadata.with_suffix(''), obstacles)
        records.append(result)
        print(f"PASS {result['file']}: goal={result['goal_distance']:.6f}, "
              f"independent ODE error={result['independent_max_error']:.3g}")
    assert records, 'No trajectories to verify'
    (args.output / 'validation.json').write_text(json.dumps(records, indent=2) + '\n')
    print(f'{len(records)} exported control paths independently verified at <=2.5 ms intervals.')


if __name__ == '__main__':
    main()
