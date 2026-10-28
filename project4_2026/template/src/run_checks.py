#!/usr/bin/env python3
"""Exercise 1/2 functional smoke runs, not the Exercise 4 benchmark."""
import argparse
import json
from pathlib import Path
import subprocess


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seeds', nargs='+', type=int, default=[1, 2, 3])
    parser.add_argument('--time', type=float, default=30)
    parser.add_argument('--output', type=Path, default=Path('output'))
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    logs = args.output / 'logs'
    logs.mkdir(exist_ok=True)
    results = []
    cases = [('pendulum', planner, torque) for planner in ['RRT', 'KPIECE1'] for torque in [3, 5, 10]]
    cases += [('car', planner, None) for planner in ['RRT', 'KPIECE1']]
    for system, planner, torque in cases:
        for seed in args.seeds:
            label = f'{system}_{planner}' + (f'_tau{torque}' if torque else '') + f'_seed{seed}'
            command = ['./Project4Pendulum' if system == 'pendulum' else './Project4Car',
                       '--planner', planner, '--seed', str(seed), '--time', str(args.time),
                       '--output', str(args.output)]
            if torque:
                command += ['--torque', str(torque)]
            try:
                run = subprocess.run(command, text=True, capture_output=True, timeout=args.time + 20)
                (logs / f'{label}.log').write_text(run.stdout + run.stderr)
                passed = run.returncode == 0
            except subprocess.TimeoutExpired as error:
                (logs / f'{label}.log').write_text(f'Process timeout: {error}')
                passed = False
            results.append(dict(system=system, planner=planner, torque=torque, seed=seed, passed=passed))
            (args.output / 'run_results.json').write_text(json.dumps(results, indent=2) + '\n')
            print(f'{label}: {"PASS" if passed else "FAIL"}', flush=True)
    raise SystemExit(0 if all(row['passed'] for row in results) else 1)


if __name__ == '__main__':
    main()
