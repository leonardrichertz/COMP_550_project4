#!/usr/bin/env python3
"""Assemble a one-page checkpoint report from verified, generated figures."""
import argparse
import json
from pathlib import Path
import textwrap
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('output'))
    args = parser.parse_args()
    validation = json.loads((args.output / 'validation.json').read_text())
    results = json.loads((args.output / 'run_results.json').read_text())
    assert len(results) >= 24 and all(r['passed'] for r in results)
    assert len(validation) >= 24 and all(r['passed'] for r in validation)
    plt.rcParams['font.family'] = 'DejaVu Sans'
    fig = plt.figure(figsize=(8.5, 11), facecolor='white')
    fig.text(0.06, 0.955, 'Project 4 | Exercises 1 and 2', fontsize=19, weight='bold', color='#213246')
    fig.text(0.06, 0.93, 'Control-based RRT and KPIECE1 - checkpoint', fontsize=11, color='#52647a')
    intro = ('We solve the assigned pendulum swing-up and car navigation problems with OMPL control planners.\n'
             'The pendulum uses SO(2) x [-10,10] and torque limits 3, 5, and 10. The car uses SE(2) x [-2,2],\n'
             'including reverse motion; angular velocity and acceleration are each bounded by +/-1.5.\n'
             'Both planners use identical problem definitions for each system. All figures use actual seed-1 solutions.')
    fig.text(0.06, 0.9, intro, fontsize=9, va='top', linespacing=1.5)
    ax = fig.add_axes([0.045, 0.485, 0.91, 0.335])
    ax.imshow(plt.imread(args.output / 'plots' / 'pendulum_all.png'))
    ax.axis('off')
    fig.text(0.06, 0.475, 'Figure 1. Phase-space trajectories. Times are simulated motion durations; angle seams are broken.',
             fontsize=8.2, color='#52647a')
    ax = fig.add_axes([0.045, 0.135, 0.51, 0.32])
    ax.imshow(plt.imread(args.output / 'plots' / 'car_comparison.png'))
    ax.axis('off')
    fig.text(0.575, 0.438, 'What the paths show', fontsize=11, weight='bold', color='#213246')
    paragraphs = [
        'At torque 3, repeated swings build energy. For these examples, RRT motion durations are 8.25, 4.75, and 3.00 s as the torque bound increases. KPIECE1 durations are 13.25, 9.00, and 4.40 s. These are illustrative paths, not optimized motions or a statistical planner comparison.',
        'Figure 2. The 12 x 10 street has two alternating rectangular barriers: (3.5,0,1,6) and (7,4,1,6). A point car starts at (1,5,0,0) and targets (11,5,0,0). Both paths go around the barriers. Crosses mark final positions; the dotted circle is a workspace envelope of the full goal region.',
        'Goal radii in the compound state metrics are 0.1 for the pendulum and 0.5 for the car. Exact solutions reach these neighborhoods; they do not imply exact arrival at zero velocity.'
    ]
    y = 0.414
    for paragraph in paragraphs:
        wrapped = textwrap.fill(paragraph, width=49)
        fig.text(0.575, y, wrapped, fontsize=8.1, va='top', linespacing=1.3)
        y -= (wrapped.count('\n') + 1) * 0.0125 + 0.014
    fig.text(0.06, 0.105, 'Verification', fontsize=11, weight='bold', color='#213246')
    max_error = max(r['independent_max_error'] for r in validation)
    evidence = (f'All 8 configurations succeeded for seeds 1, 2, and 3 (24/24). All three CTest model/collision tests passed.\n'
                f'Independent SciPy replay at <=2.5 ms checked dynamics, bounds, obstacles, and goal arrival; max local error {max_error:.2g}.\n'
                'Integration step: 5 ms; planning propagation step: 50 ms. RG-RRT and benchmarking remain unimplemented.')
    fig.text(0.06, 0.084, evidence, fontsize=8.2, va='top', linespacing=1.5)
    destination = args.output / 'pdf'
    destination.mkdir(exist_ok=True)
    fig.savefig(destination / 'checkpoint_report.pdf')
    plt.close(fig)
    print(destination / 'checkpoint_report.pdf')


if __name__ == '__main__':
    main()
