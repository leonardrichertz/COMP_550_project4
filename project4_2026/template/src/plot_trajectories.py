#!/usr/bin/env python3
"""Plot actual OMPL output; choose the latest successful run per configuration/seed."""
import argparse
import json
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

COLORS = {3: '#2878b5', 5: '#e28723', 10: '#8b4b9f'}
PLANNERS = ['RRT', 'KPIECE1']


def phase(ax, states, **kwargs):
    # SO(2) wraps at +/-pi. Break lines at that seam instead of drawing
    # a spurious horizontal segment across phase space.
    seam = np.where(np.abs(np.diff(states[:, 0])) > np.pi)[0] + 1
    first = True
    for part in np.split(states, seam):
        style = dict(kwargs)
        if not first:
            style.pop('label', None)
        ax.plot(part[:, 0], part[:, 1], **style)
        first = False


def phase_axes(ax):
    ax.scatter([-np.pi/2], [0], color='#16865b', marker='o', s=45, label='Start', zorder=5)
    ax.scatter([np.pi/2], [0], color='#be3740', marker='*', s=110, label='Goal', zorder=5)
    ax.set(xlim=(-np.pi, np.pi), ylim=(-10, 10), xlabel=r'$\theta$ (rad)', ylabel=r'$\omega$ (rad/s)')
    ax.set_xticks([-np.pi, -np.pi/2, 0, np.pi/2, np.pi],
                  [r'$-\pi$', r'$-\pi/2$', '0', r'$\pi/2$', r'$\pi$'])
    ax.grid(alpha=0.2)


def street_axes(ax, obstacles):
    for x, y, w, h in obstacles:
        ax.add_patch(Rectangle((x, y), w, h, facecolor='#646c79', edgecolor='#333a45'))
    ax.scatter([1], [5], color='#16865b', s=55, label='Start (at rest)', zorder=5)
    ax.scatter([11], [5], color='#be3740', marker='*', s=130, label='Goal (at rest)', zorder=5)
    ax.add_patch(plt.Circle((11, 5), 0.5, fill=False, linestyle=':', color='#be3740'))
    ax.set(xlim=(0, 12), ylim=(0, 10), xlabel='x', ylabel='y', aspect='equal')
    ax.grid(alpha=0.15)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=Path('output'))
    parser.add_argument('--seed', type=int, default=1)
    args = parser.parse_args()
    plots = args.output / 'plots'
    plots.mkdir(parents=True, exist_ok=True)
    paths = {}
    for file in sorted((args.output / 'trajectories').glob('*.json')):
        info = json.loads(file.read_text())
        if info['seed'] == args.seed and info['exact']:
            key = (info['system'], info['planner'], info['torque'] if info['system'] == 'pendulum' else None)
            paths[key] = (np.loadtxt(file.with_suffix('.txt')), info)
    expected = [('pendulum', p, t) for p in PLANNERS for t in [3, 5, 10]]
    expected += [('car', p, None) for p in PLANNERS]
    missing = set(expected) - paths.keys()
    if missing:
        raise SystemExit(f'Missing actual solutions for seed {args.seed}: {missing}')
    plt.rcParams.update({'font.size': 10, 'axes.spines.top': False, 'axes.spines.right': False})
    overview, axes = plt.subplots(2, 3, figsize=(12, 7), layout='constrained')
    comparison, comp_axes = plt.subplots(1, 2, figsize=(12, 4), layout='constrained')
    for row, planner in enumerate(PLANNERS):
        phase_axes(comp_axes[row])
        for col, torque in enumerate([3, 5, 10]):
            states, info = paths['pendulum', planner, torque]
            phase(comp_axes[row], states, color=COLORS[torque], lw=1.2, label=f'|tau| <= {torque}')
            ax = axes[row, col]
            phase_axes(ax)
            phase(ax, states, color=COLORS[torque], lw=1.3, label='OMPL trajectory')
            ax.set_title(f'{planner}, |tau| <= {torque}  /  {info["duration"]:.2f} s')
            ax.legend(fontsize=8, loc='lower left')
            fig, single = plt.subplots(figsize=(6, 4), layout='constrained')
            phase_axes(single)
            phase(single, states, color=COLORS[torque], lw=1.4, label='OMPL trajectory')
            single.set_title(f'Pendulum {planner}, |tau| <= {torque} (seed {args.seed})')
            single.legend(fontsize=8)
            fig.savefig(plots / f'pendulum_{planner}_tau{torque}.png', dpi=180)
            plt.close(fig)
        comp_axes[row].set_title(f'{planner}: torque comparison (seed {args.seed})')
        comp_axes[row].legend(fontsize=8, ncols=2)
    overview.savefig(plots / 'pendulum_all.png', dpi=180)
    comparison.savefig(plots / 'pendulum_comparison.png', dpi=180)
    plt.close(overview); plt.close(comparison)
    obstacles = np.loadtxt(args.output / 'street.txt', ndmin=2)
    fig, ax = plt.subplots(figsize=(7, 6), layout='constrained')
    street_axes(ax, obstacles)
    for planner, color in zip(PLANNERS, ['#2878b5', '#d05c28']):
        states, info = paths['car', planner, None]
        ax.plot(states[:, 0], states[:, 1], color=color, lw=1.6, label=planner)
        ax.scatter(*states[-1, :2], color=color, marker='x', s=45, zorder=5)
        single_fig, single = plt.subplots(figsize=(7, 6), layout='constrained')
        street_axes(single, obstacles)
        single.plot(states[:, 0], states[:, 1], color=color, lw=1.5, label=f'{planner} trajectory')
        # Heading arrows are taken from actual states, including reverse segments.
        sample = states[::max(1, len(states)//20)]
        single.quiver(sample[:, 0], sample[:, 1], np.cos(sample[:, 2]), np.sin(sample[:, 2]),
                      color=color, scale=25, width=0.003)
        single.scatter(*states[-1, :2], color=color, marker='x', s=45, zorder=5)
        single.set_title(f'Car {planner} (seed {args.seed}); duration {info["duration"]:.2f} s')
        single.legend(fontsize=9, loc='upper left')
        single_fig.savefig(plots / f'car_{planner}.png', dpi=180)
        plt.close(single_fig)
    ax.set_title(f'Car: same street and start-goal query (seed {args.seed})')
    ax.legend(fontsize=9, loc='upper left')
    fig.savefig(plots / 'car_comparison.png', dpi=180)
    plt.close(fig)
    print(f'Saved 11 PNG figures to {plots}')


if __name__ == '__main__':
    main()
