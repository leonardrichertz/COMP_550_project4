#!/usr/bin/env python3
"""After running all eight menu combinations, run: python src/plot_trajectories.py"""
from pathlib import Path
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
import numpy as np

output = Path(__file__).resolve().parents[1] / 'output'
planners = ['RRT', 'KPIECE1']
torques = [3, 5, 10]
files = [output / f'pendulum_{p}_tau{t}.txt' for p in planners for t in torques]
files += [output / f'car_{p}.txt' for p in planners] + [output / 'street.txt']
missing = [str(p) for p in files if not p.exists()]
if missing:
    raise SystemExit('Run the missing planning combinations first:\n' + '\n'.join(missing))

plots = output / 'plots'
plots.mkdir(exist_ok=True)
fig, axes = plt.subplots(2, 3, figsize=(12, 7), layout='constrained')
for row, planner in enumerate(planners):
    for col, torque in enumerate(torques):
        states = np.loadtxt(output / f'pendulum_{planner}_tau{torque}.txt', ndmin=2)
        ax = axes[row, col]
        # Break the line at SO(2)'s angle seam; never interpolate across +/-pi.
        seams = np.where(np.abs(np.diff(states[:, 0])) > np.pi)[0] + 1
        for segment in np.split(states, seams):
            ax.plot(segment[:, 0], segment[:, 1], color=['#2878b5', '#e28723', '#8b4b9f'][col])
        ax.scatter(-np.pi / 2, 0, color='seagreen', label='Start', zorder=3)
        ax.scatter(np.pi / 2, 0, color='crimson', marker='*', s=90, label='Goal', zorder=3)
        ax.set(title=f'{planner}, torque bound {torque}', xlabel=r'$\theta$ (rad)',
               ylabel=r'$\omega$ (rad/s)', xlim=(-np.pi, np.pi), ylim=(-10, 10))
        ax.grid(alpha=0.2)
        ax.legend(fontsize=8)
fig.savefig(plots / 'pendulum_comparison.png', dpi=180)
plt.close(fig)

fig, ax = plt.subplots(figsize=(7, 6), layout='constrained')
# These rectangles are exported directly from the C++ makeStreet() data.
for x, y, width, height in np.loadtxt(output / 'street.txt', ndmin=2):
    ax.add_patch(Rectangle((x, y), width, height, facecolor='slategray', edgecolor='black'))
for planner in planners:
    states = np.loadtxt(output / f'car_{planner}.txt', ndmin=2)
    line, = ax.plot(states[:, 0], states[:, 1], label=planner)
    ax.scatter(*states[-1, :2], marker='x', color=line.get_color(), zorder=3)
ax.scatter(1, 5, color='seagreen', label='Start', zorder=3)
ax.scatter(11, 5, color='crimson', marker='*', s=100, label='Goal', zorder=3)
# The full goal condition also includes heading and speed errors.
ax.add_patch(plt.Circle((11, 5), 0.5, fill=False, linestyle=':', color='crimson'))
ax.set(title='Car: RRT and KPIECE1', xlabel='x', ylabel='y',
       xlim=(0, 12), ylim=(0, 10), aspect='equal')
ax.grid(alpha=0.2)
ax.legend()
fig.savefig(plots / 'car_comparison.png', dpi=180)
plt.close(fig)
print(f'Saved pendulum_comparison.png and car_comparison.png in {plots}')
