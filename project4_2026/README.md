# COMP/ELEC/MECH 450/550 - Project 4 (2026)

Plan motions for a torque-controlled pendulum and a car with OMPL's control planners, implement RG-RRT in C++, and compare it with RRT and KPIECE.

The assignment is in [`pdf/project4.pdf`](pdf/project4.pdf), with editable LaTeX in [`src/project4.tex`](src/project4.tex). Deadlines are at **1pm**:

- Checkpoint (Exercises 1 and 2): **October 29, 2026**.
- Final code and report: **November 10, 2026**.
- Undergraduate presentation: **November 17, 2026**.

Use your Project 2 and Project 3 environment:

```bash
micromamba activate ompl_env
cd project4_2026/template
make
```

Run `./Project4Pendulum` or `./Project4Car` and use the original menus. RRT and KPIECE1 are implemented; RG-RRT and benchmarking remain placeholders. Run the pendulum with each planner and torque bound (3, 5, 10), and run the car with each planner.

Successful runs export states to `template/output/`, with separate filenames for each planner and torque. The car also exports its obstacle rectangles to `output/street.txt`. From `template/`, generate the two comparison figures with:

```bash
python src/plot_trajectories.py
```

This requires NumPy and Matplotlib. The figures are saved in `template/output/plots/`; the required one-page report is `template/output/pdf/checkpoint_report.pdf`. Rerunning a configuration replaces its previous trajectory. Use `make clean` before packaging the submission.

To rebuild the handout, run `make` from `project4_2026/` with LaTeX installed.
