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

Run `./Project4Pendulum` or `./Project4Car`. Exercises 1 and 2 now support OMPL's control-based RRT and KPIECE1 for both systems. RG-RRT and benchmarking remain placeholders. See [`template/README`](template/README) for build, execution, validation, and visualization commands, and [`template/TEST_RESULTS.md`](template/TEST_RESULTS.md) for tested results. Generated trajectories, PNG figures, and a one-page checkpoint report are in [`template/output/`](template/output/).

To rebuild the handout, run `make` from `project4_2026/` with LaTeX installed.
