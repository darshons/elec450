# COMP/ELEC/MECH 450/550 - Project 4 (2026)

## Completed Part 1 checkpoint

- Submission ZIP: [Project4_Checkpoint_wl85_ds161.zip](Project4_Checkpoint_wl85_ds161.zip)
- One-page report: [project4_part1_writeup.pdf](project4_part1_writeup.pdf)
- Implementation, saved paths, verification and reproduction instructions:
  [project4_2026/README.md](project4_2026/README.md)

The checkpoint implements Exercises 1 and 2. The original handout and starter
below remain available; RG-RRT and benchmarking belong to the final submission.

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

Run `./Project4Pendulum` or `./Project4Car`. See [`template/README`](template/README) for implementation, visualization, and benchmark instructions. The starter compiles; the planning functions are student TODOs.

To rebuild the handout, run `make` from `project4_2026/` with LaTeX installed.
