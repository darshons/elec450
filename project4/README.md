# COMP/ELEC/MECH 450/550 - Project 4 (2026)

## Completed final submission

- Final archive: [Project4_wl85_ds161.zip](Project4_wl85_ds161.zip)
- Final report: [project4_final_report.pdf](project4_2026/pdf/project4_final_report.pdf)
- Presentation (10 slides): [project4_slides.pdf](project4_2026/pdf/project4_slides.pdf)
- Code, saved paths, benchmarks and reproduction instructions:
  [project4_2026/README.md](project4_2026/README.md)
- Validation record: [REVIEW.md](project4_2026/REVIEW.md)

All four exercises are implemented. The original checkpoint archive and one-page
report are retained as `Project4_Checkpoint_wl85_ds161.zip` and
`project4_part1_writeup.pdf`. The presentation recording and Canvas upload must
be completed by the team.

Plan motions for a torque-controlled pendulum and a car with OMPL's control planners, implement RG-RRT in C++, and compare it with RRT and KPIECE.

The assignment is in [`pdf/project4.pdf`](pdf/project4.pdf), with editable LaTeX in [`src/project4.tex`](src/project4.tex). Deadlines are at **1pm**:

- Checkpoint (Exercises 1 and 2): **October 29, 2026**.
- Final code and report: **November 10, 2026**.
- Undergraduate presentation: **November 17, 2026**.

Use your Project 2 and Project 3 environment:

```bash
micromamba activate ompl_env
cd project4_2026
make
```

Run `./Project4Pendulum` or `./Project4Car`. See [`template/README`](template/README) for implementation, visualization, and benchmark instructions. The final implementation covers RRT, KPIECE1, RG-RRT, and benchmarking.

To rebuild the assignment handout, run `make` from `project4/` with LaTeX installed.
