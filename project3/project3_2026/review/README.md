# October 14 verification

This review starts from commit `27efda8` on `winson-project3-review`. Its parent contains only an empty project placeholder, so Git cannot reconstruct a starter-code diff. Infrastructure is identified as supplied by the handout/source comments, not attributed to a student merely because it appears in their commit.

## Preserved artifacts

The original RTP and other C++ implementation files, scenes, robot assets, saved paths, screenshots, planner configurations, benchmark databases, CSVs, and original benchmark plots match their baseline Git objects. The Apple-only CMake visibility correction is the sole production build change. It was explicitly approved; no planning or collision-checking source was edited.

## Executed checks

`validation.json` records the saved-artifact audit, both platforms' smoke results, test summaries, and hashes of protected files. Builds and test outputs were isolated from this checkout.

| Check | macOS ARM64 | Ubuntu 24.04 ARM64 |
| --- | --- | --- |
| Fresh Python-extension build, OMPL 2.0.2 | Passed after visibility fix | Passed |
| Supplied support suite | 7 passed, 1 obsolete starter check skipped | 7 passed, 1 skipped |
| Synthetic benchmark/conversion suite | 4 passed | 4 passed |
| Standalone RTP lifecycle | 25 clear/reuse cycles and all edge cases passed | Same checks passed |
| Python RTP binding | Exact free-space and zero-time approximate cases passed | Same checks passed |
| Custom RTP smoke cases | All four exact; dense checks positive | All four exact; dense checks positive |
| Original saved-artifact audit | Passed | Passed |

The lifecycle check covers continuation, exact/approximate paths, start-equals-goal, invalid starts, tree labels/edges, solution-path independence after clearing, and balanced state allocation/free counts. It is not a general heap leak detector. Each seeded binding trial runs in a fresh Python process.

The saved-artifact audit uses independent NumPy FK, checked against the Pinocchio fixture, and at most 0.002 rad sampling for paths. All four saved custom RTP paths pass. The saved clutter RRT-Connect example has about 0.84 mm penetration between the prescribed validator samples; the compiled native checker independently confirms that collision. Positive sampled clearance is not a continuous-path guarantee.

Each of the four original benchmark databases has exactly 500 runs. Settings, resolution, query/trial counts, all summary statistics, and every number in the full report's 20-row comparison table were checked. Quality statistics use only exact runs; time includes unsuccessful runs. No original benchmark experiment was rerun. The synthetic benchmark suite uses small analytic fixtures, not the assignment experiment matrix.

## Remaining handoff

- Both partners: review contributions and report wording. Difficulty/hour estimates are retained from the original draft, not newly measured.
- Darshon: recover original native benchmark logs and custom-scene repeated-trial records from the experiment machine if available. The unsupported custom aggregates were removed, rather than reconstructed under different conditions.
- Winson: upload the existing `results/benchmark.db` to Planner Arena and download/inspect its plots as required by the handout. Existing report plots were generated locally; no prior Arena workflow was verified.
- Both: review the clean staged archives before personally submitting to Canvas. No upload, submission, commit, or push was performed during the review.

Both report PDFs were rebuilt with Tectonic and visually inspected at five pages each. Their LaTeX sources remain compatible with the documented pdfLaTeX workflow. The original screenshots are preserved; some wide gripper excursions already leave the frame, so figures are illustrations rather than full swept-volume validation.
