# HKU ECE Passivity-Based Stability Workbench

A MATLAB/MATPOWER research prototype for decentralized passivity-based small-signal stability screening and local virtual-shunt passivation of converter-dominated power systems.

This release is aligned with the no-grounding current-balance model used in the IEEE 39-bus case study:

- no static load admittance `Y_L` in the dynamic network model;
- no bus `GS/BS` shunts and no artificial grounding floor by default;
- line charging capacitances are retained and lumped into the nodal capacitance matrix;
- constant-power loads from the power-flow solution are absorbed into bus-local converter equilibrium injections;
- the isolated network is passive but may contain a common-mode pair near `+/- j*w0`;
- local passivation gains follow the default rule `g_i = 1.02*g_min_i + 1e-4`.

## What the workbench does

1. Imports a MATPOWER power-flow case.
2. Builds the no-grounding current-balance dynamic network.
3. Constructs bus-local grid-forming converter port models.
4. Computes the local passivation threshold `g_min_i` at each bus.
5. Recommends virtual shunt gains `g_i` with a small robustness margin.
6. Validates current and recommended designs using frequency-domain and eigenvalue-domain diagnostics.
7. Provides a dashboard and bus-wise results table for quick engineering interpretation.

Closed-loop eigenvalues are used for validation only; they are not used to compute the local gains.

## Folder structure

```text
HKU_ECE_PassivityWorkbench_v3.0/
  launch_hku_passivity_workbench.m      # GUI launcher
  launch_HKU_PassivityWorkbench.m       # alternative GUI launcher
  src/
    HKU_ECE_PassivityWorkbenchApp.m     # MATLAB app
    hku_passivity_engine.m              # numerical backend
    hku_logo_clean_white.png            # GUI-ready logo
  demo/
    hku_case39_no_ground_demo.m         # paper-style case39 script
  examples/
    run_case39_no_ground_demo.m         # minimal demo runner
  docs/
    USER_GUIDE.md                       # quick user guide
  assets/
    hku_logo_clean_white.png
    hku_official_logo_english_cropped.png
  CITATION.cff
  LICENSE.txt
  VERSION.txt
```

## Requirements

- MATLAB R2022b or newer is recommended.
- MATPOWER must be installed and added to the MATLAB path.
- Control System Toolbox is required for `ss`, `freqresp`, and `lsim`.

## Quick start

```matlab
addpath(genpath('path_to_matpower'));
addpath('path_to/HKU_PassivityWorkbench_OpenSource_v3_Professional');
launch_hku_passivity_workbench
```

Recommended first run:

- case: `case39`
- placement: all buses
- display bus: `0` for automatic limiting-bus selection
- reactance scale: `1.5`
- resistance scale: `0.7`
- grounding floor: `0`
- existing uniform virtual conductance: `0`

Click **Run**.

## Paper-style reproduction script

```matlab
addpath(genpath('path_to_matpower'));
addpath('path_to/HKU_PassivityWorkbench_OpenSource_v3_Professional');
run('examples/run_case39_no_ground_demo.m')
```

The script exports a three-panel case-study figure and a numerical summary. In the expected IEEE 39-bus no-grounding run, the limiting bus is bus 39, the isolated network contains the common-mode pair near `+/- j 376.99 rad/s`, the current closed loop has one right-half-plane mode, and the recommended virtual shunts move the rightmost eigenvalues into the open left-half plane.

## Important modeling limitation

The default no-grounding implementation assumes all-bus converter placement, because loads are absorbed into local converter equilibrium injections. For partial converter placement studies, use a loaded-network formulation that explicitly retains static load admittances.

## License

This package is released under the MIT License. See `LICENSE.txt`.

## Citation

If you use this software in academic work, please cite the associated passivity-based decentralized stability study and the software package metadata in `CITATION.cff`.
