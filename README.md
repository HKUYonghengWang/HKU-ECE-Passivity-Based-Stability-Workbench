# HKU ECE Passivity-Based Stability Workbench

A MATLAB/MATPOWER research prototype for passivity-based small-signal stability screening and virtual-shunt passivation of converter-dominated power systems.

The current release provides:

- a graphical workbench for MATPOWER case import and bus-wise stability screening;
- local passivation-threshold computation for grid-forming converter ports;
- virtual-shunt gain recommendations with a small robustness margin;
- frequency-domain and eigenvalue-domain validation tools;
- a reproducible IEEE 39-bus no-grounding case study.

Closed-loop eigenvalues are used for validation only; they are not used to compute the local passivation gains.

## Scope

Version 3.0 implements the no-grounding current-balance model used in the IEEE 39-bus case study. Static load admittances are not retained in the dynamic network model; constant-power loads from the power-flow solution are absorbed into bus-local converter equilibrium injections. The default release is therefore intended for all-bus converter placement studies.

For partial converter placement studies, use a loaded-network formulation that explicitly retains static load admittances.

## Requirements

- MATLAB R2022b or newer is recommended.
- MATPOWER must be installed and added to the MATLAB path.
- Control System Toolbox is required for `ss`, `freqresp`, and `lsim`.

## Quick Start

```matlab
addpath(genpath('path_to_matpower'));
addpath(genpath('path_to/HKU_ECE_PassivityWorkbench_v3.0'));
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

## Reproducing the Case Study

```matlab
addpath(genpath('path_to_matpower'));
addpath(genpath('path_to/HKU_ECE_PassivityWorkbench_v3.0'));
run('examples/run_case39_no_ground_demo.m')
```

The script exports a three-panel case-study figure and a numerical summary.

## Repository Layout

```text
launch_hku_passivity_workbench.m      GUI launcher
src/HKU_ECE_PassivityWorkbenchApp.m   MATLAB app
src/hku_passivity_engine.m            numerical backend
demo/hku_case39_no_ground_demo.m      paper-style IEEE 39-bus demo
examples/run_case39_no_ground_demo.m  minimal demo runner
docs/USER_GUIDE.md                    user guide
```

## Citation

If you use this software in academic work, please cite the associated passivity-based decentralized stability study and the software metadata in `CITATION.cff`.

## License

This package is released under the MIT License. See `LICENSE.txt`.
