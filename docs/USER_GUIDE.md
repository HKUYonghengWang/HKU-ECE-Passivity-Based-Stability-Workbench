# HKU ECE Passivity-Based Stability Workbench: User Guide

## Purpose

HKU ECE Passivity-Based Stability Workbench is a MATLAB/MATPOWER research prototype for passivity-based small-signal stability screening and virtual-shunt passivation of converter-dominated power systems.

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

- Case: `case39`
- Placement: all buses
- Display bus: `0`
- Reactance scale: `1.5`
- Resistance scale: `0.7`
- Grounding floor: `0`
- Existing uniform virtual conductance: `0`

## Reproduction Script

```matlab
addpath(genpath('path_to_matpower'));
addpath(genpath('path_to/HKU_ECE_PassivityWorkbench_v3.0'));
run('examples/run_case39_no_ground_demo.m')
```

## Citation

Please cite the associated passivity-based decentralized stability study and the software metadata in `CITATION.cff` when using this package in academic work.

## License

This software is released under the MIT License. See `LICENSE.txt`.
