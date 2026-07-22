# Payload-Aware Hybrid Sampling Bi-RRT for Rebar Transport

This repository contains the MATLAB code used for mobile-manipulator rebar transport experiments.

## Main scripts

- `PHS_Bi_RRT.m` - proposed payload-aware HS-Bi-RRT planner, configured as a reproducible single simulation run.
- `HS_Bi_RRT.m` - HS-Bi-RRT baseline, configured as a reproducible single simulation run.
- `Bi_RRT.m` - standard Bi-RRT baseline, configured as a reproducible single simulation run.
- `loadRebarTransportScenario.m` - robot, payload, and environment generator.

## Required assets

- `UR20.urdf`, `meshes/` - UR20 robot model and meshes.
- `gripper.urdf`, `gripper_meshes/` - gripper model and meshes.
- `computeRebarDeflection.m`, `deflection_predictor.m`, `deflection_nn_4x7_bs32_lr001.keras`, `defl_scaler_4x7_bs32_lr001.json` - static rebar deflection model support files.
- `findTransportHeightCircleChain.m`, `PriorityQueue.m` - payload-aware height/corridor selection helpers.

## MATLAB requirements

Developed with MATLAB R2025a/R2025b style APIs, Robotics System Toolbox, and Deep Learning Toolbox.

## Basic usage

Open MATLAB in this repository root, then run one of:

```matlab
PHS_Bi_RRT
HS_Bi_RRT
Bi_RRT
```

Each script saves run bundles under a local `seeds/` folder when execution reaches the save block and opens a replay figure for the generated trajectory.

## Changing the rebar length and environment scenario

The payload and test environment are configured in `loadRebarTransportScenario.m`.

### Rebar length

Near `4) Start configuration`, immediately below it in the `5) Environment (rebar)` block, change:

```matlab
rebarLength = 3;
```

The value is in metres. For example, use `rebarLength = 2;` for a 2 m rebar or `rebarLength = 4;` for a 4 m rebar. The selected value is propagated through `rebarMeta.LengthM`, so the planners and deflection calculation use the same length.

### Environment scenario

The alternative obstacle layouts are defined in Section 5 of `loadRebarTransportScenario.m`, including the S-corridor, walls-and-door, Cluttered Site 1, and Cluttered Site 2 blocks. Select a layout by commenting out the currently active scenario block and uncommenting every line of the desired block. Only one scenario block should be active at a time; otherwise obstacles from multiple scenarios will be added to `env` together.

For example, Cluttered Site 1 is active in the supplied configuration, while Cluttered Site 2 is commented out:

```matlab
% ===================== Cluttered Site 1 ======================
% Active code has no leading "%".

% ===================== Cluttered Site 2 ======================
% Commented code has a leading "%" and is inactive.
```

To use Cluttered Site 2, add `%` to the Cluttered Site 1 obstacle-definition lines and remove `%` from the Cluttered Site 2 obstacle-definition lines. Apply the same comment/uncomment procedure when selecting the S-corridor or walls-and-door blocks.

When switching to the S-corridor or door layout, also select its matching placement block in Section 7 (`below is code for S-corridor`, `below is code for door`, or `below is code for construction site`). Comment out the current placement block and uncomment the one corresponding to the selected environment.

## Optional swept-rebar envelope

All three planner scripts expose the same switch near the top of the file:

```matlab
useRebarEnvelope = false;
```

The sag-aware envelope is always displayed during trajectory replay, regardless of this setting. The switch controls only whether the envelope participates in Plan 4 collision checking. The default is `false` so the planner configuration remains consistent with the reported results; set it to `true` to include the envelope in planning/validation. The displayed envelope uses `1.10 x` the physical rebar length and the predicted deflection as its downward swept height. The selected setting and envelope dimensions are saved in each output bundle (`bundle.planner.P` and `bundle.rebarEnvelope`).

For `PHS_Bi_RRT.m` and `HS_Bi_RRT.m`, the envelope is checked while extending and validating the custom planner path. For the standard `Bi_RRT.m` baseline, MATLAB's `manipulatorRRT` continues to plan with the attached physical payload and the optional swept envelope is applied by the common Plan 4 task-rule validation.

## Notes

This is a research artifact extracted from the working project folder. Generated result files, figure exports, autosave files, and local MATLAB cache folders are excluded from version control.
