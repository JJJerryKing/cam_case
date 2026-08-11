# Axis_test Manual Set-Zero Design

## Goal

Add a manual calibration command to `Axis_test` that assigns `0.0` to the current NC position of the selected axis without changing either axis scaling factor.

## Scope

- Add one online Boolean command, `bSetZero`, to `Axis_test`.
- Add `MC_SetPosition` handling to `FB_AxisPositionTest`.
- Set only the current position to `0.0`; do not expose a configurable position input.
- Keep the currently selected axis (`GVL_Motion.gAxisCutter`) unchanged.
- Do not modify NC scaling, homing configuration, electronic-cam logic, or other POUs.

## Command Behaviour

`bSetZero` is a rising-edge command. The function block accepts it only when all of the following are true:

- the selected axis is powered;
- the selected axis reports `NotMoving`;
- no absolute-move command is active;
- no stop command is active;
- no previous set-position operation is busy.

When accepted, `MC_SetPosition` runs with:

- `Position := 0.0`;
- `Mode := FALSE` (absolute assignment);
- the selected `AXIS_REF`.

The operation changes the NC coordinate only and causes no physical motion.

## Interface and Diagnostics

`FB_AxisPositionTest` gains:

- input `bSetZero : BOOL`;
- output `bSetPositionDone : BOOL`;
- an internal rising-edge detector;
- an internal `MC_SetPosition` instance and execute latch.

The existing `bError` and `udiErrorID` outputs include `MC_SetPosition` errors. `lrActualPosition` continues to report `Axis.NcToPlc.ActPos` and should read approximately zero after a successful command.

## Verification

1. Confirm the selected axis is `GVL_Motion.gAxisCutter`.
2. Power the axis and leave it stopped.
3. Toggle `bSetZero` from `FALSE` to `TRUE` once.
4. Confirm `bSetPositionDone` becomes true and `lrActualPosition` becomes approximately `0.0` without physical movement.
5. Confirm set-zero is not accepted while an absolute move or stop command is active.
6. Confirm any `MC_SetPosition` error is exposed through `bError` and `udiErrorID`.

