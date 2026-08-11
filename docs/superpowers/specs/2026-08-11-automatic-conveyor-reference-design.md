# Automatic Conveyor Reference Design

## Objective

Replace the manual conveyor positioning step in the physical electronic-cam commissioning flow. On a reference command, the PLC shall move the conveyor automatically to the trusted raw NC position `-0.6 mm`, redefine that position as `0.0 mm`, and then move the Cutter to its trusted absolute ready position `17.0 mm`.

The conveyor raw NC coordinate is assumed to remain trustworthy after power-up. No physical reference sensor is added by this change.

## State sequence

The reference sequence shall use three explicit states:

1. `CAM_STATE_POSITIONING_MASTER`
   - Entered from `CAM_STATE_IDLE` or `CAM_STATE_READY` on the rising edge of `bReference` while both axes are stopped.
   - Starts `MC_MoveAbsolute` on the conveyor with target `stRecipe.lrMasterReferenceRawPosition`, default `-0.6 mm`.
   - Uses `stRecipe.lrConveyorVelocity`, default `5 mm/s`.
   - Uses the existing master acceleration, deceleration, and jerk recipe values.
2. `CAM_STATE_REFERENCING_MASTER`
   - Entered only after the conveyor absolute move reports `Done`, the conveyor is stopped, and its set position is within `lrPositionTolerance` of the raw reference target.
   - Pulses `MC_SetPosition(Position := 0.0, Mode := FALSE)` on the conveyor.
3. `CAM_STATE_POSITIONING_SLAVE`
   - Entered after conveyor `MC_SetPosition` reports `Done`.
   - Moves the Cutter with `MC_MoveAbsolute` to `stRecipe.lrCutterReadyPosition`, default `17.0 mm`.
   - On completion within position tolerance, sets `bReferenced := TRUE` and returns to `CAM_STATE_IDLE`.

The Cutter shall never be passed to `MC_SetPosition`.

## Commands and interlocks

- A reference command is accepted only when both axes report `NotMoving`.
- The current conveyor position no longer needs to be near `-0.6 mm` before the reference command is accepted.
- Reference start clears `bReferenced` and `bRecipeLoaded`.
- `MC_SetPosition` is not allowed until the conveyor absolute move is complete, the axis is stopped, and the target-position tolerance check passes.
- Power remains enabled throughout all three reference states.
- Other reference, recipe, and start commands during the sequence are rejected using the existing command-not-allowed behavior.

## Motion behavior

The new conveyor `MC_MoveAbsolute` instance shall use:

- Position: `stRecipe.lrMasterReferenceRawPosition` (`-0.6 mm` by default)
- Velocity: `stRecipe.lrConveyorVelocity` (`5 mm/s` by default)
- Acceleration: `stRecipe.lrMasterAcceleration`
- Deceleration: `stRecipe.lrMasterDeceleration`
- Jerk: `stRecipe.lrMasterJerk`
- Buffer mode: `MC_Aborting`

No new recipe fields are required.

## Error handling

- Conveyor reference-move `Error` or unexpected `CommandAborted` shall request `CAM_ERROR_AXIS_REFERENCE` with source `CAM_SOURCE_MASTER_AXIS` and preserve the motion error ID.
- Conveyor `MC_SetPosition` errors retain the same application error and source mapping.
- Cutter positioning failures retain the existing slave-axis reference failure behavior.
- A failed sequence shall not set `bReferenced`.
- Fault reset behavior remains unchanged.

## Status and documentation

`E_CamMachineState` shall add `CAM_STATE_POSITIONING_MASTER` before `CAM_STATE_REFERENCING_MASTER`, making the online sequence explicit. The operational guide and state-machine diagram shall describe automatic conveyor positioning instead of manual movement.

## Verification

Static tests shall verify that:

- `FB_CamMachine` contains a conveyor `MC_MoveAbsolute` instance.
- Its target is `lrMasterReferenceRawPosition` and its velocity is `lrConveyorVelocity`.
- `CAM_STATE_POSITIONING_MASTER` precedes `CAM_STATE_REFERENCING_MASTER` in the reference flow.
- The precondition requiring the conveyor already to be at `-0.6 mm` has been removed.
- Conveyor `MC_SetPosition(0.0)` remains present.
- Cutter `MC_SetPosition` remains absent.
- All modified TwinCAT XML files remain parseable.

The final implementation must also be compiled in TwinCAT XAE because the current local environment does not expose a TwinCAT-compatible build executable.
