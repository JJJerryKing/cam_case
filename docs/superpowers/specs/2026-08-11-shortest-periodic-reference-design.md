# Shortest Periodic Conveyor Reference Design

## Objective

Prevent a `bReference` command from moving the conveyor a long distance when its accumulated NC position is large, and eliminate repeated-reference coordinate drift. The conveyor shall move with `MC_MoveAbsolute` to the nearest physically equivalent reference point within the `63.7 mm` belt period, then set that point to NC zero.

The conveyor is allowed to move in either direction during reference. The shortest-path distance shall therefore be no greater than half a belt period, `31.85 mm`.

## Root cause addressed

The current implementation always commands absolute target `-0.6 mm`. After the first successful `MC_SetPosition(0.0)`, the active coordinate system has changed, so a later reference must use phase `0.0 mm`, not the original raw phase `-0.6 mm`. Returning to the fixed raw value causes unnecessary travel and introduces a `0.6 mm` shift on repeated reference operations.

## Runtime state

Add these internal variables to `FB_CamMachine`:

```iecst
bConveyorCoordinateZeroed : BOOL;
lrMasterReferencePhase : LREAL;
lrMasterReferenceTarget : LREAL;
```

`bConveyorCoordinateZeroed` is a normal non-retained function-block variable. It initializes to `FALSE` on a cold PLC/function-block initialization and remains unchanged when the application is merely disabled, reset after a motion fault, or re-enabled during the same runtime session.

After a successful conveyor `MC_SetPosition(0.0)`, set `bConveyorCoordinateZeroed := TRUE`. Do not clear it in `CAM_STATE_DISABLED`, `CAM_STATE_FAULT`, or the existing reset flow.

## Reference phase selection

When a reference command is accepted while both axes are stopped:

```iecst
IF bConveyorCoordinateZeroed THEN
    lrMasterReferencePhase := 0.0;
ELSE
    lrMasterReferencePhase := stRecipe.lrMasterReferenceRawPosition;
END_IF;
```

The first reference after cold initialization therefore uses raw phase `-0.6 mm`. Every later reference in the same runtime uses zero phase, preventing repeated `MC_SetPosition` offset accumulation.

## Shortest equivalent absolute target

Snapshot the reference target when the command is accepted:

```iecst
lrMasterReferenceTarget :=
    AxisConveyor.NcToPlc.SetPos
    + MODABS(
        lrMasterReferencePhase
        - AxisConveyor.NcToPlc.SetPos
        + stRecipe.lrObjectPitch / 2.0,
        stRecipe.lrObjectPitch)
    - stRecipe.lrObjectPitch / 2.0;
```

The calculation chooses the nearest member of the equivalent target family:

```text
ReferencePhase + k * ObjectPitch, where k is any integer
```

For pitch `63.7 mm`, the commanded travel magnitude is at most `31.85 mm`. At an exact half-period tie, the formula selects the negative-direction solution.

The existing conveyor reference block remains `MC_MoveAbsolute`, but its target changes to:

```iecst
Position := lrMasterReferenceTarget
```

Velocity, acceleration, deceleration, jerk, buffer mode, and axis remain unchanged.

## Move completion validation

In `CAM_STATE_POSITIONING_MASTER`, accept a completed move only when:

```iecst
fbMoveConveyorToReference.Done
AND AxisConveyor.Status.NotMoving
AND ABS(AxisConveyor.NcToPlc.SetPos - lrMasterReferenceTarget)
    <= stRecipe.lrPositionTolerance
AND ABS(AxisConveyor.NcToPlc.ActPos - lrMasterReferenceTarget)
    <= stRecipe.lrPositionTolerance
```

If `Done` and `NotMoving` are true but either position is outside tolerance, stop the execute request and raise `CAM_ERROR_AXIS_REFERENCE` with source `CAM_SOURCE_MASTER_AXIS`. This avoids waiting indefinitely and prevents zeroing at an inaccurate physical position.

Existing `Error` and unexpected `CommandAborted` handling remains in force.

## Zeroing and following-error handling

Initialize the conveyor set-position options every PLC scan before invoking `MC_SetPosition`:

```iecst
stSetPositionConveyorOptions.ClearPositionLag := TRUE;
```

The zeroing command remains:

```iecst
MC_SetPosition(Position := 0.0, Mode := FALSE)
```

In `CAM_STATE_REFERENCING_MASTER`, after `Done`, continue to Cutter positioning only when both conveyor positions satisfy:

```iecst
ABS(AxisConveyor.NcToPlc.SetPos) <= stRecipe.lrPositionTolerance
AND ABS(AxisConveyor.NcToPlc.ActPos) <= stRecipe.lrPositionTolerance
```

On success, set `bConveyorCoordinateZeroed := TRUE`, then start the existing Cutter `MC_MoveAbsolute` to `17 mm`. If zeroing reports `Done` but either value is outside tolerance, raise the master-axis reference fault.

The Cutter shall not use `MC_SetPosition`.

## Error and lifecycle behavior

- Reject a reference command unless both axes are stopped, as today.
- Recipe pitch must already pass the existing positive-value validation before production; target calculation additionally requires `stRecipe.lrObjectPitch > 0.0` at command acceptance.
- A non-positive pitch on reference raises `CAM_ERROR_RECIPE_PITCH` with source `CAM_SOURCE_RECIPE`.
- Move, tolerance, or zero-verification failures do not set `bConveyorCoordinateZeroed`.
- Automatic conveyor reference, Cutter positioning, recipe loading, relative cam coupling, and normal/fault stop behavior otherwise remain unchanged.

## Verification

Add an offline helper/test matrix for shortest periodic targeting, including:

```text
current=0.0,    phase=-0.6, pitch=63.7 -> target=-0.6
current=1000.0, phase=-0.6, pitch=63.7 -> travel magnitude <=31.85
current=40.0,   phase=0.0,  pitch=63.7 -> target=63.7
current=-40.0,  phase=0.0,  pitch=63.7 -> target=-63.7
```

PLC structural tests shall verify:

- The target is calculated once on reference acceptance and passed to `MC_MoveAbsolute`.
- Raw phase `-0.6` is used only before the runtime coordinate-zero flag is set.
- Repeated references use phase zero.
- Both `SetPos` and `ActPos` are checked before and after `MC_SetPosition`.
- `ClearPositionLag` is enabled.
- Cutter `MC_SetPosition` remains absent.
- All modified TwinCAT XML files parse successfully.

Final validation still requires compiling in TwinCAT XAE and testing first with the machine area clear.
