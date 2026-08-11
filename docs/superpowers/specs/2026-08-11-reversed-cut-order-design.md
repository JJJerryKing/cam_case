# Reversed Two-Edge Cut Order Design

## Objective

Change the physical electronic-cam profile so the Cutter cuts the simulated object's first edge backward and its second edge forward. With conveyor velocity `5 mm/s` and relative cutting velocity `3 mm/s`, the Cutter NC velocities shall be `-2 mm/s` for the first edge and `-8 mm/s` for the second edge.

The Cutter remains at trusted absolute ready position `17 mm` before coupling. The conveyor cycle remains `63.7 mm`, and the return remains `17–27 mm` (two seconds at conveyor speed `5 mm/s`).

## Recipe and derived geometry

Change the default recipe distances to:

```text
lrInterEdgeTransitionDistance = 4.0 mm
lrExitTransitionDistance = 2.0 mm
```

Keep `lrFirstEdgeStart = 1.0 mm`, the derived cut-master distance `5.0 mm`, `lrReturnStartPhase = 17.0 mm`, and `lrReturnEndPhase = 27.0 mm`.

Swap the derived cut slopes:

```text
FirstSlope  = -(1 - CutVelocity / ConveyorVelocity) = -0.4
SecondSlope = -(1 + CutVelocity / ConveyorVelocity) = -1.6
```

All derived Y positions continue to use the existing endpoint formulas. With the default recipe they produce the approved eight-point profile below.

## Approved cam table

| Point | Master X (mm) | Cam Y (mm) | Cutter absolute (mm) | Y' | Function to next point |
|---:|---:|---:|---:|---:|---|
| 1 | 0.0 | 0.0 | 17.0 | 0.0 | `MOTIONFUNCTYPE_POLYNOM5_MM` |
| 2 | 1.0 | -1.0 | 16.0 | -0.4 | `MOTIONFUNCTYPE_POLYNOM1` |
| 3 | 6.0 | -3.0 | 14.0 | -0.4 | `MOTIONFUNCTYPE_POLYNOM5_MM` |
| 4 | 10.0 | -7.0 | 10.0 | -1.6 | `MOTIONFUNCTYPE_POLYNOM1` |
| 5 | 15.0 | -15.0 | 2.0 | -1.6 | `MOTIONFUNCTYPE_POLYNOM5_MM` |
| 6 | 17.0 | -16.6 | 0.4 | 0.0 | `MOTIONFUNCTYPE_POLYNOM5_MM` |
| 7 | 27.0 | 0.0 | 17.0 | 0.0 | `MOTIONFUNCTYPE_POLYNOM1` |
| 8 | 63.7 | 0.0 | 17.0 | 0.0 | `MOTIONFUNCTYPE_NOTDEF` |

The accepted minimum Cutter absolute position is `0.4 mm`.

## Process phases

```text
0–1 mm      Smooth entry
1–6 mm      First edge, Cutter NC velocity -2 mm/s
6–10 mm     Direct quintic transition from slope -0.4 to -1.6
10–15 mm    Second edge, Cutter NC velocity -8 mm/s
15–17 mm    Smooth exit from slope -1.6 to zero
17–27 mm    Two-second quintic return to absolute 17 mm
27–63.7 mm  Dwell at absolute 17 mm
```

No synchronous-following plateau is introduced between the two edges.

## Object-relative diagnostic coordinate

The two physical object vertices are defined at `0 mm` and `-3 mm`. To make the scope signal use that physical sign convention, change the diagnostic formula to:

```text
ObjectRelativePosition = -(SlaveCamPosition + MasterPhase)
```

This is a diagnostic sign change only; it does not alter the commanded Cutter motion. Expected endpoint values are:

```text
First edge:  0 -> -3 mm
Transition: -3 -> -3 mm
Second edge: -3 -> 0 mm
```

## Expected dynamics

The two-second return displacement becomes `16.6 mm`. The ideal rest-to-rest quintic estimates are:

```text
Peak return velocity     ~= 15.5625 mm/s
Peak return acceleration ~= 23.96 mm/s^2
Peak return jerk         = 124.5 mm/s^3
```

The existing recovery velocity limit `20 mm/s` and existing acceleration/jerk limits remain unchanged. The loaded table must still pass the existing `MC_ReadCamTableCharacteristics` checks before the machine enters `READY`.

## Implementation scope

Update:

- `ST_CamRecipe` default transition and exit distances.
- `FB_CamRecipeValidator` first/second slope formulas.
- The offline physical-cam model and its expected points/dynamics.
- `FB_CamMachine` expected velocity diagnostics and object-relative coordinate sign.
- PLC structural tests and the physical commissioning guide.

The profile builder remains generic and shall not hardcode a second profile. Automatic conveyor reference behavior, relative `MC_CamIn`, static coupling before master start, stop behavior, and Cutter absolute ready position remain unchanged.

## Acceptance criteria

- The generated eight cam points exactly match the approved table.
- First-edge Cutter NC velocity is `-2 mm/s` and second-edge velocity is `-8 mm/s`.
- Process boundaries are `1`, `6`, `10`, `15`, `17`, `27`, and `63.7 mm`.
- Cutter absolute position remains within the accepted design range `0.4–17 mm`.
- Scope object-relative position reports `0 -> -3 -> -3 -> 0 mm` across the two cuts and transition.
- Return time is two seconds and the calculated return peak velocity remains below `20 mm/s`.
- All Python tests pass and all edited TwinCAT XML files parse successfully.
- The final project is compiled and checked in TwinCAT XAE before physical activation.
