# TwinCAT Electronic Cam Grinding Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a layered TwinCAT PLC application that generates a periodic Motion Function cam at runtime, drives Axis 1 as a constant-speed conveyor, drives linear Axis 2 through two complete 25 mm cuts at 150 mm/s and 50 mm/s, and returns Axis 2 before the next 500 mm product pitch.

**Architecture:** Keep deterministic recipe math and cam-point construction separate from NC command execution. `FB_CamRecipeValidator` derives and validates geometry, `FB_CamProfileBuilder` owns the ten Motion Function points, `FB_ElectronicCam` is the sole wrapper around camming library calls, and `FB_CamMachine` owns the machine lifecycle while `MAIN` only wires globals to the top-level block.

**Tech Stack:** TwinCAT 3.1.4024.75, IEC 61131-3 Structured Text, Tc2_MC2 3.3.72.0, Tc2_MC2_Camming 3.4.6.0, Tc2_Math, Python standard-library `unittest`, Visual Studio 2019/TwinCAT XAE.

## Global Constraints

- Target TwinCAT project version is exactly the existing 3.1.4024.75 system project and 3.1.4024 PLC project family.
- Preserve the user's existing uncommitted change in `cam_case/cam_case.tsproj`; do not stage or commit that file.
- Axis 1 is `GVL_Motion.gAxisConveyor`; Axis 2 is `GVL_Motion.gAxisCutter`.
- Simulation and real hardware use identical PLC code; switching happens only in TwinCAT NC configuration, never through a runtime BOOL.
- Default recipe: pitch 500 mm, conveyor 100 mm/s, cut speed 50 mm/s, edge length 25 mm, edge starts at 100 mm and 300 mm, blend distance 20 mm, return complete at 480 mm.
- Motion Function table ID is the value `1` stored in an `MC_CAM_ID`-typed constant, table type is `MC_TABLETYPE_MOTIONFUNCTION`, and `MC_CamTableSelect.Periodic` is `TRUE`.
- Only a stopped, decoupled machine may apply and reload a recipe.
- Axis 2 is linear and must return to position 0 after the second edge and after any controlled mid-cycle stop.
- No PLC Visualization or HMI is added; online variables and TwinCAT Scope are the runtime acceptance surfaces.
- Standard PLC logic is not a safety function and does not replace E-stop, STO, limits, homing, or TwinSAFE.
- Use `apply_patch` for source edits and stage only files belonging to the current task.

## File Map

### Offline verification

- `tools/cam_profile_model.py`: dependency-free reference implementation for geometry and normalized quintic dynamics.
- `tests/test_cam_profile_model.py`: numeric acceptance tests for the default recipe and invalid recipes.
- `tests/test_plc_project_structure.py`: parses TwinCAT XML and checks source registration, declarations, layering, and required MC calls.

### PLC data contracts

- `cam_case/Untitled1/DUTs/E_CamMachineState.TcDUT`: machine lifecycle states.
- `cam_case/Untitled1/DUTs/E_CamProcessPhase.TcDUT`: Scope-friendly process phases.
- `cam_case/Untitled1/DUTs/E_CamApplicationError.TcDUT`: application error codes.
- `cam_case/Untitled1/DUTs/E_CamErrorSource.TcDUT`: error source classification.
- `cam_case/Untitled1/DUTs/ST_CamRecipe.TcDUT`: editable recipe and recovery dynamics.
- `cam_case/Untitled1/DUTs/ST_CamDerivedGeometry.TcDUT`: derived phases, positions, and slopes.
- `cam_case/Untitled1/DUTs/ST_CamCommands.TcDUT`: operator command interface.
- `cam_case/Untitled1/DUTs/ST_CamStatus.TcDUT`: diagnostic state and calculated dynamics.
- `cam_case/Untitled1/DUTs/ST_CamScope.TcDUT`: flat Scope signal structure.
- `cam_case/Untitled1/GVLs/GVL_Motion.TcGVL`: two AXIS_REF variables only.
- `cam_case/Untitled1/GVLs/GVL_CamControl.TcGVL`: recipe, commands, status, and Scope instances.

### PLC behavior

- `cam_case/Untitled1/POUs/Cam/F_MakeCamPoint.TcPOU`: constructs one complete `MC_MotionFunctionPoint`.
- `cam_case/Untitled1/POUs/Cam/FB_CamRecipeValidator.TcPOU`: validates recipe and emits derived geometry.
- `cam_case/Untitled1/POUs/Cam/FB_CamProfileBuilder.TcPOU`: owns ten points and the `MC_CAM_REF`.
- `cam_case/Untitled1/POUs/Cam/FB_ElectronicCam.TcPOU`: loads, characterizes, couples, and decouples the table.
- `cam_case/Untitled1/POUs/Cam/FB_CamMachine.TcPOU`: owns power, reference, run, stop, recovery, and error state.
- `cam_case/Untitled1/POUs/MAIN.TcPOU`: calls the top-level machine block only.

### Handoff documentation

- `docs/twincat-operation.md`: XAE build, axis linking, simulation/real switching, command sequence, Scope setup, and real-machine warnings.

---

### Task 1: Create the Executable Cam-Math Oracle

**Files:**
- Create: `tools/cam_profile_model.py`
- Create: `tests/test_cam_profile_model.py`

**Interfaces:**
- Consumes: no project code; Python 3 standard library only.
- Produces: `Recipe`, `Point`, `derive_points(recipe)`, `validate_recipe(recipe)`, and `sample_characteristics(points, master_velocity)` for all later numeric checks.

- [ ] **Step 1: Write failing default-geometry tests**

Create `tests/test_cam_profile_model.py` with these exact behavioral assertions:

```python
import math
import unittest

from tools.cam_profile_model import (
    Recipe,
    derive_points,
    sample_characteristics,
    validate_recipe,
)


class CamProfileModelTests(unittest.TestCase):
    def test_default_points_match_approved_geometry(self):
        points = derive_points(Recipe())
        actual = [(p.x, p.y, p.slope) for p in points]
        self.assertEqual(
            actual,
            [
                (0.0, 0.0, 0.0),
                (20.0, 0.0, 0.0),
                (100.0, 100.0, 1.5),
                (150.0, 175.0, 1.5),
                (170.0, 200.0, 1.0),
                (280.0, 310.0, 1.0),
                (300.0, 325.0, 0.5),
                (350.0, 350.0, 0.5),
                (480.0, 0.0, 0.0),
                (500.0, 0.0, 0.0),
            ],
        )

    def test_two_complete_edges_are_derived_from_edge_length(self):
        points = derive_points(Recipe())
        first_relative_start = points[2].y - points[2].x
        first_relative_end = points[3].y - points[3].x
        second_relative_start = points[6].y - points[6].x
        second_relative_end = points[7].y - points[7].x
        self.assertEqual((first_relative_start, first_relative_end), (0.0, 25.0))
        self.assertEqual((second_relative_start, second_relative_end), (25.0, 0.0))

    def test_default_period_is_position_and_velocity_continuous(self):
        points = derive_points(Recipe())
        self.assertEqual(points[0].y, points[-1].y)
        self.assertEqual(points[0].slope, points[-1].slope)

    def test_default_return_dynamics_fit_approved_limits(self):
        characteristics = sample_characteristics(derive_points(Recipe()), 100.0)
        self.assertTrue(math.isclose(characteristics.max_abs_velocity, 527.0013, abs_tol=0.2))
        self.assertTrue(math.isclose(characteristics.max_abs_acceleration, 1345.374, abs_tol=1.0))
        self.assertTrue(math.isclose(characteristics.max_abs_jerk, 10623.578, abs_tol=2.0))
        self.assertLess(characteristics.max_abs_velocity, 600.0)
        self.assertLess(characteristics.max_abs_acceleration, 1600.0)
        self.assertLess(characteristics.max_abs_jerk, 12000.0)

    def test_invalid_recipes_are_rejected(self):
        cases = [
            Recipe(conveyor_velocity=0.0),
            Recipe(cut_velocity=0.0),
            Recipe(cut_velocity=100.0),
            Recipe(edge_length=0.0),
            Recipe(edge_length=100.0),
            Recipe(second_edge_start=155.0),
            Recipe(return_end_phase=510.0),
        ]
        for recipe in cases:
            with self.subTest(recipe=recipe):
                self.assertFalse(validate_recipe(recipe).valid)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the tests and verify RED**

Run:

```powershell
python -m unittest tests.test_cam_profile_model -v
```

Expected: import failure for `tools.cam_profile_model` because the oracle does not exist.

- [ ] **Step 3: Implement the minimum reference model**

Create `tools/cam_profile_model.py` with these data contracts and formulas:

```python
from dataclasses import dataclass, replace
from typing import Iterable, List, Sequence, Tuple


@dataclass(frozen=True)
class Recipe:
    pitch: float = 500.0
    conveyor_velocity: float = 100.0
    cut_velocity: float = 50.0
    edge_length: float = 25.0
    first_edge_start: float = 100.0
    second_edge_start: float = 300.0
    blend_distance: float = 20.0
    return_end_phase: float = 480.0
    max_slave_velocity: float = 600.0
    max_slave_acceleration: float = 1600.0
    max_slave_jerk: float = 12000.0


@dataclass(frozen=True)
class Point:
    x: float
    y: float
    slope: float
    function: str


@dataclass(frozen=True)
class Validation:
    valid: bool
    reason: str = ""


@dataclass(frozen=True)
class Characteristics:
    max_abs_velocity: float
    max_abs_acceleration: float
    max_abs_jerk: float


def derive_points(recipe: Recipe) -> List[Point]:
    cut_master = recipe.edge_length * recipe.conveyor_velocity / recipe.cut_velocity
    first_slope = 1.0 + recipe.cut_velocity / recipe.conveyor_velocity
    second_slope = 1.0 - recipe.cut_velocity / recipe.conveyor_velocity
    first_end = recipe.first_edge_start + cut_master
    exit_end = first_end + recipe.blend_distance
    entry_start = recipe.second_edge_start - recipe.blend_distance
    second_end = recipe.second_edge_start + cut_master
    first_y = recipe.first_edge_start
    first_end_y = first_y + first_slope * cut_master
    exit_end_y = first_end_y + 0.5 * (first_slope + 1.0) * recipe.blend_distance
    clearance = exit_end_y - exit_end
    entry_start_y = entry_start + clearance
    second_start_y = recipe.second_edge_start + recipe.edge_length
    second_end_y = second_start_y + second_slope * cut_master
    return [
        Point(0.0, 0.0, 0.0, "linear"),
        Point(recipe.blend_distance, 0.0, 0.0, "poly5_mm"),
        Point(recipe.first_edge_start, first_y, first_slope, "linear"),
        Point(first_end, first_end_y, first_slope, "poly5_mm"),
        Point(exit_end, exit_end_y, 1.0, "linear"),
        Point(entry_start, entry_start_y, 1.0, "poly5_mm"),
        Point(recipe.second_edge_start, second_start_y, second_slope, "linear"),
        Point(second_end, second_end_y, second_slope, "poly5_mm"),
        Point(recipe.return_end_phase, 0.0, 0.0, "linear"),
        Point(recipe.pitch, 0.0, 0.0, "not_defined"),
    ]


def validate_recipe(recipe: Recipe) -> Validation:
    if recipe.pitch <= 0.0:
        return Validation(False, "pitch")
    if recipe.conveyor_velocity <= 0.0:
        return Validation(False, "conveyor_velocity")
    if not 0.0 < recipe.cut_velocity < recipe.conveyor_velocity:
        return Validation(False, "cut_velocity")
    if recipe.edge_length <= 0.0 or recipe.blend_distance <= 0.0:
        return Validation(False, "geometry_positive")
    points = derive_points(recipe)
    if recipe.first_edge_start <= recipe.blend_distance:
        return Validation(False, "approach_space")
    if not all(left.x < right.x for left, right in zip(points, points[1:])):
        return Validation(False, "phase_order")
    if points[4].x >= points[5].x:
        return Validation(False, "sync_space")
    if points[7].x >= recipe.return_end_phase or recipe.return_end_phase >= recipe.pitch:
        return Validation(False, "return_space")
    characteristics = sample_characteristics(points, recipe.conveyor_velocity)
    if characteristics.max_abs_velocity > recipe.max_slave_velocity:
        return Validation(False, "slave_velocity")
    if characteristics.max_abs_acceleration > recipe.max_slave_acceleration:
        return Validation(False, "slave_acceleration")
    if characteristics.max_abs_jerk > recipe.max_slave_jerk:
        return Validation(False, "slave_jerk")
    return Validation(True)


def _quintic_coefficients(p0: Point, p1: Point) -> Tuple[float, ...]:
    dx = p1.x - p0.x
    c0, c1, c2 = p0.y, p0.slope * dx, 0.0
    distance = p1.y - c0 - c1
    velocity = p1.slope * dx - c1
    c5 = 0.5 * (-6.0 * velocity + 12.0 * distance)
    c4 = velocity - 3.0 * distance - 2.0 * c5
    c3 = distance - c4 - c5
    return c0, c1, c2, c3, c4, c5


def sample_characteristics(
    points: Sequence[Point], master_velocity: float, samples_per_segment: int = 20000
) -> Characteristics:
    max_velocity = max_acceleration = max_jerk = 0.0
    for left, right in zip(points, points[1:]):
        dx = right.x - left.x
        if left.function == "linear":
            max_velocity = max(max_velocity, abs(left.slope * master_velocity))
            continue
        if left.function == "not_defined":
            continue
        coeffs = _quintic_coefficients(left, right)
        _, c1, c2, c3, c4, c5 = coeffs
        for index in range(samples_per_segment + 1):
            u = index / samples_per_segment
            d1 = c1 + 2*c2*u + 3*c3*u**2 + 4*c4*u**3 + 5*c5*u**4
            d2 = 2*c2 + 6*c3*u + 12*c4*u**2 + 20*c5*u**3
            d3 = 6*c3 + 24*c4*u + 60*c5*u**2
            max_velocity = max(max_velocity, abs(d1 / dx * master_velocity))
            max_acceleration = max(max_acceleration, abs(d2 / dx**2 * master_velocity**2))
            max_jerk = max(max_jerk, abs(d3 / dx**3 * master_velocity**3))
    return Characteristics(max_velocity, max_acceleration, max_jerk)
```

Remove unused imports after the first green run.

- [ ] **Step 4: Run all offline tests and verify GREEN**

Run:

```powershell
python -m unittest discover -s tests -v
```

Expected: five tests pass with no third-party dependency.

- [ ] **Step 5: Commit the oracle**

```powershell
git add -- tools/cam_profile_model.py tests/test_cam_profile_model.py
git commit -m "test: add electronic cam geometry oracle"
```

---

### Task 2: Add PLC Contracts, Globals, and Project Registration

**Files:**
- Create: all eleven DUT/GVL files listed in the File Map under “PLC data contracts”
- Create: `tests/test_plc_project_structure.py`
- Modify: `cam_case/Untitled1/Untitled1.plcproj`

**Interfaces:**
- Consumes: approved names and defaults from the design specification.
- Produces: the exact public types used by Tasks 3–5 and two qualified global lists.

- [ ] **Step 1: Write the failing project-structure test**

Create `tests/test_plc_project_structure.py`:

```python
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PLC_ROOT = ROOT / "cam_case" / "Untitled1"
PROJECT = PLC_ROOT / "Untitled1.plcproj"

DATA_SOURCES = [
    "DUTs\\E_CamMachineState.TcDUT",
    "DUTs\\E_CamProcessPhase.TcDUT",
    "DUTs\\E_CamApplicationError.TcDUT",
    "DUTs\\E_CamErrorSource.TcDUT",
    "DUTs\\ST_CamRecipe.TcDUT",
    "DUTs\\ST_CamDerivedGeometry.TcDUT",
    "DUTs\\ST_CamCommands.TcDUT",
    "DUTs\\ST_CamStatus.TcDUT",
    "DUTs\\ST_CamScope.TcDUT",
    "GVLs\\GVL_Motion.TcGVL",
    "GVLs\\GVL_CamControl.TcGVL",
]


def compile_includes():
    root = ET.parse(PROJECT).getroot()
    namespace = {"msb": "http://schemas.microsoft.com/developer/msbuild/2003"}
    return [node.attrib["Include"] for node in root.findall(".//msb:Compile", namespace)]


def source_text(relative_path):
    root = ET.parse(PLC_ROOT / relative_path).getroot()
    return "\n".join((node.text or "") for node in root.iter())


class PlcProjectStructureTests(unittest.TestCase):
    def test_data_sources_exist_and_are_registered(self):
        includes = compile_includes()
        for relative_path in DATA_SOURCES:
            self.assertTrue((PLC_ROOT / relative_path).is_file(), relative_path)
            self.assertIn(relative_path, includes)

    def test_required_libraries_are_explicit(self):
        project_text = PROJECT.read_text(encoding="utf-8")
        for library in ("Tc2_MC2", "Tc2_MC2_Camming", "Tc2_Math"):
            self.assertIn(f'PlaceholderReference Include="{library}"', project_text)

    def test_axis_globals_use_stable_link_names(self):
        declaration = source_text("GVLs\\GVL_Motion.TcGVL")
        self.assertIn("gAxisConveyor : AXIS_REF", declaration)
        self.assertIn("gAxisCutter : AXIS_REF", declaration)

    def test_recipe_defaults_are_reviewable_in_one_type(self):
        declaration = source_text("DUTs\\ST_CamRecipe.TcDUT")
        for fragment in (
            "lrObjectPitch : LREAL := 500.0",
            "lrConveyorVelocity : LREAL := 100.0",
            "lrCutVelocity : LREAL := 50.0",
            "lrObjectEdgeLength : LREAL := 25.0",
            "lrFirstEdgeStart : LREAL := 100.0",
            "lrSecondEdgeStart : LREAL := 300.0",
        ):
            self.assertIn(fragment, declaration)


if __name__ == "__main__":
    unittest.main()
```

- [ ] **Step 2: Run the structure test and verify RED**

Run:

```powershell
python -m unittest tests.test_plc_project_structure -v
```

Expected: failures naming the missing DUT and GVL files and missing `Tc2_Math` reference.

- [ ] **Step 3: Add exact PLC type declarations**

Create one `.TcDUT` XML object per declaration, using `ProductVersion="3.1.4024.16"` and a unique GUID generated with `[guid]::NewGuid()`. The CDATA declarations must be:

```iecst
TYPE E_CamMachineState :
(
    CAM_STATE_DISABLED := 0,
    CAM_STATE_POWERING,
    CAM_STATE_IDLE,
    CAM_STATE_REFERENCING_MASTER,
    CAM_STATE_REFERENCING_SLAVE,
    CAM_STATE_BUILDING_TABLE,
    CAM_STATE_LOADING_TABLE,
    CAM_STATE_READY,
    CAM_STATE_CAMMING_IN,
    CAM_STATE_STARTING_MASTER,
    CAM_STATE_RUNNING,
    CAM_STATE_STOPPING_MASTER,
    CAM_STATE_RELEASING_STOP,
    CAM_STATE_CAMMING_OUT,
    CAM_STATE_RETURNING_SLAVE,
    CAM_STATE_RESETTING,
    CAM_STATE_FAULT
);
END_TYPE

TYPE E_CamProcessPhase :
(
    CAM_PHASE_DISABLED := 0,
    CAM_PHASE_PREPARE,
    CAM_PHASE_APPROACH,
    CAM_PHASE_FIRST_EDGE,
    CAM_PHASE_EXIT_FIRST_EDGE,
    CAM_PHASE_SYNCHRONOUS,
    CAM_PHASE_ENTER_SECOND_EDGE,
    CAM_PHASE_SECOND_EDGE,
    CAM_PHASE_RETURN,
    CAM_PHASE_DWELL
);
END_TYPE

TYPE E_CamApplicationError :
(
    CAM_ERROR_NONE := 0,
    CAM_ERROR_COMMAND_NOT_ALLOWED,
    CAM_ERROR_NOT_REFERENCED,
    CAM_ERROR_RECIPE_PITCH,
    CAM_ERROR_RECIPE_CONVEYOR_VELOCITY,
    CAM_ERROR_RECIPE_CUT_VELOCITY,
    CAM_ERROR_RECIPE_EDGE_LENGTH,
    CAM_ERROR_RECIPE_PHASE_ORDER,
    CAM_ERROR_RECIPE_SYNC_SPACE,
    CAM_ERROR_RECIPE_RETURN_SPACE,
    CAM_ERROR_RECIPE_DYNAMICS,
    CAM_ERROR_AXIS_POWER,
    CAM_ERROR_AXIS_REFERENCE,
    CAM_ERROR_CAM_BUILD,
    CAM_ERROR_CAM_LOAD,
    CAM_ERROR_CAM_CHARACTERISTICS,
    CAM_ERROR_CAM_IN,
    CAM_ERROR_MASTER_MOVE,
    CAM_ERROR_MASTER_STOP,
    CAM_ERROR_CAM_OUT,
    CAM_ERROR_SLAVE_RETURN,
    CAM_ERROR_AXIS_RESET
);
END_TYPE

TYPE E_CamErrorSource :
(
    CAM_SOURCE_NONE := 0,
    CAM_SOURCE_RECIPE,
    CAM_SOURCE_MASTER_AXIS,
    CAM_SOURCE_SLAVE_AXIS,
    CAM_SOURCE_TABLE_LOAD,
    CAM_SOURCE_TABLE_CHARACTERISTICS,
    CAM_SOURCE_CAM_IN,
    CAM_SOURCE_CAM_OUT
);
END_TYPE

TYPE ST_CamRecipe :
STRUCT
    lrObjectPitch : LREAL := 500.0;
    lrConveyorVelocity : LREAL := 100.0;
    lrCutVelocity : LREAL := 50.0;
    lrObjectEdgeLength : LREAL := 25.0;
    lrFirstEdgeStart : LREAL := 100.0;
    lrSecondEdgeStart : LREAL := 300.0;
    lrBlendMasterDistance : LREAL := 20.0;
    lrReturnEndPhase : LREAL := 480.0;
    lrMasterAcceleration : LREAL := 500.0;
    lrMasterDeceleration : LREAL := 500.0;
    lrMasterJerk : LREAL := 5000.0;
    lrMaxSlaveVelocity : LREAL := 600.0;
    lrMaxSlaveAcceleration : LREAL := 1600.0;
    lrMaxSlaveJerk : LREAL := 12000.0;
    lrRecoveryVelocity : LREAL := 100.0;
    lrRecoveryAcceleration : LREAL := 500.0;
    lrRecoveryDeceleration : LREAL := 500.0;
    lrRecoveryJerk : LREAL := 5000.0;
    lrPositionTolerance : LREAL := 0.1;
END_STRUCT
END_TYPE

TYPE ST_CamDerivedGeometry :
STRUCT
    lrCutMasterDistance : LREAL;
    lrFirstSlope : LREAL;
    lrSecondSlope : LREAL;
    lrFirstEdgeEnd : LREAL;
    lrExitBlendEnd : LREAL;
    lrEntryBlendStart : LREAL;
    lrSecondEdgeEnd : LREAL;
    lrClearanceRelative : LREAL;
    lrFirstStartY : LREAL;
    lrFirstEndY : LREAL;
    lrExitEndY : LREAL;
    lrEntryStartY : LREAL;
    lrSecondStartY : LREAL;
    lrSecondEndY : LREAL;
END_STRUCT
END_TYPE

TYPE ST_CamCommands :
STRUCT
    bEnable : BOOL;
    bReference : BOOL;
    bApplyRecipe : BOOL;
    bStart : BOOL;
    bStop : BOOL;
    bReset : BOOL;
END_STRUCT
END_TYPE

TYPE ST_CamStatus :
STRUCT
    eState : E_CamMachineState;
    eProcessPhase : E_CamProcessPhase;
    bPowerEnabled : BOOL;
    bReferenced : BOOL;
    bRecipeLoaded : BOOL;
    bCamActive : BOOL;
    bRunning : BOOL;
    bBusy : BOOL;
    eApplicationError : E_CamApplicationError;
    eErrorSource : E_CamErrorSource;
    udiMotionErrorID : UDINT;
    lrCalculatedMaxSlaveVelocity : LREAL;
    lrCalculatedMaxSlaveAcceleration : LREAL;
    lrCalculatedMaxSlaveJerk : LREAL;
END_STRUCT
END_TYPE

TYPE ST_CamScope :
STRUCT
    lrMasterSetPosition : LREAL;
    lrMasterActualPosition : LREAL;
    lrMasterSetVelocity : LREAL;
    lrMasterActualVelocity : LREAL;
    lrMasterPhase : LREAL;
    lrSlaveSetPosition : LREAL;
    lrSlaveActualPosition : LREAL;
    lrSlaveSetVelocity : LREAL;
    lrSlaveActualVelocity : LREAL;
    lrExpectedSlaveVelocity : LREAL;
    lrObjectRelativePosition : LREAL;
    eMachineState : E_CamMachineState;
    eProcessPhase : E_CamProcessPhase;
    bCamActive : BOOL;
    bRecipeLoaded : BOOL;
    eApplicationError : E_CamApplicationError;
    udiMotionErrorID : UDINT;
END_STRUCT
END_TYPE
```

- [ ] **Step 4: Add qualified global interfaces**

Create `GVL_Motion.TcGVL` and `GVL_CamControl.TcGVL` with these declarations:

```iecst
{attribute 'qualified_only'}
VAR_GLOBAL
    gAxisConveyor : AXIS_REF;
    gAxisCutter : AXIS_REF;
END_VAR
```

```iecst
{attribute 'qualified_only'}
VAR_GLOBAL
    stCommands : ST_CamCommands;
    stRecipe : ST_CamRecipe;
    stStatus : ST_CamStatus;
    stScope : ST_CamScope;
END_VAR
```

- [ ] **Step 5: Register sources and Tc2_Math in the PLC project**

In `Untitled1.plcproj`, add each DUT/GVL as a `<Compile Include="...">` item before the behavior POUs, add folders `POUs\Cam` and `Tests`, and add:

```xml
<PlaceholderReference Include="Tc2_Math">
  <DefaultResolution>Tc2_Math, * (Beckhoff Automation GmbH)</DefaultResolution>
  <Namespace>Tc2_Math</Namespace>
</PlaceholderReference>
```

Do not modify `cam_case.tsproj`.

- [ ] **Step 6: Run offline tests and build the PLC project**

Run:

```powershell
python -m unittest discover -s tests -v
```

Expected: all tests pass.

Then run:

```powershell
& 'G:\Program Files (x86)\Microsoft Visual Studio\2019\Community\Common7\IDE\devenv.com' 'E:\PLC_Project\cam_case\cam_case.sln' /Build 'Debug|TwinCAT RT (x64)' /Out 'E:\PLC_Project\cam_case\twincat-build.log'
```

Expected: TwinCAT/PLC build succeeds. If XAE reports that the existing system-project change must be saved, cancel that save and inspect the build log rather than overwriting the user's change.

- [ ] **Step 7: Commit contracts only**

Stage the DUTs, GVLs, PLC project, and structure test explicitly; verify `cam_case.tsproj` is absent from the staged diff.

```powershell
git commit -m "feat: add electronic cam PLC contracts"
```

---

### Task 3: Implement Recipe Validation and Motion Function Construction

**Files:**
- Modify: `tests/test_plc_project_structure.py`
- Create: `cam_case/Untitled1/POUs/Cam/F_MakeCamPoint.TcPOU`
- Create: `cam_case/Untitled1/POUs/Cam/FB_CamRecipeValidator.TcPOU`
- Create: `cam_case/Untitled1/POUs/Cam/FB_CamProfileBuilder.TcPOU`
- Modify: `cam_case/Untitled1/Untitled1.plcproj`

**Interfaces:**
- Consumes: `ST_CamRecipe`, `ST_CamDerivedGeometry`, `E_CamApplicationError`.
- Produces: `FB_CamRecipeValidator(bExecute, stRecipe)` and `FB_CamProfileBuilder(bExecute, stRecipe, stDerived)`; the builder exposes `aPoints : ARRAY[1..10] OF MC_MotionFunctionPoint` and `stCamRef : MC_CAM_REF`.

- [ ] **Step 1: Extend structure tests before adding production POUs**

Append behavior paths to a `BEHAVIOR_SOURCES` list and add assertions:

```python
BEHAVIOR_SOURCES = [
    "POUs\\Cam\\F_MakeCamPoint.TcPOU",
    "POUs\\Cam\\FB_CamRecipeValidator.TcPOU",
    "POUs\\Cam\\FB_CamProfileBuilder.TcPOU",
]

def test_profile_sources_are_registered(self):
    includes = compile_includes()
    for relative_path in BEHAVIOR_SOURCES:
        self.assertTrue((PLC_ROOT / relative_path).is_file(), relative_path)
        self.assertIn(relative_path, includes)

def test_builder_owns_exactly_ten_motion_points(self):
    source = source_text("POUs\\Cam\\FB_CamProfileBuilder.TcPOU")
    self.assertIn("ARRAY[1..10] OF MC_MotionFunctionPoint", source)
    self.assertIn("MC_TABLETYPE_MOTIONFUNCTION", source)
    self.assertIn("NoOfRows := 10", source)
    self.assertIn("NoOfColumns := 1", source)
```

- [ ] **Step 2: Run the new tests and verify RED**

Run:

```powershell
python -m unittest tests.test_plc_project_structure -v
```

Expected: missing-file failures for the three cam-profile POUs.

- [ ] **Step 3: Implement the point factory**

Create `F_MakeCamPoint` with this exact public signature and assignment behavior:

```iecst
FUNCTION F_MakeCamPoint : MC_MotionFunctionPoint
VAR_INPUT
    udiPointIndex : UDINT;
    eFunctionType : MC_MotionFunctionType;
    ePointType : MC_MotionPointType;
    udiRelIndexNextPoint : UDINT;
    lrMasterPos : LREAL;
    lrSlavePos : LREAL;
    lrSlaveVelo : LREAL;
END_VAR

F_MakeCamPoint.PointIndex := udiPointIndex;
F_MakeCamPoint.FunctionType := eFunctionType;
F_MakeCamPoint.PointType := ePointType;
F_MakeCamPoint.RelIndexNextPoint := udiRelIndexNextPoint;
F_MakeCamPoint.MasterPos := lrMasterPos;
F_MakeCamPoint.SlavePos := lrSlavePos;
F_MakeCamPoint.SlaveVelo := lrSlaveVelo;
F_MakeCamPoint.SlaveAcc := 0.0;
F_MakeCamPoint.SlaveJerk := 0.0;
```

- [ ] **Step 4: Implement deterministic recipe validation**

Use an `R_TRIG` on `bExecute`. On its rising edge, clear outputs, calculate all fields below, and then apply the validation order exactly so the first reported error is deterministic:

```iecst
stDerived.lrCutMasterDistance :=
    stRecipe.lrObjectEdgeLength * stRecipe.lrConveyorVelocity / stRecipe.lrCutVelocity;
stDerived.lrFirstSlope := 1.0 + stRecipe.lrCutVelocity / stRecipe.lrConveyorVelocity;
stDerived.lrSecondSlope := 1.0 - stRecipe.lrCutVelocity / stRecipe.lrConveyorVelocity;
stDerived.lrFirstEdgeEnd := stRecipe.lrFirstEdgeStart + stDerived.lrCutMasterDistance;
stDerived.lrExitBlendEnd := stDerived.lrFirstEdgeEnd + stRecipe.lrBlendMasterDistance;
stDerived.lrEntryBlendStart := stRecipe.lrSecondEdgeStart - stRecipe.lrBlendMasterDistance;
stDerived.lrSecondEdgeEnd := stRecipe.lrSecondEdgeStart + stDerived.lrCutMasterDistance;
stDerived.lrFirstStartY := stRecipe.lrFirstEdgeStart;
stDerived.lrFirstEndY := stDerived.lrFirstStartY
    + stDerived.lrFirstSlope * stDerived.lrCutMasterDistance;
stDerived.lrExitEndY := stDerived.lrFirstEndY
    + 0.5 * (stDerived.lrFirstSlope + 1.0) * stRecipe.lrBlendMasterDistance;
stDerived.lrClearanceRelative := stDerived.lrExitEndY - stDerived.lrExitBlendEnd;
stDerived.lrEntryStartY := stDerived.lrEntryBlendStart + stDerived.lrClearanceRelative;
stDerived.lrSecondStartY := stRecipe.lrSecondEdgeStart + stRecipe.lrObjectEdgeLength;
stDerived.lrSecondEndY := stDerived.lrSecondStartY
    + stDerived.lrSecondSlope * stDerived.lrCutMasterDistance;
```

Validation order:

```iecst
IF stRecipe.lrObjectPitch <= 0.0 THEN
    eError := CAM_ERROR_RECIPE_PITCH;
ELSIF stRecipe.lrConveyorVelocity <= 0.0 THEN
    eError := CAM_ERROR_RECIPE_CONVEYOR_VELOCITY;
ELSIF (stRecipe.lrCutVelocity <= 0.0)
   OR (stRecipe.lrCutVelocity >= stRecipe.lrConveyorVelocity) THEN
    eError := CAM_ERROR_RECIPE_CUT_VELOCITY;
ELSIF (stRecipe.lrObjectEdgeLength <= 0.0)
   OR (stRecipe.lrBlendMasterDistance <= 0.0) THEN
    eError := CAM_ERROR_RECIPE_EDGE_LENGTH;
ELSIF stRecipe.lrFirstEdgeStart <= stRecipe.lrBlendMasterDistance THEN
    eError := CAM_ERROR_RECIPE_PHASE_ORDER;
ELSIF stDerived.lrExitBlendEnd >= stDerived.lrEntryBlendStart THEN
    eError := CAM_ERROR_RECIPE_SYNC_SPACE;
ELSIF (stDerived.lrSecondEdgeEnd >= stRecipe.lrReturnEndPhase)
   OR (stRecipe.lrReturnEndPhase >= stRecipe.lrObjectPitch) THEN
    eError := CAM_ERROR_RECIPE_RETURN_SPACE;
ELSIF (stRecipe.lrMaxSlaveVelocity <= 0.0)
   OR (stRecipe.lrMaxSlaveAcceleration <= 0.0)
   OR (stRecipe.lrMaxSlaveJerk <= 0.0) THEN
    eError := CAM_ERROR_RECIPE_DYNAMICS;
ELSE
    bValid := TRUE;
END_IF;
bDone := TRUE;
bError := NOT bValid;
```

Guard the divisions by checking raw positive velocity inputs before calculating derived values; either perform the positive checks before the calculation block or use a local safe denominator. Do not execute a division when either velocity is non-positive.

- [ ] **Step 5: Implement the ten-point profile builder**

On the `bExecute` rising edge, populate points using `F_MakeCamPoint`. Use REST points for zero-velocity boundaries and VELOCITY points for non-zero slopes:

```iecst
aPoints[1] := F_MakeCamPoint(1, MOTIONFUNCTYPE_POLYNOM1,
    MOTIONPOINTTYPE_REST, 1, 0.0, 0.0, 0.0);
aPoints[2] := F_MakeCamPoint(2, MOTIONFUNCTYPE_POLYNOM5_MM,
    MOTIONPOINTTYPE_REST, 1, stRecipe.lrBlendMasterDistance, 0.0, 0.0);
aPoints[3] := F_MakeCamPoint(3, MOTIONFUNCTYPE_POLYNOM1,
    MOTIONPOINTTYPE_VELOCITY, 1, stRecipe.lrFirstEdgeStart,
    stDerived.lrFirstStartY, stDerived.lrFirstSlope);
aPoints[4] := F_MakeCamPoint(4, MOTIONFUNCTYPE_POLYNOM5_MM,
    MOTIONPOINTTYPE_VELOCITY, 1, stDerived.lrFirstEdgeEnd,
    stDerived.lrFirstEndY, stDerived.lrFirstSlope);
aPoints[5] := F_MakeCamPoint(5, MOTIONFUNCTYPE_POLYNOM1,
    MOTIONPOINTTYPE_VELOCITY, 1, stDerived.lrExitBlendEnd,
    stDerived.lrExitEndY, 1.0);
aPoints[6] := F_MakeCamPoint(6, MOTIONFUNCTYPE_POLYNOM5_MM,
    MOTIONPOINTTYPE_VELOCITY, 1, stDerived.lrEntryBlendStart,
    stDerived.lrEntryStartY, 1.0);
aPoints[7] := F_MakeCamPoint(7, MOTIONFUNCTYPE_POLYNOM1,
    MOTIONPOINTTYPE_VELOCITY, 1, stRecipe.lrSecondEdgeStart,
    stDerived.lrSecondStartY, stDerived.lrSecondSlope);
aPoints[8] := F_MakeCamPoint(8, MOTIONFUNCTYPE_POLYNOM5_MM,
    MOTIONPOINTTYPE_VELOCITY, 1, stDerived.lrSecondEdgeEnd,
    stDerived.lrSecondEndY, stDerived.lrSecondSlope);
aPoints[9] := F_MakeCamPoint(9, MOTIONFUNCTYPE_POLYNOM1,
    MOTIONPOINTTYPE_REST, 1, stRecipe.lrReturnEndPhase, 0.0, 0.0);
aPoints[10] := F_MakeCamPoint(10, MOTIONFUNCTYPE_NOTDEF,
    MOTIONPOINTTYPE_REST, 0, stRecipe.lrObjectPitch, 0.0, 0.0);

stCamRef.pArray := ADR(aPoints);
stCamRef.ArraySize := SIZEOF(aPoints);
stCamRef.TableType := MC_TABLETYPE_MOTIONFUNCTION;
stCamRef.NoOfRows := 10;
stCamRef.NoOfColumns := 1;
bDone := TRUE;
```

Expose `aPoints` and `stCamRef` as `VAR_OUTPUT`; use no axis reference and no MC command in this block.

- [ ] **Step 6: Register POUs, run tests, and compile**

Register all three POUs in `Untitled1.plcproj`. Run the full `unittest` suite, then the `devenv.com` build command from Task 2.

Expected: offline tests pass and TwinCAT resolves all Motion Function enum/type names against installed `Tc2_MC2_Camming 3.4.6.0`.

- [ ] **Step 7: Commit the deterministic cam builder**

```powershell
git commit -m "feat: build validated motion-function cam profile"
```

Stage only the three new POUs, the PLC project, and the changed structure test.

---

### Task 4: Encapsulate NC Cam Loading and Coupling

**Files:**
- Modify: `tests/test_plc_project_structure.py`
- Create: `cam_case/Untitled1/POUs/Cam/FB_ElectronicCam.TcPOU`
- Modify: `cam_case/Untitled1/Untitled1.plcproj`

**Interfaces:**
- Consumes: `Master : AXIS_REF`, `Slave : AXIS_REF`, `CamTable : MC_CAM_REF`, command pulses `bLoad`, `bCouple`, `bDecouple`, `bReset`, and `nCamTableId : MC_CAM_ID`.
- Produces: `bLoadDone`, `bCoupleDone`, `bDecoupleDone`, `bBusy`, `bCamActive`, `bError`, `udiErrorID`, and `stCharacteristics : MC_TableCharacValues`.

- [ ] **Step 1: Write the failing wrapper-layer test**

Add:

```python
def test_electronic_cam_is_the_only_camming_wrapper(self):
    relative_path = "POUs\\Cam\\FB_ElectronicCam.TcPOU"
    self.assertTrue((PLC_ROOT / relative_path).is_file())
    self.assertIn(relative_path, compile_includes())
    source = source_text(relative_path)
    for function_block in (
        "MC_CamTableSelect",
        "MC_ReadCamTableCharacteristics",
        "MC_CamIn",
        "MC_CamOut",
    ):
        self.assertIn(function_block, source)
    self.assertIn("MC_STARTMODE_RELATIVE", source)
    self.assertIn("Periodic := TRUE", source)
```

- [ ] **Step 2: Run the test and verify RED**

Expected: failure because `FB_ElectronicCam.TcPOU` is absent.

- [ ] **Step 3: Declare the wrapper and private MC instances**

Use this public contract:

```iecst
FUNCTION_BLOCK FB_ElectronicCam
VAR_INPUT
    bLoad : BOOL;
    bCouple : BOOL;
    bDecouple : BOOL;
    bReset : BOOL;
    nCamTableId : MC_CAM_ID := 1;
END_VAR
VAR_IN_OUT
    Master : AXIS_REF;
    Slave : AXIS_REF;
    CamTable : MC_CAM_REF;
END_VAR
VAR_OUTPUT
    bLoadDone : BOOL;
    bCoupleDone : BOOL;
    bDecoupleDone : BOOL;
    bBusy : BOOL;
    bCamActive : BOOL;
    bError : BOOL;
    udiErrorID : UDINT;
    stCharacteristics : MC_TableCharacValues;
END_VAR
VAR
    rtLoad : R_TRIG;
    rtCouple : R_TRIG;
    rtDecouple : R_TRIG;
    fbTableSelect : MC_CamTableSelect;
    fbCharacteristics : MC_ReadCamTableCharacteristics;
    fbCamIn : MC_CamIn;
    fbCamOut : MC_CamOut;
    stCamInOptions : ST_CamInOptions;
    stCharacteristicsOptions : ST_ReadCamTableCharacteristicsOptions;
    stCamOutOptions : ST_CamOutOptions;
    bTableSelectExecute : BOOL;
    bCharacteristicsExecute : BOOL;
    bCamInExecute : BOOL;
    bCamOutExecute : BOOL;
    usiLoadState : USINT;
END_VAR
```

- [ ] **Step 4: Implement edge-owned command sequences**

Call every MC instance every PLC scan with retained Execute flags:

```iecst
fbTableSelect(
    Execute := bTableSelectExecute,
    Periodic := TRUE,
    CamTableID := nCamTableId,
    CamTable := CamTable);
fbCharacteristics(
    Execute := bCharacteristicsExecute,
    CamTableID := nCamTableId,
    Options := stCharacteristicsOptions,
    CamTableCharac := stCharacteristics);
fbCamIn(
    Execute := bCamInExecute,
    MasterOffset := 0.0,
    SlaveOffset := 0.0,
    MasterScaling := 1.0,
    SlaveScaling := 1.0,
    StartMode := MC_STARTMODE_RELATIVE,
    CamTableID := nCamTableId,
    BufferMode := MC_Aborting,
    Options := stCamInOptions,
    Master := Master,
    Slave := Slave);
fbCamOut(
    Execute := bCamOutExecute,
    Options := stCamOutOptions,
    Slave := Slave);
```

Use `usiLoadState = 10` for `MC_CamTableSelect` and `20` for `MC_ReadCamTableCharacteristics`. A load is complete only after both blocks report Done. Set each Execute flag FALSE immediately after its Done/Error result so the instance rearms.

On `rtCouple.Q`, assert `bCamInExecute`; on Done set `bCamActive` and pulse `bCoupleDone`. On `rtDecouple.Q`, assert `bCamOutExecute`; on Done clear `bCamActive` and pulse `bDecoupleDone`. Latch the first MC ErrorID until `bReset` is TRUE. Reject overlapping load/couple/decouple requests by leaving the active command unchanged; `FB_CamMachine` prevents those conflicts at the application layer.

- [ ] **Step 5: Run tests and compile**

Run the full offline suite and the TwinCAT `devenv.com` build. Expected: PASS and no implicit-conversion errors for `MC_CAM_ID`, `MC_CAM_REF`, or cam options.

- [ ] **Step 6: Commit the NC wrapper**

```powershell
git commit -m "feat: encapsulate NC cam loading and coupling"
```

Stage only the wrapper POU, PLC project, and changed structure test.

---

### Task 5: Implement the Machine State, Recovery, MAIN, and Scope

**Files:**
- Modify: `tests/test_plc_project_structure.py`
- Create: `cam_case/Untitled1/POUs/Cam/FB_CamMachine.TcPOU`
- Modify: `cam_case/Untitled1/POUs/MAIN.TcPOU`
- Modify: `cam_case/Untitled1/Untitled1.plcproj`

**Interfaces:**
- Consumes: `ST_CamCommands`, `ST_CamRecipe`, two `AXIS_REF` values, validator, profile builder, and cam wrapper.
- Produces: `ST_CamStatus`, `ST_CamScope`, all approved lifecycle behavior, and a thin `MAIN` wiring layer.

- [ ] **Step 1: Write failing layering and state coverage tests**

Add these tests:

```python
def test_main_contains_only_top_level_wiring(self):
    source = source_text("POUs\\MAIN.TcPOU")
    self.assertIn("FB_CamMachine", source)
    self.assertIn("GVL_Motion.gAxisConveyor", source)
    self.assertIn("GVL_Motion.gAxisCutter", source)
    for forbidden in ("MC_Power", "MC_MoveVelocity", "MC_CamIn", "MC_CamOut"):
        self.assertNotIn(forbidden, source)

def test_machine_contains_required_recovery_order(self):
    relative_path = "POUs\\Cam\\FB_CamMachine.TcPOU"
    self.assertTrue((PLC_ROOT / relative_path).is_file())
    self.assertIn(relative_path, compile_includes())
    source = source_text(relative_path)
    ordered_states = (
        "CAM_STATE_STOPPING_MASTER",
        "CAM_STATE_RELEASING_STOP",
        "CAM_STATE_CAMMING_OUT",
        "CAM_STATE_RETURNING_SLAVE",
    )
    positions = [source.index(state) for state in ordered_states]
    self.assertEqual(positions, sorted(positions))
    self.assertIn("MC_MoveAbsolute", source)
    self.assertIn("MODABS", source)
```

- [ ] **Step 2: Run the tests and verify RED**

Expected: missing machine POU and missing top-level wiring failures.

- [ ] **Step 3: Declare the top-level machine contract and MC ownership**

Use:

```iecst
FUNCTION_BLOCK FB_CamMachine
VAR_INPUT
    stCommands : ST_CamCommands;
    stRecipe : ST_CamRecipe;
END_VAR
VAR_IN_OUT
    AxisConveyor : AXIS_REF;
    AxisCutter : AXIS_REF;
END_VAR
VAR_OUTPUT
    stStatus : ST_CamStatus;
    stScope : ST_CamScope;
END_VAR
```

Private instances are exactly two `MC_Power`, two `MC_SetPosition`, two `MC_Reset`, one `MC_MoveVelocity`, one `MC_Stop`, one `MC_MoveAbsolute`, one validator, one builder, and one electronic-cam wrapper. Add one `R_TRIG` per pulse command. Keep the fixed table ID as:

```iecst
VAR CONSTANT
    c_nCamTableId : MC_CAM_ID := 1;
END_VAR
```

- [ ] **Step 4: Refresh status and power in every scan**

At the start of each call:

```iecst
AxisConveyor.ReadStatus();
AxisCutter.ReadStatus();

fbPowerConveyor(
    Enable := bPowerRequest,
    Enable_Positive := TRUE,
    Enable_Negative := TRUE,
    Override := 100.0,
    BufferMode := MC_Aborting,
    Axis := AxisConveyor);
fbPowerCutter(
    Enable := bPowerRequest,
    Enable_Positive := TRUE,
    Enable_Negative := TRUE,
    Override := 100.0,
    BufferMode := MC_Aborting,
    Axis := AxisCutter);
```

Set `bPowerRequest` TRUE while enabling, running, stopping, decoupling, or returning. Do not remove power until controlled stop/recovery finishes. `stStatus.bPowerEnabled` is TRUE only when both MC_Power `Status` outputs are TRUE.

- [ ] **Step 5: Implement reference, recipe apply, and start states**

Implement these exact transitions:

```text
DISABLED + bEnable edge/level -> POWERING
POWERING + both power Status -> IDLE
IDLE/READY + bReference edge + both StandStill -> REFERENCING_MASTER
REFERENCING_MASTER + master MC_SetPosition.Done -> REFERENCING_SLAVE
REFERENCING_SLAVE + cutter MC_SetPosition.Done -> IDLE, bReferenced := TRUE
IDLE/READY + bApplyRecipe edge + referenced + both axes StandStill + decoupled -> BUILDING_TABLE
BUILDING_TABLE + validator valid -> trigger builder, then LOADING_TABLE
LOADING_TABLE + builder done -> trigger cam load
LOADING_TABLE + cam load and characteristic read done -> validate physical limits, then READY
READY + bStart edge -> CAMMING_IN
CAMMING_IN + cam-in done -> STARTING_MASTER
STARTING_MASTER + MC_MoveVelocity.InVelocity -> RUNNING
```

Reference both axes with `MC_SetPosition(Position := 0.0, Mode := FALSE)`. Before cam-in, require `ABS(AxisCutter.NcToPlc.SetPos) <= stRecipe.lrPositionTolerance`, store both set-position origins, then use the wrapper's relative start mode.

Calculate physical characteristic limits from normalized table values:

```iecst
stStatus.lrCalculatedMaxSlaveVelocity :=
    MAX(ABS(stChar.fSlaveVeloMin), ABS(stChar.fSlaveVeloMax))
    * stRecipe.lrConveyorVelocity;
stStatus.lrCalculatedMaxSlaveAcceleration :=
    MAX(ABS(stChar.fSlaveAccMin), ABS(stChar.fSlaveAccMax))
    * stRecipe.lrConveyorVelocity * stRecipe.lrConveyorVelocity;
stStatus.lrCalculatedMaxSlaveJerk :=
    MAX(ABS(stChar.fSlaveJerkMin), ABS(stChar.fSlaveJerkMax))
    * stRecipe.lrConveyorVelocity * stRecipe.lrConveyorVelocity
    * stRecipe.lrConveyorVelocity;
```

Do not enter READY if any result exceeds its recipe maximum.

- [ ] **Step 6: Implement controlled stop and linear-axis recovery**

Use the accepted order and keep MC_Stop called until it releases:

```text
RUNNING + bStop edge or bEnable FALSE -> STOPPING_MASTER
STOPPING_MASTER: MC_Stop.Execute TRUE until Done
STOPPING_MASTER + Done -> set MC_Stop.Execute FALSE, RELEASING_STOP
RELEASING_STOP + MC_Stop.Busy FALSE -> CAMMING_OUT
CAMMING_OUT + cam-out done -> RETURNING_SLAVE
RETURNING_SLAVE + MC_MoveAbsolute(Position := 0).Done -> READY if enabled, otherwise DISABLED
```

Use recipe recovery velocity, acceleration, deceleration, and jerk with `MC_Aborting`. Never call `MC_MoveAbsolute` while `stStatus.bCamActive` is TRUE. The wrapper's `bDecoupleDone` is the only event that permits the return command.

- [ ] **Step 7: Implement deterministic error and reset behavior**

Create one local fault-latching section that receives `E_CamApplicationError`, `E_CamErrorSource`, and raw `UDINT`. It preserves the first failure until reset. Map each MC instance to the matching enum from `E_CamApplicationError` and source enum.

Rules:

```text
Invalid command in RUNNING, STOPPING, CAMMING_OUT, or RETURNING -> CAM_ERROR_COMMAND_NOT_ALLOWED
Apply without reference -> CAM_ERROR_NOT_REFERENCED
Validator failure -> validator's recipe error
Cam wrapper load/characteristic/cam-in/cam-out error -> matching cam error and wrapper ErrorID
Axis power/reference/move/stop/return/reset error -> matching axis error and MC ErrorID
```

In FAULT, `bStart` and `bApplyRecipe` have no effect. A reset edge is accepted only when both axes report `Status.NotMoving`; pulse both `MC_Reset` instances, reset the cam wrapper, clear the error latch after both reset blocks report Done, clear `bRecipeLoaded`, and return to POWERING if `bEnable` remains TRUE or DISABLED otherwise. Reset never starts motion.

- [ ] **Step 8: Populate process phase and Scope output**

While cam is active or running, calculate:

```iecst
stScope.lrMasterPhase := MODABS(
    AxisConveyor.NcToPlc.SetPos - lrMasterCycleOrigin,
    stRecipe.lrObjectPitch);
lrSlaveCamPosition := AxisCutter.NcToPlc.SetPos - lrSlaveCycleOrigin;
stScope.lrObjectRelativePosition := lrSlaveCamPosition - stScope.lrMasterPhase;
```

Assign process phases by the derived boundaries in ascending order. In constant-speed phases set expected velocity to 0, 150, 100, 50, or 0 using recipe formulas; in polynomial transition/return phases use `AxisCutter.NcToPlc.SetVelo` so the Scope signal stays continuous. Populate all other Scope fields directly from `NcToPlc`, machine status, and error fields.

- [ ] **Step 9: Replace MAIN with wiring only**

`MAIN` declaration and body:

```iecst
PROGRAM MAIN
VAR
    fbCamMachine : FB_CamMachine;
END_VAR

fbCamMachine(
    stCommands := GVL_CamControl.stCommands,
    stRecipe := GVL_CamControl.stRecipe,
    AxisConveyor := GVL_Motion.gAxisConveyor,
    AxisCutter := GVL_Motion.gAxisCutter,
    stStatus => GVL_CamControl.stStatus,
    stScope => GVL_CamControl.stScope);
```

Do not place MC function blocks, phase math, or command edges in MAIN.

- [ ] **Step 10: Run offline tests and compile TwinCAT**

Run the full `unittest` suite, then build with `devenv.com`. Inspect `twincat-build.log` for `error` and `warning` lines even if the command returns zero. Expected: no PLC compile errors and no new warnings from these sources.

- [ ] **Step 11: Commit the runnable PLC application**

```powershell
git commit -m "feat: run electronic cam machine lifecycle"
```

Stage only the machine POU, MAIN, PLC project, and changed structure test. Confirm the user's `cam_case.tsproj` remains unstaged.

---

### Task 6: Document XAE Configuration and Scope Acceptance

**Files:**
- Create: `docs/twincat-operation.md`
- Modify: `tests/test_plc_project_structure.py`

**Interfaces:**
- Consumes: final PLC symbol names and the accepted design.
- Produces: an operator/developer procedure that covers every manual action Codex cannot safely perform.

- [ ] **Step 1: Write the failing documentation test**

Add:

```python
def test_operation_guide_covers_manual_boundaries(self):
    guide = (ROOT / "docs" / "twincat-operation.md").read_text(encoding="utf-8")
    for heading in (
        "TF5050 许可",
        "AXIS_REF 链接",
        "仿真轴配置",
        "实机恢复",
        "操作顺序",
        "TwinCAT Scope",
        "验收值",
        "安全边界",
    ):
        self.assertIn(heading, guide)
    for symbol in (
        "GVL_Motion.gAxisConveyor",
        "GVL_Motion.gAxisCutter",
        "GVL_CamControl.stCommands",
        "GVL_CamControl.stScope",
    ):
        self.assertIn(symbol, guide)
```

- [ ] **Step 2: Run the test and verify RED**

Expected: file-not-found failure for `docs/twincat-operation.md`.

- [ ] **Step 3: Write the manual configuration guide**

The guide must contain these concrete procedures:

1. Open `cam_case.sln` in Visual Studio 2019 with TwinCAT XAE and confirm target/runtime version 3.1.4024.75 or a compatible 3.1.4024 runtime not older than 3.1.4024.55.
2. Under TwinCAT licensing, confirm TF5050 NC Camming is licensed or activate the available trial before downloading.
3. Build the PLC project so `GVL_Motion` symbols exist.
4. Link `GVL_Motion.gAxisConveyor.PlcToNc/NcToPlc` to NC Axis 1 FromPlc/ToPlc and `GVL_Motion.gAxisCutter.PlcToNc/NcToPlc` to NC Axis 2 FromPlc/ToPlc. Confirm no link remains “Unrestored”.
5. For simulation, save a recoverable copy of the existing hardware configuration, unlink the two axes from the EtherCAT drives, set each NC encoder/drive to the available simulation type, activate configuration, and restart Runtime.
6. For real hardware, reverse the simulation changes, restore the existing Drive 1/Drive 2 EtherCAT mappings, confirm scaling, software limits, maximum dynamics, homing, STO, and direction before enabling PLC commands.
7. Online command sequence: `bEnable`, pulse `bReference` after physical alignment, pulse `bApplyRecipe`, confirm `CAM_STATE_READY`, then pulse `bStart`; pulse `bStop` for controlled stop; use `bReset` only when stationary.
8. Scope variables: all fields under `GVL_CamControl.stScope`, plus `gAxisConveyor.NcToPlc.ErrorCode` and `gAxisCutter.NcToPlc.ErrorCode`.
9. Acceptance values: master 100 mm/s; slave 150 mm/s at phase 100–150; slave 100 mm/s at 170–280; slave 50 mm/s at 300–350; Axis 2 at zero by phase 480; continuous 500-to-0 transition.
10. State that `bReference` changes the process coordinate and does not replace mechanical homing; first real run uses reduced velocity/dynamics and an unloaded mechanism.

- [ ] **Step 4: Run tests and perform a clean build**

Run:

```powershell
python -m unittest discover -s tests -v
```

Then delete only the prior `twincat-build.log` if it exists, rebuild with `devenv.com`, and inspect the newly generated log. Do not delete TwinCAT project or user configuration files.

- [ ] **Step 5: Commit the handoff guide**

```powershell
git commit -m "docs: add TwinCAT cam commissioning guide"
```

Stage only `docs/twincat-operation.md` and the changed documentation test.

---

### Task 7: Final Verification and User-Run Runtime Check

**Files:**
- Verify: all files in the File Map
- Verify: `docs/superpowers/specs/2026-08-06-electronic-cam-grinding-design.md`
- Verify: `docs/twincat-operation.md`

**Interfaces:**
- Consumes: completed implementation and the user's XAE/runtime access.
- Produces: recorded evidence that offline tests, PLC compilation, configuration boundaries, and Scope acceptance are all addressed.

- [ ] **Step 1: Verify repository state and protected user change**

Run:

```powershell
git status --short --branch
git diff -- cam_case/cam_case.tsproj
```

Expected: the original user modification to `cam_case.tsproj` remains present and unstaged; no unrelated file is staged.

- [ ] **Step 2: Run all dependency-free automated tests**

```powershell
python -m unittest discover -s tests -v
```

Expected: all tests pass with no skipped test and no third-party package requirement.

- [ ] **Step 3: Run and inspect the TwinCAT build**

Run the Task 2 `devenv.com` command. Then inspect `twincat-build.log` and report the exact build summary. A successful process exit without a successful PLC compilation is not sufficient evidence.

- [ ] **Step 4: Review implementation against every acceptance value**

Confirm in sources and offline oracle:

```text
Point coordinates: (0,0), (20,0), (100,100), (150,175),
                   (170,200), (280,310), (300,325),
                   (350,350), (480,0), (500,0)
Constant velocities: 150 mm/s, 100 mm/s, 50 mm/s
Relative cut strokes: 0 -> 25 mm and 25 -> 0 mm
Return deadline: phase 480 mm
Cycle seam: position 0 and slope 0 on both sides
```

- [ ] **Step 5: Hand off the bounded runtime procedure**

If Codex cannot activate the NC configuration or control the licensed runtime, ask the user to execute `docs/twincat-operation.md` through the simulation Scope check and return:

- a screenshot or exported Scope trace,
- final `GVL_CamControl.stStatus`,
- Axis 1 and Axis 2 NC ErrorCode values,
- the phase and velocity values at both cut intervals.

Do not claim runtime acceptance until this evidence is available. Code/compile completion and runtime commissioning must be reported separately.
