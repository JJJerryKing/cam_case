from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
PLC = ROOT / "cam_case" / "Untitled1"


def source(relative: str) -> str:
    path = PLC / relative
    ET.parse(path)
    return path.read_text(encoding="utf-8")


class PhysicalCamPlcStructureTests(unittest.TestCase):
    def test_recipe_contains_physical_defaults(self):
        text = source("DUTs/ST_CamRecipe.TcDUT")
        for token in (
            "lrObjectPitch : LREAL := 63.7",
            "lrConveyorVelocity : LREAL := 5.0",
            "lrCutVelocity : LREAL := 3.0",
            "lrObjectEdgeLength : LREAL := 3.0",
            "lrInterEdgeTransitionDistance : LREAL := 4.0",
            "lrExitTransitionDistance : LREAL := 2.0",
            "lrCutterReadyPosition : LREAL := 17.0",
            "lrMasterReferenceRawPosition : LREAL := -0.6",
            "lrReturnStartPhase : LREAL := 17.0",
            "lrReturnEndPhase : LREAL := 27.0",
        ):
            self.assertIn(token, text)

    def test_phases_have_no_synchronous_plateau(self):
        text = source("DUTs/E_CamProcessPhase.TcDUT")
        self.assertIn("CAM_PHASE_INTER_EDGE_TRANSITION", text)
        self.assertIn("CAM_PHASE_EXIT", text)
        self.assertNotIn("CAM_PHASE_SYNCHRONOUS", text)

    def test_machine_state_positions_slave_instead_of_zeroing_it(self):
        text = source("DUTs/E_CamMachineState.TcDUT")
        self.assertIn("CAM_STATE_POSITIONING_MASTER", text)
        self.assertIn("CAM_STATE_POSITIONING_SLAVE", text)
        self.assertNotIn("CAM_STATE_REFERENCING_SLAVE", text)
        positioning_master = text.index("CAM_STATE_POSITIONING_MASTER")
        referencing_master = text.index("CAM_STATE_REFERENCING_MASTER")
        self.assertLess(positioning_master, referencing_master)

    def test_validator_uses_negative_slopes_and_direct_transition(self):
        text = source("POUs/Cam/FB_CamRecipeValidator.TcPOU")
        first = text.index("stDerived.lrFirstSlope")
        second = text.index("stDerived.lrSecondSlope")
        self.assertIn(
            "-(1.0 - stRecipe.lrCutVelocity", text[first:second]
        )
        self.assertIn(
            "-(1.0 + stRecipe.lrCutVelocity", text[second:]
        )
        self.assertIn("lrInterEdgeTransitionEnd", text)
        self.assertNotIn("lrClearanceRelative", text)

    def test_builder_contains_approved_eight_rows(self):
        text = source("POUs/Cam/FB_CamProfileBuilder.TcPOU")
        self.assertIn("ARRAY[1..8] OF MC_MotionFunctionPoint", text)
        self.assertIn("stCamRef.NoOfRows := 8", text)
        self.assertIn("stDerived.lrExitEndY", text)
        self.assertNotIn("aPoints[9]", text)

    def test_machine_references_only_master_and_positions_cutter_at_17(self):
        text = source("POUs/Cam/FB_CamMachine.TcPOU")
        self.assertIn("lrMasterReferenceRawPosition", text)
        self.assertIn("lrCutterReadyPosition", text)
        self.assertIn("fbSetPositionConveyor", text)
        self.assertIn("Position := 0.0", text)
        self.assertNotIn("fbSetPositionCutter", text)
        self.assertIn("CAM_STATE_POSITIONING_SLAVE", text)

    def test_machine_moves_conveyor_to_raw_reference_before_zeroing(self):
        text = source("POUs/Cam/FB_CamMachine.TcPOU")
        self.assertIn("fbMoveConveyorToReference : MC_MoveAbsolute", text)
        self.assertIn("CAM_STATE_POSITIONING_MASTER:", text)
        self.assertIn("bConveyorCoordinateZeroed : BOOL", text)
        self.assertIn("lrMasterReferencePhase : LREAL", text)
        self.assertIn("lrMasterReferenceTarget : LREAL", text)
        self.assertIn("IF bConveyorCoordinateZeroed THEN", text)
        self.assertIn("lrMasterReferencePhase := 0.0", text)
        self.assertIn("lrMasterReferencePhase :=", text)
        self.assertIn("stRecipe.lrMasterReferenceRawPosition", text)
        self.assertIn("MODABS(", text)
        self.assertIn("stRecipe.lrObjectPitch / 2.0", text)
        self.assertIn("Position := lrMasterReferenceTarget", text)
        self.assertNotIn(
            "Position := stRecipe.lrMasterReferenceRawPosition", text
        )
        self.assertIn("Velocity := stRecipe.lrConveyorVelocity", text)
        self.assertIn("Acceleration := stRecipe.lrMasterAcceleration", text)
        self.assertIn("Deceleration := stRecipe.lrMasterDeceleration", text)
        self.assertIn("Jerk := stRecipe.lrMasterJerk", text)
        self.assertIn("AxisConveyor.NcToPlc.SetPos", text)
        self.assertIn("AxisConveyor.NcToPlc.ActPos", text)
        self.assertGreaterEqual(text.count("- lrMasterReferenceTarget"), 2)
        self.assertIn(
            "stSetPositionConveyorOptions.ClearPositionLag := TRUE", text
        )
        self.assertIn("bConveyorCoordinateZeroed := TRUE", text)
        self.assertNotIn("bConveyorCoordinateZeroed := FALSE", text)
        self.assertIn("CAM_ERROR_RECIPE_PITCH", text)
        positioning = text.index("CAM_STATE_POSITIONING_MASTER:")
        zeroing = text.index("CAM_STATE_REFERENCING_MASTER:")
        slave = text.index("CAM_STATE_POSITIONING_SLAVE:")
        self.assertLess(positioning, zeroing)
        self.assertLess(zeroing, slave)
        self.assertNotIn("fbSetPositionCutter", text)

    def test_machine_couples_before_starting_master(self):
        text = source("POUs/Cam/FB_CamMachine.TcPOU")
        camming = text.index("CAM_STATE_CAMMING_IN:")
        starting = text.index("CAM_STATE_STARTING_MASTER:")
        self.assertLess(camming, starting)
        self.assertIn(
            "-(lrSlaveCamPosition + stScope.lrMasterPhase)", text
        )

    def test_machine_reports_reversed_cut_velocities_and_physical_object_sign(self):
        text = source("POUs/Cam/FB_CamMachine.TcPOU")
        self.assertIn(
            "-(stActiveRecipe.lrConveyorVelocity - stActiveRecipe.lrCutVelocity)",
            text,
        )
        self.assertIn(
            "-(stActiveRecipe.lrConveyorVelocity + stActiveRecipe.lrCutVelocity)",
            text,
        )
        self.assertIn(
            "-(lrSlaveCamPosition + stScope.lrMasterPhase)",
            text,
        )

    def test_cam_wrapper_stays_relative(self):
        text = source("POUs/Cam/FB_ElectronicCam.TcPOU")
        self.assertIn("StartMode := MC_STARTMODE_RELATIVE", text)
        self.assertIn("MasterAbsolute := FALSE", text)
        self.assertIn("SlaveAbsolute := FALSE", text)
