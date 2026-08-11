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
        self.assertIn("CAM_STATE_POSITIONING_SLAVE", text)
        self.assertNotIn("CAM_STATE_REFERENCING_SLAVE", text)
