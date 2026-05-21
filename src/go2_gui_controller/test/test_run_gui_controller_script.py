from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[3]
SCRIPT = ROOT / "scripts" / "run_gui_controller.sh"


class RunGuiControllerScriptTest(unittest.TestCase):
    def test_uses_project_workspace_only(self):
        script = SCRIPT.read_text(encoding="utf-8")
        legacy_workspace_name = "go2_gui_controller" + "_ws"
        legacy_workspace_var = "GUI" + "_WS_DIR"

        self.assertNotIn(legacy_workspace_name, script)
        self.assertNotIn(legacy_workspace_var, script)
        self.assertNotIn("PARENT_DIR", script)
        self.assertIn('${PROJECT_DIR}/install/setup.bash', script)
        self.assertIn("Build it from ${PROJECT_DIR}", script)


if __name__ == "__main__":
    unittest.main()
