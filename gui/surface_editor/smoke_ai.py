"""Real Qt/VTK integration smoke test with a scripted provider, no API charges.

Run from gui/: python -m surface_editor.smoke_ai
Includes a real geometry-only solver run. No source .3dm is overwritten.
"""
import hashlib
from pathlib import Path
import tempfile
import time
from unittest.mock import patch

import numpy as np
from PySide6 import QtWidgets

from . import ai_app
from .model import NurbsModel
from .providers import Reply, ToolCall


class ScriptedProvider:
    def __init__(self, *args): self.step = 0
    def start(self, history): pass
    def request(self, system, tools):
        if self.step == 0:
            return Reply("", [ToolCall("read", "get_surface_state", {})], [])
        if self.step == 1:
            return Reply("", [ToolCall("edit", "edit_surface", {"expected_revision": self.revision,
                "operations": [{"kind": "pull", "center_uv": [.5, .5], "radius_uv": .25,
                                "vector": [0, 35, 0], "keep_boundary": True},
                               {"kind": "taper", "axis": "height", "factor": .98}]})], [])
        return Reply("Applied a local bulge and a gentle taper. You can drag the handles or undo this change.", [], [])
    def accept(self, reply, results):
        assert "error" not in results[0], results
        if self.step == 0:
            self.revision = results[0]["revision"]
        self.step += 1


def pump(app, done, timeout=120):
    deadline = time.monotonic() + timeout
    while not done():
        app.processEvents()
        if time.monotonic() > deadline:
            raise TimeoutError("GUI smoke check timed out")
        time.sleep(.01)
    app.processEvents()


def main():
    root = Path(__file__).resolve().parents[2]
    source = root / "Reference Surface_1.3dm"
    original_hash = hashlib.sha256(source.read_bytes()).hexdigest()
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    app.setStyle("Fusion")
    window = ai_app.MainWindow(source)
    try:
        window.show()
        app.processEvents()
        before = window.editor.cps.copy()
        with patch.object(ai_app, "Provider", ScriptedProvider):
            window.input.setPlainText("Add a small bulge in the middle and gently narrow the top.")
            window.send_message()
            pump(app, lambda: not window.agent_busy)
        assert window.session.revision == 1
        assert not np.allclose(before, window.editor.cps)
        assert "Applied a local bulge" in window.chat.toPlainText()
        ed = window.editor
        np.testing.assert_allclose(ed.base_line.points, ed.model.evaluate(ed.res, ed.res)[ed.base_index])
        assert np.ptp(ed.base_line.points[:, 2]) < 1e-6  # base edge found, not a side edge
        window.plotter.view_xy()
        ed.toggle_handles()
        assert len(window.plotter.widgets.sphere_widgets) == 0 and not ed.cp_actor.GetVisibility()
        ed.toggle_handles()
        assert len(window.plotter.widgets.sphere_widgets) == ed.n * ed.m and ed.cp_actor.GetVisibility()
        print("PASS chat worker -> tools on GUI thread -> scene update", flush=True)
        window.editor.undo()
        np.testing.assert_allclose(window.editor.cps, before)
        assert window.session.revision == 2
        point = before.reshape(-1, 3)[7].copy() + [0, 10, 0]
        window.editor.on_drag(point, 7, window.plotter.widgets.sphere_widgets[7])
        assert window.session.revision == 3
        window.editor.undo()
        np.testing.assert_allclose(window.editor.cps, before)
        print("PASS shared manual/AI undo and revisions", flush=True)
        surface_before = window.editor.model.evaluate(40, 40)
        window.editor.add_control_points(0, 3)
        assert window.editor.cps.shape == (15, 6, 3)
        assert len(window.plotter.widgets.sphere_widgets) == 15 * 6
        assert np.abs(window.editor.model.evaluate(40, 40) - surface_before).max() < 1e-6
        moved = window.editor.cps.reshape(-1, 3)[20] + [0, 10, 0]
        window.editor.on_drag(moved, 20, window.plotter.widgets.sphere_widgets[20])
        np.testing.assert_allclose(window.editor.cps[divmod(20, 6)], moved)
        window.editor.undo()
        window.editor.undo()
        assert window.editor.cps.shape == (6, 6, 3)
        assert len(window.plotter.widgets.sphere_widgets) == 36
        np.testing.assert_allclose(window.editor.cps, before)
        print("PASS add control points: handles rebuilt, shape kept, drag and undo", flush=True)
        with tempfile.TemporaryDirectory() as tmp:
            saved = window.editor.model.save(Path(tmp) / "edited.3dm", backup=False)
            np.testing.assert_allclose(NurbsModel.load(saved).control_points(), before)
            window.session.results_dir = Path(tmp) / "results"
            started = window.start_analysis("fast")
            assert started.get("status") == "running", started
            # Force a manual edit while analysis runs: its result must be stale.
            window.editor.on_drag(point, 7, window.plotter.widgets.sphere_widgets[7])
            pump(app, lambda: window.session.running is None, timeout=180)
            assert window.session.last_error is None, window.session.last_error
            assert window.session.analysis_status() == "outdated"
            assert window.session.summary()["equilibrium_pass"] is None
            window.editor.undo()
            assert window.session.analysis_status() == "current"
            print("PASS real solver, frozen snapshot, freshness after manual edit/undo", flush=True)
            window.grab().save(str(root / "ai_gui_preview.png"))
        assert hashlib.sha256(source.read_bytes()).hexdigest() == original_hash
        print("PASS save round-trip; original surface unchanged", flush=True)
        print("GUI_SMOKE_OK", flush=True)
    finally:
        window.close()


if __name__ == "__main__":
    main()
