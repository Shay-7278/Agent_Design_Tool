"""GUI opening creation/resize/undo plus real full solver, no API charges."""
from pathlib import Path
import tempfile
import numpy as np
from PySide6 import QtWidgets
from .ai_app import MainWindow
from .smoke_ai import pump
from .test_openings import window as opening
from .openings import resized, geometry
from .model import NurbsModel


def main():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    app.setStyle("Fusion")
    w = MainWindow()
    try:
        w.show()
        app.processEvents()
        before = w.editor.cps.copy()
        w.session.execute("set_opening", {"expected_revision": 0, "opening": opening()})
        w.draw_openings()
        assert len(w.opening_widgets) == 4
        assert len(w.plotter.widgets.sphere_widgets) == 40
        # Drive the actual end-interaction callback, including its visual offset.
        widget = w.opening_widgets[2]
        center = np.array(widget.GetCenter())
        widget.SetCenter(center+[40, 0, 0])
        widget.InvokeEvent("EndInteractionEvent")
        assert w.editor.model.openings[0]["width_mm"] > 900
        w.editor.undo()
        w.draw_openings()
        np.testing.assert_array_equal(w.editor.cps, before)
        w.editor.undo()
        w.draw_openings()
        assert not w.editor.model.openings and len(w.plotter.widgets.sphere_widgets) == 36
        w.editor.redo()
        w.draw_openings()
        o = w.editor.model.openings[0]
        _, corners, _ = geometry(w.editor.model, o, {})
        candidate = resized(w.editor.model, o, 2, corners[2]+[70, 0, 25], {})
        w.session.execute("set_opening", {"expected_revision": w.session.revision, "opening": candidate})
        assert w.editor.model.openings[0]["width_mm"] > o["width_mm"]
        w.editor.undo()
        w.draw_openings()
        print("PASS opening insertion, four handles, resize projection and undo/redo; NURBS unchanged", flush=True)
        with tempfile.TemporaryDirectory() as d:
            path = Path(d)/"opening.3dm"
            w.editor.model.save(path)
            assert NurbsModel.load(path).openings == w.editor.model.openings
            w.session.results_dir = Path(d)/"results"
            started = w.start_analysis("full")
            assert started.get("status") == "running", started
            pump(app, lambda: w.session.running is None, timeout=180)
            assert w.session.last_error is None, w.session.last_error
            r = w.session.result
            assert not r["fatal"], r["fatal"]
            assert r["l0_pass"], r["report"]
            assert r["l1_state"] == "solved", r["report"]
            assert r["stages"] is not None
            assert sum(b["kind"] == "lintel" for b in r["bricks"]) == 1
            lintel = next(b for b in r["bricks"] if b["kind"] == "lintel")
            bearings = [i for i in r["interfaces"] if i["upper"] == [lintel["course"], lintel["index"]]]
            assert any(i["centroid"][0] < 1050 for i in bearings), "CRA missed left bearing"
            assert any(i["centroid"][0] > 1950 for i in bearings), "CRA missed right bearing"
            assert w.session.analysis_status() == "current"
            w.session.execute("remove_opening", {"expected_revision": w.session.revision, "opening_id": o["id"]})
            assert w.session.analysis_status() == "outdated"
            w.editor.undo()
            assert w.session.analysis_status() == "current"
            w.draw_openings()
            app.processEvents()
            w.grab().save(str(Path(__file__).resolve().parents[2]/"opening_preview.png"))
            print("PASS saved openings, real L0/L1/construction checks, lintel load path and analysis freshness", flush=True)
            print({"elements": len(r["bricks"]), "custom_pieces": len(r["custom_piece_schedule"]), "l1": r["l1_state"], "overall": r["ok"]}, flush=True)
        print("OPENINGS_SMOKE_OK", flush=True)
    finally:
        w.close()


if __name__ == "__main__": main()
