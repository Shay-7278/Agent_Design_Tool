"""Real GUI regression checks for upright orbit, undo and topology changes."""
from pathlib import Path
import numpy as np
from PySide6 import QtCore, QtWidgets, QtTest

from .ai_app import MainWindow
from .navigation import orbit
from .topology import sample
from .session import surface_hash


def main():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    app.setStyle("Fusion")
    window = MainWindow()
    try:
        window.show()
        window.activateWindow()
        app.processEvents()
        e, s = window.editor, window.session
        viewport = window.plotter.interactor
        assert e.model.shape == (6, 6)
        assert np.ptp(e.cps[..., 1]) == 0
        assert not e.wall_actors and s.result is None
        before = e.cps.copy()
        print("PASS default flat wall, no unrelated brick overlay", flush=True)

        widget = window.plotter.widgets.sphere_widgets[7]
        widget.InvokeEvent("StartInteractionEvent")
        for amount in (10, 20, 30):
            e.on_drag(before.reshape(-1, 3)[7]+[0, amount, 0], 7, widget)
        widget.InvokeEvent("EndInteractionEvent")
        assert len(e.undo_stack) == 1
        edited = e.cps.copy()
        viewport.setFocus()
        app.processEvents()
        QtTest.QTest.keyClick(viewport, QtCore.Qt.Key.Key_Z, QtCore.Qt.KeyboardModifier.ControlModifier)
        app.processEvents()
        np.testing.assert_allclose(e.cps, before)
        QtTest.QTest.keyClick(viewport, QtCore.Qt.Key.Key_Y, QtCore.Qt.KeyboardModifier.ControlModifier)
        app.processEvents()
        np.testing.assert_allclose(e.cps, edited)
        print("PASS Ctrl+Z/Ctrl+Y; an entire multi-event drag is one undo", flush=True)

        initial_hash = surface_hash(e.model)
        window.input.setFocus()
        QtTest.QTest.keyClicks(window.input, "some text")
        QtTest.QTest.keyClick(window.input, QtCore.Qt.Key.Key_Z, QtCore.Qt.KeyboardModifier.ControlModifier)
        app.processEvents()
        assert surface_hash(e.model) == initial_hash
        assert not window.input.toPlainText()
        print("PASS text-field undo does not change the surface", flush=True)

        shape_before = sample(e.model._srf)
        report = s.execute("rebuild_surface", {"expected_revision": s.revision, "count_u": 10, "count_v": 8,
                                              "mode": "refine", "tolerance_mm": .01})
        assert e.model.shape == (10, 8)
        assert len(window.plotter.widgets.sphere_widgets) == 80
        np.testing.assert_allclose(sample(e.model._srf), shape_before, atol=1e-8)
        e.undo()
        assert e.model.shape == (6, 6) and len(window.plotter.widgets.sphere_widgets) == 36
        np.testing.assert_allclose(e.cps, edited)
        e.redo()
        assert e.model.shape == (10, 8)
        point = e.cps[-1, -1].copy()
        e.on_drag(point+[0, 5, 0], 79, window.plotter.widgets.sphere_widgets[79])
        np.testing.assert_allclose(e.cps[-1, -1], point+[0, 5, 0])
        e.undo()
        print("PASS AI refine, regenerated handles, topology undo/redo and subsequent drag", flush=True)

        s.execute("new_flat_surface", {"expected_revision": s.revision, "width_mm": 2400, "height_mm": 1200, "count_u": 8, "count_v": 7})
        assert e.model.shape == (8, 7)
        s.execute("rebuild_surface", {"expected_revision": s.revision, "count_u": 4, "count_v": 4,
                                     "mode": "rebuild", "tolerance_mm": .001})
        assert e.model.shape == (4, 4)
        e.undo()
        assert e.model.shape == (8, 7)
        e.undo()
        assert e.model.shape == (10, 8)
        e.reset()
        assert e.model.shape == (6, 6)
        np.testing.assert_allclose(e.cps, before)
        print("PASS new plane, reducing grid, undo and reset across topologies", flush=True)

        camera = window.plotter.camera
        camera.up = (0, 0, 1)
        radius = np.linalg.norm(np.array(camera.position)-camera.focal_point)
        z = camera.position[2]
        orbit(camera, 100, 0)
        assert abs(camera.position[2]-z) < 1e-8
        for dy in (10000, -10000, 10000, 50):
            orbit(camera, 80, dy)
            np.testing.assert_allclose(camera.up, [0, 0, 1])
            assert abs(np.dot(camera.direction, [0, 0, 1])) < .9999
            assert abs(np.linalg.norm(np.array(camera.position)-camera.focal_point)-radius) < 1e-7
        window.reset_view()
        pos = np.array(camera.position)
        viewport.setFocus()
        QtTest.QTest.mousePress(viewport, QtCore.Qt.MouseButton.RightButton, pos=QtCore.QPoint(250, 200))
        QtTest.QTest.mouseMove(viewport, QtCore.QPoint(350, 230))
        QtTest.QTest.mouseRelease(viewport, QtCore.Qt.MouseButton.RightButton, pos=QtCore.QPoint(350, 230))
        app.processEvents()
        assert not np.allclose(pos, camera.position)
        np.testing.assert_allclose(camera.up, [0, 0, 1])
        np.testing.assert_allclose(e.cps, before)
        print("PASS real right-drag navigation; horizontal Z orbit and pole clamps", flush=True)
        window.reset_view()
        window.input.clear()
        app.processEvents()
        window.grab().save(str(Path(__file__).resolve().parents[2] / "controls_preview.png"))
        print("CONTROLS_SMOKE_OK", flush=True)
    finally:
        window.close()


if __name__ == "__main__":
    main()
