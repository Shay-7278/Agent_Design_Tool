"""Regression: orbit cannot move opening geometry; A still generates."""
import numpy as np
from PySide6 import QtCore, QtWidgets, QtTest
from .ai_app import MainWindow
from .navigation import orbit
from .test_openings import window


def main():
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    w = MainWindow()
    try:
        w.show()
        w.activateWindow()
        app.processEvents()
        w.session.execute("set_opening", {"expected_revision": 0, "opening": window()})
        w.draw_openings()
        handles = np.array([x.GetCenter() for x in w.opening_widgets])
        mesh = w.plotter.actors["opening_volume"].mapper.dataset.points.copy()
        for dx, dy in ((90, 20), (-200, 40), (60, -70)):
            orbit(w.plotter.camera, dx, dy)
            w.draw_openings()
            np.testing.assert_allclose([x.GetCenter() for x in w.opening_widgets], handles)
            np.testing.assert_allclose(w.plotter.actors["opening_volume"].mapper.dataset.points, mesh)
        print("PASS fixed opening volume and handles across camera orbits", flush=True)
        calls = []
        w.start_analysis = lambda mode: calls.append(mode)
        viewport = w.plotter.interactor
        viewport.setFocus()
        app.processEvents()
        QtTest.QTest.keyClick(viewport, QtCore.Qt.Key.Key_A)
        app.processEvents()
        assert calls == ["full"], calls
        print("PASS A invokes full generation/analysis", flush=True)
    finally:
        w.close()


if __name__ == "__main__": main()
