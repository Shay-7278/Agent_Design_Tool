"""Qt desktop chat + PyVista viewport. Launch: python -m surface_editor --ai."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import queue
import sys
import threading
import uuid
import numpy as np
import pyvista as pv

os.environ.setdefault("QT_API", "pyside6")
from PySide6 import QtCore, QtGui, QtWidgets
from pyvistaqt import QtInteractor

from .agent import portable_turn, run_agent
from .editor import SurfaceEditor, default_file, _project_root
from .providers import ENV_KEYS, URLS, Provider, ProviderConfig
from .session import DesignSession, run_analysis
from .navigation import UprightNavigation
from .openings import geometry as opening_geometry, resized as resized_opening


class Bridge(QtCore.QObject):
    tool = QtCore.Signal(object)
    audit = QtCore.Signal(object)
    answer = QtCore.Signal(object)
    analysis_done = QtCore.Signal(object)


class EmbeddedEditor(SurfaceEditor):
    def toggle_solver(self):
        super().toggle_solver()
        if hasattr(self, "window"):
            self.window.solver.setCurrentText(self.solver)

    def start_refit(self):
        if hasattr(self, "window"):
            self.window.start_analysis("full")

    def save(self):
        if hasattr(self, "window"):
            self.window.save_as()


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, path=None, result_path=None):
        super().__init__()
        self.setWindowTitle("Surface Studio — AI design assistant")
        self.resize(1560, 980)
        self.history = []
        self.agent_busy = False
        self.cancelled = threading.Event()
        self.closed = False
        self.opening_widgets = []
        self.opening_refresh = QtCore.QTimer(self)
        self.opening_refresh.setSingleShot(True)
        self.opening_refresh.timeout.connect(self.draw_openings)
        self.bridge = Bridge()
        self.bridge.tool.connect(self._execute_tool)
        self.bridge.audit.connect(self._audit)
        self.bridge.answer.connect(self._answer)
        self.bridge.analysis_done.connect(self._analysis_done)

        splitter = QtWidgets.QSplitter()
        self.setCentralWidget(splitter)
        self.plotter = QtInteractor(splitter)
        splitter.addWidget(self.plotter.interactor)
        panel = QtWidgets.QWidget()
        panel.setMinimumWidth(390)
        panel.setMaximumWidth(620)
        layout = QtWidgets.QVBoxLayout(panel)
        title = QtWidgets.QLabel("Design assistant")
        title.setStyleSheet("font-size: 23px; font-weight: 600; padding: 8px 0;")
        layout.addWidget(title)
        hint = QtWidgets.QLabel("Describe a change, drag the handles, or ask about a wall analysis.")
        hint.setWordWrap(True)
        layout.addWidget(hint)

        form = QtWidgets.QFormLayout()
        self.provider = QtWidgets.QComboBox()
        self.provider.addItems(list(URLS))
        self.model = QtWidgets.QLineEdit(os.getenv("AI_MODEL", ""))
        self.model.setPlaceholderText("Model ID from your API account")
        self.endpoint = QtWidgets.QLineEdit()
        self.key = QtWidgets.QLineEdit()
        self.key.setEchoMode(QtWidgets.QLineEdit.EchoMode.Password)
        form.addRow("Provider", self.provider)
        form.addRow("Model", self.model)
        form.addRow("API base URL", self.endpoint)
        form.addRow("API key", self.key)
        layout.addLayout(form)
        self.provider.currentTextChanged.connect(self._provider_changed)
        self._provider_changed(self.provider.currentText())
        privacy = QtWidgets.QLabel("Chat sends prompts, control points and requested analysis summaries to the selected API. Keys stay in memory.")
        privacy.setWordWrap(True)
        privacy.setStyleSheet("color: #64748b; font-size: 11px;")
        layout.addWidget(privacy)

        self.analysis_label = QtWidgets.QLabel()
        self.analysis_label.setWordWrap(True)
        self.analysis_label.setStyleSheet("background: #e8eef5; color: #17324d; border-radius: 6px; padding: 10px;")
        layout.addWidget(self.analysis_label)
        controls = QtWidgets.QHBoxLayout()
        self.solver = QtWidgets.QComboBox()
        self.solver.addItems(["rbe", "cra_penalty"])
        self.fast = QtWidgets.QPushButton("Geometry check")
        self.full = QtWidgets.QPushButton("Generate + analyze (A)")
        controls.addWidget(self.solver)
        controls.addWidget(self.fast)
        controls.addWidget(self.full)
        layout.addLayout(controls)

        self.chat = QtWidgets.QTextBrowser()
        self.chat.setOpenExternalLinks(False)
        layout.addWidget(self.chat, 1)
        self.input = QtWidgets.QPlainTextEdit()
        self.input.setPlaceholderText("e.g. Narrow the top, add a gentle central bulge, and keep the boundary fixed.")
        self.input.setMaximumHeight(105)
        layout.addWidget(self.input)
        buttons = QtWidgets.QHBoxLayout()
        self.send = QtWidgets.QPushButton("Send")
        self.send.setStyleSheet("background: #245d8f; color: white; padding: 8px;")
        self.stop = QtWidgets.QPushButton("Stop AI")
        self.stop.setEnabled(False)
        self.clear = QtWidgets.QPushButton("Clear chat")
        for b in (self.send, self.stop, self.clear):
            buttons.addWidget(b)
        layout.addLayout(buttons)
        splitter.addWidget(panel)
        splitter.setSizes([1060, 500])

        self.editor = EmbeddedEditor(path, result_path=result_path, plotter=self.plotter)
        self.editor.window = self
        for slider in self.editor.sliders:
            rep = slider.GetRepresentation()
            rep.SetTitleHeight(0.018)
            rep.SetLabelHeight(0.015)
            rep.SetSliderWidth(0.025)
            rep.SetTubeWidth(0.012)
        self.session = DesignSession(self.editor, _project_root() / "results")
        self.session.changed = self.refresh_status
        self.session.start_analysis = self.start_analysis
        self.plotter.add_axes()
        self.plotter.camera_position = "iso"
        self.navigation = UprightNavigation(self.plotter)
        self.plotter.set_background("#f4f7fa")
        # The embedded window supplies its own save/analyze controls.
        self.plotter.add_text("Right-drag: orbit · Shift+right-drag: pan · Wheel: zoom\nDrag handles to edit · Ctrl+Z: undo · Ctrl+Y: redo",
                              name="help", position="upper_left", font_size=10, color="#243b53")
        toolbar = self.addToolBar("Design")
        toolbar.setMovable(False)
        for label, callback in (("Undo (Ctrl+Z)", self.editor.undo), ("Redo (Ctrl+Y)", self.editor.redo),
                                ("Window…", self.opening_dialog),
                                ("New flat wall…", self.new_flat_dialog), ("Rebuild grid…", self.rebuild_dialog),
                                ("Reset surface", self.editor.reset),
                                ("Toggle wall", self.editor.toggle_wall), ("Reset view", self.reset_view),
                                ("Save surface as…", self.save_as), ("Analysis details", self.show_analysis)):
            action = toolbar.addAction(label)
            action.triggered.connect(lambda checked=False, fn=callback: fn())
        # Restrict geometry shortcuts to the viewport. Text boxes keep text undo.
        self.geometry_shortcuts = []
        for key, callback in (("Ctrl+Z", self.editor.undo), ("Ctrl+Y", self.editor.redo), ("Ctrl+Shift+Z", self.editor.redo)):
            shortcut = QtGui.QShortcut(QtGui.QKeySequence(key), self.plotter.interactor)
            shortcut.setContext(QtCore.Qt.ShortcutContext.WidgetWithChildrenShortcut)
            shortcut.activated.connect(callback)
            self.geometry_shortcuts.append(shortcut)
        self.solver.currentTextChanged.connect(self._solver_changed)
        self.fast.clicked.connect(lambda: self.start_analysis("fast"))
        self.full.clicked.connect(lambda: self.start_analysis("full"))
        self.send.clicked.connect(self.send_message)
        self.stop.clicked.connect(self.stop_agent)
        self.clear.clicked.connect(self.clear_chat)
        shortcut = QtGui.QShortcut(QtGui.QKeySequence("Ctrl+Return"), self)
        shortcut.activated.connect(self.send_message)
        self.append("Assistant", "Ready. Manual editing and analysis work without an API key. For chat, choose a provider, enter its model ID and key, then describe your design. Ctrl+Enter sends.")
        self.refresh_status()

    def opening_dialog(self):
        existing = self.editor.model.openings
        data = dict(existing[0]) if existing else {"id": "window-"+uuid.uuid4().hex[:8], "center_fraction": .5,
            "width_mm": 900, "height_mm": 600, "sill_mm": 500, "bearing_mm": 190,
            "min_piece_mm": 50, "lintel": True, "allow_custom": True}
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("Window opening — separate from the NURBS")
        layout = QtWidgets.QFormLayout(dialog)
        controls = {}
        for key, label, lo, hi in (("center_fraction", "Position along wall (0–1)", 0, 1),
                                  ("width_mm", "Width along wall", 50, 100000),
                                  ("height_mm", "Requested height", 50, 100000),
                                  ("sill_mm", "Requested sill above base", 0, 100000),
                                  ("bearing_mm", "Lintel bearing each end", 50, 2000),
                                  ("min_piece_mm", "Minimum custom brick length", 20, 190)):
            control = QtWidgets.QDoubleSpinBox()
            control.setRange(lo, hi)
            control.setDecimals(3 if key == "center_fraction" else 1)
            control.setValue(data[key])
            if key != "center_fraction": control.setSuffix(" mm")
            controls[key] = control
            layout.addRow(label, control)
        for key, label in (("lintel", "Assume concealed lintel (keeps brick courses)"), ("allow_custom", "Allow custom jamb pieces if full/half cannot fit")):
            control = QtWidgets.QCheckBox(label)
            control.setChecked(data[key])
            controls[key] = control
            layout.addRow(control)
        note = QtWidgets.QLabel("Sill rounds down and head rounds up to whole courses. Effective dimensions are reported after applying. Lintel capacity is not checked. Cyan corner handles resize the opening; numeric fields set its position precisely.")
        note.setWordWrap(True)
        layout.addRow(note)
        error = QtWidgets.QLabel()
        error.setWordWrap(True)
        layout.addRow(error)
        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.StandardButton.Ok | QtWidgets.QDialogButtonBox.StandardButton.Cancel)
        layout.addRow(buttons)
        buttons.rejected.connect(dialog.reject)
        def apply():
            candidate = {"id": data["id"]}
            for key, control in controls.items():
                candidate[key] = control.isChecked() if isinstance(control, QtWidgets.QCheckBox) else control.value()
            try:
                r = self.session.execute("set_opening", {"expected_revision": self.session.revision, "opening": candidate})
            except Exception as exc:
                error.setText(str(exc))
                return
            self.report_opening(r)
            dialog.accept()
        buttons.accepted.connect(apply)
        if existing:
            delete = QtWidgets.QPushButton("Remove opening")
            layout.addRow(delete)
            def remove():
                self.session.execute("remove_opening", {"expected_revision": self.session.revision, "opening_id": data["id"]})
                dialog.accept()
            delete.clicked.connect(remove)
        dialog.exec()

    def report_opening(self, report):
        o = report["opening"]
        self.append("Opening", f"{o['width_mm']:.1f} mm wide. Effective sill {o['effective_sill_mm']:.1f} mm; head {o['effective_head_mm']:.1f} mm; height {o['effective_height_mm']:.1f} mm after course alignment. NURBS unchanged.")

    def draw_openings(self):
        if self.closed: return
        for widget in self.opening_widgets:
            widget.Off()
            if widget in self.plotter.widgets.sphere_widgets:
                self.plotter.widgets.sphere_widgets.remove(widget)
        self.opening_widgets = []
        self.plotter.remove_actor("opening_boundary", render=False)
        self.plotter.remove_actor("opening_volume", render=False)
        if not self.editor.model.openings:
            self.plotter.render()
            return
        o = self.editor.model.openings[0]
        params = self.session.result.get("params", {}) if self.session.result else {}
        try:
            effective, corners, perimeter = opening_geometry(self.editor.model, o, params)
        except Exception as exc:
            self.statusBar().showMessage("Opening invalid for current surface: " + str(exc))
            return
        radius = max(np.linalg.norm(np.ptp(self.editor.cps, axis=(0, 1)))*.009, 8)
        # World-fixed exclusion extrusion; orbiting never changes its geometry.
        shift = self.session.frame[:, 1]*(float(params.get("brickW", 90)) + radius)
        if np.allclose(perimeter[0], perimeter[-1]): perimeter = perimeter[:-1]
        n = len(perimeter)
        points = np.concatenate([perimeter-shift, perimeter+shift])
        faces = []
        for i in range(n):
            j = (i+1) % n
            faces.extend([4, i, j, n+j, n+i])
        faces.extend([n, *range(n-1, -1, -1), n, *range(n, 2*n)])
        self.plotter.add_mesh(pv.PolyData(points, np.asarray(faces)), name="opening_volume", color="deepskyblue", opacity=.12, show_edges=True, pickable=False)
        self.plotter.add_mesh(pv.lines_from_points(perimeter, close=True), name="opening_boundary", color="deepskyblue", line_width=4, render_lines_as_tubes=True, pickable=False)
        count = len(self.plotter.widgets.sphere_widgets)
        def drag(point, index, widget):
            try:
                candidate = resized_opening(self.editor.model, o, index, np.asarray(point)-shift, params)
                r = self.session.execute("set_opening", {"expected_revision": self.session.revision, "opening": candidate})
                self.report_opening(r)
            except Exception as exc:
                self.append("Opening", "Resize rejected: " + str(exc))
            self.opening_refresh.start(1)
        self.plotter.add_sphere_widget(drag, center=corners+shift, radius=radius, color="deepskyblue", selected_color="white",
                                      interaction_event="end", pass_widget=True, test_callback=False)
        self.opening_widgets = list(self.plotter.widgets.sphere_widgets[count:])
        self.plotter.render()

    def reset_view(self):
        self.plotter.camera_position = "iso"
        self.plotter.camera.up = (0, 0, 1)
        self.plotter.reset_camera()

    def surface_dialog(self, rebuild=False):
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("Rebuild control grid" if rebuild else "New flat wall")
        form = QtWidgets.QFormLayout(dialog)
        u, v = QtWidgets.QSpinBox(), QtWidgets.QSpinBox()
        for widget, count in ((u, self.editor.n), (v, self.editor.m)):
            widget.setRange(4, 20)
            widget.setValue(count)
        form.addRow("U control points (width)", u)
        form.addRow("V control points (height)", v)
        if rebuild:
            mode = QtWidgets.QComboBox()
            mode.addItem("Refine — add points, preserve shape", "refine")
            mode.addItem("Rebuild — fit a new uniform grid", "rebuild")
            tolerance = QtWidgets.QDoubleSpinBox()
            tolerance.setRange(0, 1000)
            tolerance.setDecimals(3)
            tolerance.setValue(5)
            tolerance.setSuffix(" mm")
            tolerance.setEnabled(False)
            mode.currentIndexChanged.connect(lambda: tolerance.setEnabled(mode.currentData() == "rebuild"))
            form.addRow("Method", mode)
            form.addRow("Allowed sampled deviation", tolerance)
        else:
            width, height = QtWidgets.QDoubleSpinBox(), QtWidgets.QDoubleSpinBox()
            for widget, value in ((width, 3000), (height, 1500)):
                widget.setRange(100, 100000)
                widget.setValue(value)
                widget.setSuffix(" mm")
            form.addRow("Width", width)
            form.addRow("Height", height)
        note = QtWidgets.QLabel("This change is undoable. Refinement only increases counts; rebuilding may slightly change the shape." if rebuild else "Creates a vertical flat wall in XZ. This replaces the current surface and can be undone.")
        note.setWordWrap(True)
        form.addRow(note)
        error = QtWidgets.QLabel()
        error.setWordWrap(True)
        form.addRow(error)
        buttons = QtWidgets.QDialogButtonBox(QtWidgets.QDialogButtonBox.StandardButton.Ok | QtWidgets.QDialogButtonBox.StandardButton.Cancel)
        form.addRow(buttons)
        buttons.rejected.connect(dialog.reject)
        def apply():
            args = {"expected_revision": self.session.revision, "count_u": u.value(), "count_v": v.value()}
            name = "rebuild_surface" if rebuild else "new_flat_surface"
            if rebuild:
                args.update(mode=mode.currentData(), tolerance_mm=tolerance.value())
            else:
                args.update(width_mm=width.value(), height_mm=height.value())
            try:
                report = self.session.execute(name, args)
            except Exception as exc:
                error.setText(str(exc))
                return
            self.append("Surface", f"Grid: {u.value()} × {v.value()}. " + (f"Maximum sampled deviation: {report['max_sampled_deviation_mm']:.4f} mm." if rebuild else "New flat wall created."))
            dialog.accept()
            if not rebuild:
                self.reset_view()
            self.plotter.interactor.setFocus()
        buttons.accepted.connect(apply)
        dialog.exec()

    def new_flat_dialog(self):
        self.surface_dialog(False)

    def rebuild_dialog(self):
        self.surface_dialog(True)

    def append(self, who, text):
        # Treat model/user output as plain text, never executable HTML.
        cursor = self.chat.textCursor()
        cursor.movePosition(QtGui.QTextCursor.MoveOperation.End)
        cursor.insertText(f"{who}\n{text}\n\n")
        self.chat.setTextCursor(cursor)
        self.chat.ensureCursorVisible()

    def _provider_changed(self, name):
        self.endpoint.setText(URLS[name])
        self.key.clear()
        self.key.setPlaceholderText(f"Or environment variable {ENV_KEYS[name]}")
        self.model.setText(os.getenv({"OpenAI": "OPENAI_MODEL", "Anthropic": "ANTHROPIC_MODEL", "Gemini": "GEMINI_MODEL", "OpenAI-compatible": "AI_MODEL"}[name], ""))

    def _solver_changed(self, name):
        self.editor.solver = name
        self.refresh_status()

    def refresh_status(self):
        s = self.session
        state = s.analysis_status()
        mode = (s.result or {}).get("design_context", {}).get("mode", "")
        text = f"Surface revision {s.revision} · Grid {self.editor.n} × {self.editor.m} · Analysis: {state}"
        if mode:
            text += f" ({mode})"
        if s.running:
            text += f"\nRunning {s.running['mode']} analysis of revision {s.running['revision']}…"
        if s.last_error:
            text += "\nLast run failed: " + s.last_error[:180]
        self.analysis_label.setText(text)
        self.fast.setEnabled(not s.running)
        self.full.setEnabled(not s.running)
        self.statusBar().showMessage(text.replace("\n", " | "))
        self.opening_refresh.start(180)

    def _set_busy(self, busy):
        self.agent_busy = busy
        self.send.setEnabled(not busy)
        self.clear.setEnabled(not busy)
        self.stop.setEnabled(busy)
        for w in (self.provider, self.model, self.endpoint, self.key):
            w.setEnabled(not busy)

    def send_message(self):
        text = self.input.toPlainText().strip()
        if not text or self.agent_busy:
            return
        try:
            provider = Provider(ProviderConfig(self.provider.currentText(), self.model.text(),
                                               self.key.text().strip(), self.endpoint.text().strip()))
        except Exception as exc:
            self.append("Connection", str(exc))
            return
        self.input.clear()
        self.append("You", text)
        self._set_busy(True)
        self.cancelled = threading.Event()
        cancelled = self.cancelled
        history = list(self.history)

        def execute(name, args):
            response = queue.Queue(maxsize=1)
            self.bridge.tool.emit((name, args, response, cancelled))
            while not cancelled.is_set():
                try:
                    value = response.get(timeout=.1)
                    if isinstance(value, Exception):
                        raise value
                    return value
                except queue.Empty:
                    pass
            raise RuntimeError("AI stopped")

        def worker():
            audit = []
            def emit(record):
                audit.append(record)
                self.bridge.audit.emit(record)
            try:
                answer, _ = run_agent(provider, history, text, execute, cancelled, emit)
            except Exception as exc:
                answer = f"Request stopped: {exc}. Any completed edits remain available to undo."
            if not self.closed:
                self.bridge.answer.emit((text, answer, audit))
        threading.Thread(target=worker, daemon=True).start()

    @QtCore.Slot(object)
    def _execute_tool(self, request):
        name, args, response, cancelled = request
        try:
            if cancelled.is_set() or self.closed:
                raise ValueError("AI stopped; tool not executed")
            result = self.session.execute(name, args)
        except Exception as exc:
            result = exc
        response.put(result)

    @QtCore.Slot(object)
    def _audit(self, record):
        r = record["result"]
        if "error" in r:
            self.append("Tool · " + record["tool"], r["error"])
        elif record["tool"] == "set_opening":
            self.report_opening(r)
        elif record["tool"] in ("edit_surface", "undo_edit", "redo_edit", "rebuild_surface", "new_flat_surface", "remove_opening"):
            self.append("Tool · " + record["tool"], f"Completed. Surface revision {r['revision']}.")
        else:
            self.statusBar().showMessage("AI used " + record["tool"])

    @QtCore.Slot(object)
    def _answer(self, payload):
        text, answer, audit = payload
        self.history.extend(portable_turn(text, answer, audit))
        self.history = self.history[-24:]
        self.append("Assistant", answer)
        self._set_busy(False)
        self.refresh_status()

    def stop_agent(self):
        self.cancelled.set()
        self.stop.setEnabled(False)
        self.append("System", "Stopping. No further AI tools will execute. An in-flight API request may take up to 90 seconds to return.")

    def clear_chat(self):
        self.history.clear()
        self.chat.clear()
        self.append("System", "Chat cleared. The surface and undo history are unchanged.")

    def start_analysis(self, mode):
        try:
            snapshot, params, meta = self.session.prepare_analysis(mode)
        except Exception as exc:
            self.append("Analysis", str(exc))
            return {"error": str(exc)}
        self.append("Analysis", f"Started {mode} check of revision {meta['revision']}. You can keep editing.")
        def worker():
            try:
                path, seconds = run_analysis(snapshot, params, meta, self.session.results_dir)
                payload = (path, seconds, None)
            except Exception as exc:
                payload = (None, 0, str(exc))
            if not self.closed:
                self.bridge.analysis_done.emit(payload)
        threading.Thread(target=worker, daemon=True).start()
        return {"status": "running", **meta, "note": "Completion will appear in chat. Ask a follow-up to discuss the result."}

    @QtCore.Slot(object)
    def _analysis_done(self, payload):
        path, seconds, error = payload
        self.session.finish_analysis(path, error)
        if error:
            self.append("Analysis", "Run failed: " + error)
            return
        s = self.session.summary()
        if s.get("fatal"):
            self.append("Generation failed", s["fatal"] + "\nAny previous wall remains visible; it is not the result of this run.")
            return
        geometry = "passed" if s["geometry_check"] else "failed"
        equilibrium = "not checked" if s["equilibrium_pass"] is None else "passed" if s["equilibrium_pass"] else f"not passed ({s['equilibrium_state']})"
        self.append("Analysis", f"Finished in {seconds:.1f}s · {s['freshness']}\n"
                    f"{s['masonry_piece_count']} masonry pieces, {s['lintel_count']} lintels. Geometry: {geometry}. Equilibrium: {equilibrium}.\n"
                    f"{s['failed_interface_count']} failing interfaces, {s['dropped_count']} dropped bricks.\n"
                    f"Optimization rounds: {s.get('iterations') or 0}. Selected solver: {s.get('solver', 'unknown')}.\n"
                    "Use Analysis details for construction-stage results, or ask the assistant to explain or compare changes.")
        if s.get("fatal"):
            self.append("Analysis issue", s["fatal"])
        if s.get("opening_assumptions"):
            self.append("Opening analysis assumptions", s["opening_assumptions"])

    def show_analysis(self):
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("Analysis evidence")
        dialog.resize(800, 650)
        layout = QtWidgets.QVBoxLayout(dialog)
        text = QtWidgets.QPlainTextEdit(json.dumps(self.session.summary(), indent=2))
        text.setReadOnly(True)
        layout.addWidget(text)
        dialog.exec()

    def save_as(self):
        initial = self.editor.model.path.with_name(self.editor.model.path.stem + "_edited.3dm")
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "Save surface", str(initial), "Rhino files (*.3dm)")
        if path:
            try:
                self.editor.model.save(path)
                self.append("Saved", path)
            except Exception as exc:
                self.append("Save failed", str(exc))

    def closeEvent(self, event):
        self.closed = True
        self.cancelled.set()
        self.plotter.close()
        event.accept()


def main(argv=None):
    parser = argparse.ArgumentParser(description="Chat-enabled NURBS surface editor")
    parser.add_argument("surface", nargs="?")
    parser.add_argument("-r", "--result")
    args = parser.parse_args(argv)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication(sys.argv[:1])
    app.setStyle("Fusion")
    window = MainWindow(Path(args.surface) if args.surface else None, args.result)
    window.show()
    app.exec()


if __name__ == "__main__":
    main()
