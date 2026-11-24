"""Qt desktop chat + PyVista viewport. Launch: python -m surface_editor --ai."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import queue
import sys
import threading

os.environ.setdefault("QT_API", "pyside6")
from PySide6 import QtCore, QtGui, QtWidgets
from pyvistaqt import QtInteractor

from .agent import portable_turn, run_agent
from .editor import SurfaceEditor, default_file, _project_root
from .providers import ENV_KEYS, URLS, Provider, ProviderConfig
from .session import DesignSession, run_analysis


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

    def _mortar_cb(self, value):
        super()._mortar_cb(value)
        if hasattr(self, "window"):
            self.window.refresh_status()  # a different bed joint makes the shown analysis outdated

    def start_refit(self):
        if hasattr(self, "window"):
            self.window.start_analysis("full")

    def save(self):
        if hasattr(self, "window"):
            self.window.save_as()


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, path, result_path=None):
        super().__init__()
        self.setWindowTitle("Surface Studio — AI design assistant")
        self.resize(1560, 980)
        self.history = []
        self.agent_busy = False
        self.cancelled = threading.Event()
        self.closed = False
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
        self.full = QtWidgets.QPushButton("Full analysis")
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
        self.plotter.set_background("#f4f7fa")
        # The embedded window supplies its own save/analyze controls.
        self.plotter.add_text("Drag orange handles · x/y/z: axis lock · g: free\nu: undo · b: reset · v: wall · a: analyze",
                              name="help", position="upper_left", font_size=10, color="#243b53")
        toolbar = self.addToolBar("Design")
        toolbar.setMovable(False)
        for label, callback in (("Undo", self.editor.undo), ("Reset surface", self.editor.reset),
                                ("Add control points…", self.add_control_points),
                                ("Toggle wall", self.editor.toggle_wall), ("Fit view", self.plotter.reset_camera),
                                ("Top view", self.plotter.view_xy),
                                ("View from below", lambda: self.plotter.view_xy(negative=True)),
                                ("Toggle handles", self.editor.toggle_handles),
                                ("Save surface as…", self.save_as), ("Analysis details", self.show_analysis)):
            action = toolbar.addAction(label)
            action.triggered.connect(lambda checked=False, fn=callback: fn())
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
        text = f"Surface revision {s.revision} · Analysis: {state}"
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
        elif record["tool"] in ("edit_surface", "undo_edit"):
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
            self.append("Analysis", f"Finished in {seconds:.1f}s · {s['freshness']}\n"
                        f"No wall was fitted. Solver error: {s['fatal']}")
            return
        geometry = "passed" if s["geometry_check"] else "failed"
        equilibrium = s["equilibrium_state"] if s["equilibrium_pass"] is not None else "not checked"
        self.append("Analysis", f"Finished in {seconds:.1f}s · {s['freshness']}\n"
                    f"{s['brick_count']} bricks. Geometry: {geometry}. Equilibrium: {equilibrium}.\n"
                    f"{s['failed_interface_count']} failing interfaces, {s['dropped_count']} dropped bricks.\n"
                    "Use Analysis details for construction-stage results, or ask the assistant to explain or compare changes.")

    def show_analysis(self):
        dialog = QtWidgets.QDialog(self)
        dialog.setWindowTitle("Analysis evidence")
        dialog.resize(800, 650)
        layout = QtWidgets.QVBoxLayout(dialog)
        text = QtWidgets.QPlainTextEdit(json.dumps(self.session.summary(), indent=2))
        text.setReadOnly(True)
        layout.addWidget(text)
        dialog.exec()

    def add_control_points(self):
        choices = ["Along the wall", "Up the height"]
        choice, ok = QtWidgets.QInputDialog.getItem(self, "Add control points", "Direction", choices, 0, False)
        if not ok:
            return
        per_span, ok = QtWidgets.QInputDialog.getInt(
            self, "Add control points", "New control points per existing span (shape is unchanged)", 1, 1, 4)
        if not ok:
            return
        try:
            self.editor.add_control_points(choices.index(choice), per_span)
        except Exception as exc:
            self.append("Control points", str(exc))
            return
        self.append("Control points", f"Control grid is now {self.editor.n} × {self.editor.m}. Surface shape unchanged.")

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
    window = MainWindow(Path(args.surface) if args.surface else default_file(), args.result)
    window.show()
    app.exec()


if __name__ == "__main__":
    main()
