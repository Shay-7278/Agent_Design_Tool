"""Offline tests: python -m unittest surface_editor.test_ai -v (from gui/)."""
import copy
import json
from pathlib import Path
import tempfile
import threading
import unittest
from unittest.mock import patch

import numpy as np

from .operations import apply_operations, local_frame
from .providers import Provider, ProviderConfig, Reply, ToolCall
from .agent import run_agent, TOOLS
from .session import DesignSession, clone_model, surface_hash, run_analysis
from .model import NurbsModel
from . import refit
from . import wall_view

ROOT = Path(__file__).resolve().parents[2]


def plane(n=6):
    x, z = np.meshgrid(np.linspace(0, 1000, n), np.linspace(0, 1000, 6), indexing="ij")
    return np.stack([x, np.zeros_like(x), z], axis=-1)


class OperationsTests(unittest.TestCase):
    def setUp(self):
        self.cps = plane()
        self.origin, self.frame = local_frame(self.cps)

    def apply(self, *operations):
        return apply_operations(self.cps, list(operations), self.origin, self.frame)

    def test_translation_and_scale(self):
        q = self.apply({"kind": "translate", "vector": [10, 20, 30]})
        np.testing.assert_allclose(q, self.cps + [10, 20, 30])
        q = self.apply({"kind": "scale", "factors": [2, 1, .5]})
        np.testing.assert_allclose(np.ptp(q, axis=(0, 1)), [2000, 0, 500])

    def test_local_frame_rotated_wall(self):
        angle = .7
        rotation = np.array([[np.cos(angle), -np.sin(angle), 0], [np.sin(angle), np.cos(angle), 0], [0, 0, 1]])
        p = self.cps @ rotation.T
        origin, frame = local_frame(p)
        q = apply_operations(p, [{"kind": "translate", "vector": [0, 100, 0]}], origin, frame)
        np.testing.assert_allclose(q-p, np.broadcast_to(frame[:, 1]*100, q.shape), atol=1e-10)

    def test_pull_boundary_and_strength(self):
        q = self.apply({"kind": "pull", "center_uv": [.5, .5], "radius_uv": .2,
                        "vector": [0, 100, 0], "keep_boundary": True})
        np.testing.assert_array_equal(q[0], self.cps[0])
        np.testing.assert_array_equal(q[:, -1], self.cps[:, -1])
        self.assertGreater(q[2, 2, 1], q[1, 1, 1])

    def test_twist_and_taper(self):
        q = self.apply({"kind": "twist", "axis": "height", "angle_degrees": 90})
        np.testing.assert_allclose(q[:, 0], self.cps[:, 0], atol=1e-10)
        self.assertAlmostEqual(np.ptp(q[:, -1, 0]), 0)
        self.assertAlmostEqual(np.ptp(q[:, -1, 1]), 1000)
        q = self.apply({"kind": "taper", "axis": "height", "factor": .5})
        self.assertAlmostEqual(np.ptp(q[:, -1, 0]), 500)

    def test_bend_flatten_smooth(self):
        q = self.apply({"kind": "bend", "axis": "width", "amount_mm": 200})
        self.assertGreater(q[2, 2, 1], 100)
        np.testing.assert_allclose(q[[0, -1], :, 1], 0)
        q = apply_operations(q, [{"kind": "flatten", "axis": "depth"}], self.origin, self.frame)
        self.assertLess(np.ptp(q[..., 1]), 1e-8)
        q = self.apply({"kind": "set_points", "points": [{"i": 2, "j": 2, "position": [400, 300, 400]}]},
                       {"kind": "smooth", "strength": .5, "iterations": 1})
        self.assertAlmostEqual(q[2, 2, 1], 150)

    def test_wave_limit_follows_its_own_direction(self):
        wave = {"kind": "wave", "axis": "width", "amount_mm": 100, "cycles": 3}
        with self.assertRaisesRegex(ValueError, "too detailed"):
            self.apply(wave)
        fine = plane(15)
        origin, frame = local_frame(fine)
        q = apply_operations(fine, [wave], origin, frame)
        self.assertGreater(np.abs(q[..., 1]).max(), 50)

    def test_wave_fade_height_affects_base_only(self):
        q = self.apply({"kind": "wave", "axis": "width", "amount_mm": 100, "cycles": 1, "fade_height": .3})
        self.assertGreater(np.abs(q[:, 0, 1]).max(), 50)
        np.testing.assert_allclose(q[:, 2:], self.cps[:, 2:], atol=1e-9)
        with self.assertRaises(ValueError):
            self.apply({"kind": "wave", "axis": "height", "amount_mm": 100, "cycles": 1, "fade_height": .3})
        with self.assertRaisesRegex(ValueError, "keep_boundary"):
            self.apply({"kind": "wave", "axis": "width", "amount_mm": 100, "cycles": 1, "fade_height": .3,
                        "keep_boundary": True})

    def test_batch_failure_leaves_original_untouched(self):
        original = self.cps.copy()
        with self.assertRaises(ValueError):
            self.apply({"kind": "translate", "vector": [1, 2, 3]}, {"kind": "scale", "factors": [0, 1, 1]})
        np.testing.assert_array_equal(self.cps, original)

    def test_reject_invalid_arguments(self):
        bad = [{"kind": "exec", "code": "print(1)"},
               {"kind": "translate", "vector": [0, float("nan"), 0]},
               {"kind": "translate", "vector": [0, True, 0]},
               {"kind": "pull", "center_uv": [2, .5], "radius_uv": .1, "vector": [0, 1, 0]},
               {"kind": "wave", "axis": "width", "amount_mm": 100, "cycles": 8},
               {"kind": "set_points", "points": [{"i": -1, "j": 0, "position": [0, 0, 0]}]},
               {"kind": "flatten", "axis": "width"},
               {"kind": "translate", "vector": [0, 1, 0], "unexpected": 1}]
        for op in bad:
            with self.subTest(op=op), self.assertRaises(ValueError):
                self.apply(op)


class FakeEditor:
    def __init__(self):
        self.model = NurbsModel.load(ROOT / "Reference Surface_1.3dm")
        self.cps = self.model.control_points()
        self.undo_stack = []
        self._wall_result_path = None
        self.solver = "rbe"

    def apply_control_points(self, p):
        self.undo_stack.append(self.cps.copy())
        self.cps = p.copy()
        self.model.set_control_points(self.cps)
        self.on_surface_changed()

    def add_control_points(self, direction, per_span):
        self.undo_stack.append(self.model.snapshot())
        self.model.insert_knots(direction, per_span)
        self.cps = self.model.control_points()
        self.on_surface_changed()

    def undo(self):
        entry = self.undo_stack.pop()
        if isinstance(entry, np.ndarray):
            self.cps = entry
            self.model.set_control_points(self.cps)
        else:
            self.model.restore(entry)
            self.cps = self.model.control_points()
        self.on_surface_changed()

    def show_wall(self, path):
        self._wall_result_path = path


class SessionTests(unittest.TestCase):
    def setUp(self):
        self.editor = FakeEditor()
        self.session = DesignSession(self.editor, ROOT / "results")

    def edit(self, revision=0):
        return self.session.execute("edit_surface", {"expected_revision": revision,
            "operations": [{"kind": "translate", "vector": [0, 25, 0]}]})

    def test_edit_revision_undo(self):
        before = self.editor.cps.copy()
        self.edit()
        self.assertEqual(self.session.revision, 1)
        self.session.execute("undo_edit", {"expected_revision": 1})
        np.testing.assert_allclose(self.editor.cps, before)
        self.assertEqual(self.session.revision, 2)

    def test_base_must_stay_level(self):
        self.assertEqual(self.session.state()["base_edge"], "j=0")
        before = self.editor.cps.copy()
        with self.assertRaisesRegex(ValueError, "base .* must stay level"):
            self.session.execute("edit_surface", {"expected_revision": 0,
                "operations": [{"kind": "rotate", "axis": "depth", "angle_degrees": 5}]})
        np.testing.assert_array_equal(self.editor.cps, before)
        self.assertEqual(self.session.revision, 0)
        self.session.execute("edit_surface", {"expected_revision": 0,
            "operations": [{"kind": "translate", "vector": [0, 0, 100]}]})
        self.assertEqual(self.session.revision, 1)

    def test_insert_knots_keeps_shape(self):
        model = self.editor.model
        before = model.evaluate(80, 20)
        model.insert_knots(0, 1)
        self.assertEqual(model.shape, (9, 6))
        model.insert_knots(1, 2)
        self.assertEqual(model.shape, (9, 12))
        self.assertLess(np.abs(model.evaluate(80, 20) - before).max(), 1e-6)
        self.assertLess(np.ptp(model.control_points()[:, 0, 2]), 1e-9)
        for direction, per_span in ((2, 1), (0, 0), (0, 5), (0, 1.0)):
            with self.subTest(direction=direction, per_span=per_span), self.assertRaises(ValueError):
                model.insert_knots(direction, per_span)

    def test_add_control_points_tool_and_undo(self):
        before = self.editor.cps.copy()
        out = self.session.execute("add_control_points", {"expected_revision": 0, "direction": "width", "per_span": 3})
        self.assertEqual(out["grid_shape"], [15, 6])
        self.assertEqual(self.session.revision, 1)
        self.assertEqual(self.session.state()["grid_shape"], [15, 6])
        self.session.execute("edit_surface", {"expected_revision": 1, "operations": [
            {"kind": "wave", "axis": "width", "amount_mm": 100, "cycles": 3, "fade_height": .3}]})
        self.session.execute("undo_edit", {"expected_revision": 2})
        self.session.execute("undo_edit", {"expected_revision": 3})
        np.testing.assert_array_equal(self.editor.cps, before)
        bad = [{"expected_revision": 0, "direction": "width", "per_span": 1},
               {"expected_revision": 4, "direction": "depth", "per_span": 1},
               {"expected_revision": 4, "direction": "width", "per_span": 9}]
        for args in bad:
            with self.subTest(args=args), self.assertRaises(ValueError):
                self.session.execute("add_control_points", args)
        self.assertEqual(self.editor.cps.shape, (6, 6, 3))

    def test_manual_change_rejects_stale_ai(self):
        self.editor.apply_control_points(self.editor.cps + [0, 1, 0])
        with self.assertRaisesRegex(ValueError, "Surface changed"):
            self.edit(0)
        self.assertEqual(len(self.editor.undo_stack), 1)

    def test_snapshot_and_analysis_freshness(self):
        snapshot, params, meta = self.session.prepare_analysis("fast")
        fingerprint = surface_hash(snapshot)
        self.edit()
        self.assertEqual(fingerprint, surface_hash(snapshot))
        self.session.result = {"design_context": meta, "use_cra": False, "l1_pass": False}
        self.assertEqual(self.session.analysis_status(), "outdated")
        self.assertIsNone(self.session.summary()["equilibrium_pass"])
        self.editor.undo()
        self.assertEqual(self.session.analysis_status(), "current")
        self.editor.solver = "cra_penalty"
        self.assertEqual(self.session.analysis_status(), "outdated")

    def test_old_results_unknown(self):
        self.session.result = {"ok": True}
        self.assertEqual(self.session.analysis_status(), "unknown surface version")

    def test_fast_pass_not_labelled_structural_fail(self):
        label = wall_view.result_summary({"use_cra": False, "ok": False, "l0_pass": True, "bricks": []}, "result.json")
        self.assertIn("GEOMETRY PASS", label)
        self.assertIn("equilibrium not checked", label)
        self.assertNotIn("FAIL", label)

    def test_bad_tool_or_revision_types(self):
        for name, args in [("save", {}), ("get_surface_state", {"secret": 1}), ("undo_edit", {"expected_revision": False})]:
            with self.assertRaises(ValueError):
                self.session.execute(name, args)

    def test_analysis_running_and_metadata(self):
        snapshot, params, meta = self.session.prepare_analysis("fast")
        with self.assertRaises(ValueError):
            self.session.prepare_analysis("full")
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / "result.json"
            path.write_text(json.dumps({"ok": False, "bricks": [], "l0_pass": True, "use_cra": False}))
            with patch.object(refit, "build_job", return_value={}) as build, patch.object(refit, "run_solver", return_value=(path, 1.5)):
                result_path, seconds = run_analysis(snapshot, params, meta, d)
                build.assert_called_once_with(snapshot, params, use_cra=False)
                self.assertEqual(json.loads(result_path.read_text())["design_context"], meta)
                self.session.finish_analysis(result_path)
                self.assertEqual(self.session.analysis_status(), "current")
                self.assertIsNone(self.session.running)


class ProviderTests(unittest.TestCase):
    def test_native_tool_roundtrips(self):
        fixtures = {
            "OpenAI": {"output": [{"type": "reasoning", "id": "r", "encrypted_content": "opaque"}, {"type": "function_call", "call_id": "c", "name": "get_surface_state", "arguments": "{}"}]},
            "Anthropic": {"content": [{"type": "tool_use", "id": "c", "name": "get_surface_state", "input": {}}]},
            "Gemini": {"candidates": [{"content": {"role": "model", "parts": [{"thoughtSignature": "opaque", "functionCall": {"id": "c", "name": "get_surface_state", "args": {}}}]}}]},
            "OpenAI-compatible": {"choices": [{"message": {"role": "assistant", "tool_calls": [{"id": "c", "type": "function", "function": {"name": "get_surface_state", "arguments": "{}"}}]}}]},
        }
        for name, response in fixtures.items():
            with self.subTest(provider=name):
                requests = []
                def transport(url, headers, body):
                    requests.append(copy.deepcopy(body))
                    return copy.deepcopy(response)
                p = Provider(ProviderConfig(name, "test-model", "test-key"), transport)
                p.start([{"role": "user", "content": "inspect"}])
                reply = p.request("system", TOOLS)
                self.assertEqual(reply.calls[0].name, "get_surface_state")
                p.accept(reply, [{"revision": 3}])
                p.request("system", TOOLS)
                encoded = json.dumps(requests[-1])
                self.assertIn('"revision"', encoded.replace('\\"', '"'))
                if name in ("OpenAI", "Gemini"):
                    self.assertIn("opaque", encoded)

    def test_endpoint_and_model_validation(self):
        for model, url in [("", "https://example.com"), ("m", "http://example.com"), ("m", "https://user:pass@example.com")]:
            with self.assertRaises(ValueError):
                Provider(ProviderConfig("OpenAI", model, "key", url))

    def test_repr_does_not_expose_key(self):
        self.assertNotIn("super-secret", repr(ProviderConfig("OpenAI", "m", "super-secret")))

    def test_incomplete_response_never_dispatches(self):
        p = Provider(ProviderConfig("OpenAI", "m", "k"), lambda *args: {"status": "incomplete", "output": []})
        with self.assertRaises(RuntimeError):
            p.request("s", TOOLS)


class AgentTests(unittest.TestCase):
    def test_loop_executes_tools_then_answers(self):
        class Fake:
            def start(self, history): self.history = history
            def request(self, system, tools):
                if not hasattr(self, "done"):
                    return Reply("", [ToolCall("1", "get_surface_state", {})], [])
                return Reply("Inspected revision 4", [], [])
            def accept(self, reply, results): self.done = results
        provider = Fake()
        events = []
        answer, audit = run_agent(provider, [], "inspect", lambda n, a: {"revision": 4}, threading.Event(), events.append)
        self.assertEqual(answer, "Inspected revision 4")
        self.assertEqual(provider.done, [{"revision": 4}])
        self.assertEqual(audit, events)

    def test_cancel_before_request(self):
        class Fake:
            def start(self, history): pass
            def request(self, *args): raise AssertionError("must not request")
        event = threading.Event()
        event.set()
        answer, audit = run_agent(Fake(), [], "edit", None, event, None)
        self.assertIn("Stopped", answer)
        self.assertEqual(audit, [])


class SolverIOTests(unittest.TestCase):
    def test_stale_result_not_reused(self):
        with tempfile.TemporaryDirectory() as d:
            (Path(d) / "gui_refit_result.json").write_text('{"ok": true, "bricks": []}')
            result = type("Process", (), {"returncode": 1, "stdout": "", "stderr": ""})()
            with patch.object(refit.subprocess, "run", return_value=result), self.assertRaises(RuntimeError):
                refit.run_solver({}, d)

    def test_failed_check_is_valid_result_and_names_unique(self):
        paths = []
        def solver(cmd, **kwargs):
            p = Path(cmd[4])
            paths.append(p)
            p.write_text('{"ok": false, "bricks": []}')
            return type("Process", (), {"returncode": 1, "stdout": "", "stderr": ""})()
        with tempfile.TemporaryDirectory() as d, patch.object(refit.subprocess, "run", side_effect=solver):
            refit.run_solver({}, d)
            refit.run_solver({}, d)
        self.assertNotEqual(paths[0], paths[1])


if __name__ == "__main__":
    unittest.main()
