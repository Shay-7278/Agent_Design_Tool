"""Small native API adapters. Geometry and tool execution never live here.

Native response blocks are retained during a tool loop (including reasoning
items and Gemini thought signatures). Completed turns use a portable text
transcript plus tool audit so switching providers does not reuse foreign IDs.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import json
import os
import urllib.error
import urllib.parse
import urllib.request


@dataclass
class ProviderConfig:
    provider: str
    model: str
    api_key: str = field(default="", repr=False)
    base_url: str = ""


@dataclass
class ToolCall:
    id: str
    name: str
    arguments: dict


@dataclass
class Reply:
    text: str
    calls: list[ToolCall]
    raw: object


ENV_KEYS = {"OpenAI": "OPENAI_API_KEY", "Anthropic": "ANTHROPIC_API_KEY",
            "Gemini": "GEMINI_API_KEY", "OpenAI-compatible": "AI_API_KEY"}
URLS = {"OpenAI": "https://api.openai.com/v1", "Anthropic": "https://api.anthropic.com/v1",
        "Gemini": "https://generativelanguage.googleapis.com/v1beta",
        "OpenAI-compatible": "http://localhost:11434/v1"}


class NoRedirect(urllib.request.HTTPRedirectHandler):
    def redirect_request(self, req, fp, code, msg, headers, newurl):
        raise ValueError("API redirects are disabled; configure the final API base URL")


def post_json(url, headers, body):
    req = urllib.request.Request(url, data=json.dumps(body, allow_nan=False).encode(),
                                 headers={"Content-Type": "application/json", **headers})
    try:
        with urllib.request.build_opener(NoRedirect).open(req, timeout=300) as response:
            return json.load(response)
    except urllib.error.HTTPError as exc:
        # Do not echo server bodies or request headers: they can contain secrets.
        raise RuntimeError(f"AI API returned HTTP {exc.code}. Check model, key, quota and endpoint.") from None
    except urllib.error.URLError:
        raise RuntimeError("Cannot reach AI API. Check connection and endpoint.") from None


class Provider:
    def __init__(self, config, transport=post_json):
        if config.provider not in URLS:
            raise ValueError("unknown provider")
        if not config.model.strip():
            raise ValueError("Enter a model ID available in your API account")
        self.config = config
        self.key = config.api_key or os.getenv(ENV_KEYS[config.provider], "")
        if not self.key and config.provider != "OpenAI-compatible":
            raise ValueError(f"Enter an API key or set {ENV_KEYS[config.provider]}")
        self.base = (config.base_url or URLS[config.provider]).rstrip("/")
        parsed = urllib.parse.urlparse(self.base)
        if parsed.username or parsed.password or parsed.query or parsed.fragment:
            raise ValueError("Base URL must not contain credentials, query or fragment")
        if parsed.scheme != "https" and not (parsed.scheme == "http" and parsed.hostname in ("localhost", "127.0.0.1", "::1")):
            raise ValueError("Use HTTPS, or HTTP for a local provider")
        self.transport = transport
        self.history = []

    def start(self, history):
        p = self.config.provider
        if p == "Gemini":
            self.history = [{"role": "model" if x["role"] == "assistant" else "user",
                             "parts": [{"text": x["content"]}]} for x in history]
        else:
            self.history = [dict(x) for x in history]

    def request(self, system, tools):
        p, model = self.config.provider, self.config.model.strip()
        if p == "OpenAI":
            defs = [{"type": "function", **t, "strict": False} for t in tools]
            data = self.transport(self.base + "/responses", {"Authorization": f"Bearer {self.key}"},
                                  {"model": model, "instructions": system, "input": self.history,
                                   "tools": defs, "store": False,
                                   "include": ["reasoning.encrypted_content"]})
            if data.get("status") in ("failed", "incomplete"):
                raise RuntimeError("AI response failed or was incomplete; no tools were executed")
            raw = data.get("output", [])
            calls = [ToolCall(x["call_id"], x["name"], json.loads(x["arguments"]))
                     for x in raw if x.get("type") == "function_call"]
            text = "\n".join(c.get("text", c.get("refusal", "")) for x in raw
                             if x.get("type") == "message" for c in x.get("content", []))
        elif p == "Anthropic":
            defs = [{"name": t["name"], "description": t["description"], "input_schema": t["parameters"]} for t in tools]
            data = self.transport(self.base + "/messages", {"x-api-key": self.key, "anthropic-version": "2023-06-01"},
                                  {"model": model, "max_tokens": 4096, "system": system,
                                   "messages": self.history, "tools": defs})
            if data.get("stop_reason") == "max_tokens":
                raise RuntimeError("AI response was truncated; no tools were executed")
            raw = data.get("content", [])
            calls = [ToolCall(x["id"], x["name"], x["input"]) for x in raw if x.get("type") == "tool_use"]
            text = "\n".join(x["text"] for x in raw if x.get("type") == "text")
        elif p == "Gemini":
            defs = [{"name": t["name"], "description": t["description"], "parametersJsonSchema": t["parameters"]} for t in tools]
            data = self.transport(self.base + "/models/" + urllib.parse.quote(model, safe="") + ":generateContent",
                                  {"x-goog-api-key": self.key},
                                  {"systemInstruction": {"parts": [{"text": system}]}, "contents": self.history,
                                   "tools": [{"functionDeclarations": defs}]})
            candidates = data.get("candidates", [])
            if not candidates or candidates[0].get("finishReason") not in (None, "STOP"):
                raise RuntimeError("Gemini returned no complete response; no tools were executed")
            raw = candidates[0]["content"]
            calls = [ToolCall(x["functionCall"].get("id", str(i)), x["functionCall"]["name"],
                              x["functionCall"].get("args", {}))
                     for i, x in enumerate(raw.get("parts", [])) if "functionCall" in x]
            text = "\n".join(x["text"] for x in raw.get("parts", []) if "text" in x and not x.get("thought"))
        else:
            data = self.transport(self.base + "/chat/completions", {"Authorization": f"Bearer {self.key}"} if self.key else {},
                                  {"model": model, "messages": [{"role": "system", "content": system}] + self.history,
                                   "tools": [{"type": "function", "function": t} for t in tools]})
            choice = data["choices"][0]
            if choice.get("finish_reason") in ("length", "content_filter"):
                raise RuntimeError("AI response was incomplete; no tools were executed")
            raw = choice["message"]
            calls = [ToolCall(x["id"], x["function"]["name"], json.loads(x["function"]["arguments"]))
                     for x in raw.get("tool_calls", [])]
            text = raw.get("content") or ""
        if not calls and not text:
            raise RuntimeError("AI returned neither text nor a tool call")
        return Reply(text, calls, raw)

    def accept(self, reply, results):
        p = self.config.provider
        if p == "OpenAI":
            self.history.extend(reply.raw)
            self.history.extend({"type": "function_call_output", "call_id": c.id, "output": json.dumps(r)}
                                for c, r in zip(reply.calls, results))
        elif p == "Anthropic":
            self.history.append({"role": "assistant", "content": reply.raw})
            if results:
                self.history.append({"role": "user", "content": [
                    {"type": "tool_result", "tool_use_id": c.id, "content": json.dumps(r), "is_error": "error" in r}
                    for c, r in zip(reply.calls, results)]})
        elif p == "Gemini":
            self.history.append(reply.raw)
            if results:
                parts = []
                native_calls = [x["functionCall"] for x in reply.raw.get("parts", []) if "functionCall" in x]
                for c, r, native in zip(reply.calls, results, native_calls):
                    response = {"name": c.name, "response": r}
                    if "id" in native:
                        response["id"] = native["id"]
                    parts.append({"functionResponse": response})
                self.history.append({"role": "user", "parts": parts})
        else:
            self.history.append(reply.raw)
            self.history.extend({"role": "tool", "tool_call_id": c.id, "content": json.dumps(r)}
                                for c, r in zip(reply.calls, results))
