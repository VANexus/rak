"""Agentic tool loop for the harness (P1).

Owns the Claude Agent SDK: one SDK in-process MCP tool per registry entry,
the transcript -> function-calling -> /tool_proposals -> /tool_result ->
speech loop, and thinking traces forwarded to the parent over stdio.

Persona (persona.py/persona.toml), prompt engineering (prompts.py) and
long-context (context.py SessionDigest) all compose here; see
docs/brain_arm_tools_design.md.

Isolation contract (D7 in docs/brain_arm_tools_design.md):
  * Runs as a SUBPROCESS of harness_node: the SDK bundles a pydantic-v2
    dependency tree that must never leak into the ROS system Python. The
    overlay path is resolved by rak_harness.sdk and injected via PYTHONPATH
    by the parent; this module never hardcodes a home directory.
  * Tool EXECUTION authority lives in rak_tool_executor (deterministic
    preconditions + audit). This process only proposes and narrates.
  * Motion-class tools are only exposed when the parent says so; the
    executor still requires its own allow_motion gate per call.

stdio protocol (one JSON object per line):
  parent -> child:  {"type":"transcript","text":...}
                    {"type":"context","vehicle":...,"decision":...,"wake":...}
                    {"type":"tool_response","call_id":...,"status":...,
                     "reason":...,"detail":{...}}
                    {"type":"shutdown"}
  child -> parent:  {"type":"thinking","text":...}
                    {"type":"assistant","text":...}
                    {"type":"tool_call","name":...,"args":{...}}
                    {"type":"tool_result","name":...,"ok":bool,"summary":...}
                    {"type":"final","text":...}
                    {"type":"status","state":"up|degraded|down","detail":...}
"""

from __future__ import annotations

import asyncio
import json
import os
import sys
import uuid
from pathlib import Path
from typing import Any



def _load_registry():
    """Import the single-source registry (rak_tool_executor package)."""
    try:
        from rak_tool_executor import tool_registry

        return tool_registry
    except Exception:  # pragma: no cover - resolved by build order
        return None


def _find_persona_file() -> str:
    for base in (Path.cwd(), Path(__file__).resolve().parent):
        for start in (base, *base.parents):
            candidate = start / "config" / "persona.toml"
            if candidate.is_file():
                return str(candidate)
            candidate = start / "share" / "rak_harness" / "config" / "persona.toml"
            if candidate.is_file():
                return str(candidate)
    return ""


def _out(obj: dict[str, Any]) -> None:
    sys.stdout.write(json.dumps(obj, ensure_ascii=False) + "\n")
    sys.stdout.flush()


class _Bridge:
    """Non-blocking stdio bridge for tool proposals -> parent -> results."""

    def __init__(self) -> None:
        self._pending: dict[str, asyncio.Future] = {}
        self._reader = asyncio.Queue()

    async def read_lines(self) -> None:
        loop = asyncio.get_running_loop()
        while True:
            line = await loop.run_in_executor(None, sys.stdin.readline)
            if not line:
                _out({"type": "status", "state": "down", "detail": "stdin closed"})
                return
            line = line.strip()
            if not line:
                continue
            try:
                msg = json.loads(line)
            except ValueError:
                continue
            if msg.get("type") == "tool_response":
                fut = self._pending.pop(msg.get("call_id", ""), None)
                if fut is not None and not fut.done():
                    fut.set_result(msg)
            else:
                await self._reader.put(msg)

    def requests(self) -> asyncio.Queue:
        return self._reader

    async def propose(self, tool: str, args: dict[str, Any], timeout: float = 60.0) -> dict[str, Any]:
        call_id = uuid.uuid4().hex
        fut: asyncio.Future = asyncio.get_running_loop().create_future()
        self._pending[call_id] = fut
        _out({"type": "tool_call", "name": tool, "args": args})
        _out({"type": "propose", "call_id": call_id, "tool": tool,
              "args_json": json.dumps(args, ensure_ascii=False)})
        try:
            return await asyncio.wait_for(fut, timeout=timeout)
        except asyncio.TimeoutError:
            self._pending.pop(call_id, None)
            return {"status": "timeout", "reason": "no ToolResult in time", "detail": {}}


class AgentLoop:
    def __init__(self, *, allow_motion_tools: bool, model: str,
                 digest, persona_file: str = "") -> None:
        self._allow_motion_tools = allow_motion_tools
        self._model = model
        self._registry = _load_registry()
        self._bridge = _Bridge()

        from .context import SessionDigest
        from .persona import load_persona
        from .prompts import build_system_prompt, PROMPT_VERSION

        self._persona = load_persona(persona_file or _find_persona_file())
        self._digest: SessionDigest = digest
        names = [t.name for t in self._registry.TOOLS] if self._registry else []
        self._system_prompt = build_system_prompt(self._persona, names)
        self._prompt_version = PROMPT_VERSION

    # ---------------------------------------------------------------- sdk setup

    def _tool_specs(self):
        specs = []
        for spec in self._registry.TOOLS:
            if spec.status != "implemented":
                continue  # unimplemented tools stay invisible to the LLM
            if spec.safety.startswith("motion") and not self._allow_motion_tools:
                continue
            specs.append(spec)
        return specs

    async def run(self) -> None:
        try:
            from claude_agent_sdk import (  # type: ignore
                ClaudeAgentOptions,
                create_sdk_mcp_server,
                query,
                tool as sdk_tool,
            )
        except Exception as exc:
            _out({"type": "status", "state": "down",
                  "detail": f"claude-agent-sdk import failed: {exc}"})
            return

        handlers = {}
        for spec in self._tool_specs():
            handlers[spec.name] = self._make_handler(sdk_tool, spec)

        server = create_sdk_mcp_server(name="robot", version="1.0.0",
                                       tools=list(handlers.values()))
        allowed = [f"mcp__robot__{name}" for name in handlers]
        options_kwargs = dict(
            mcp_servers={"robot": server},
            allowed_tools=allowed,
            system_prompt=self._system_prompt,
            model=self._model,
            permission_mode="bypassPermissions",  # headless bench: authority stays in the executor
            max_turns=12,
            setting_sources=[],  # do not inherit the operator's ~/.claude plugins/hooks
        )
        try:
            options = ClaudeAgentOptions(**options_kwargs)
        except TypeError:  # older SDK without the kwarg
            del options_kwargs["setting_sources"]
            options = ClaudeAgentOptions(**options_kwargs)

        asyncio.get_running_loop().create_task(self._bridge.read_lines())
        _out({"type": "status", "state": "up",
              "detail": f"tools={sorted(handlers)} model={self._model} "
                        f"persona={self._persona.name} prompt={self._prompt_version}"})

        while True:
            request = await self._bridge.requests().get()
            kind = request.get("type")
            if kind == "shutdown":
                return
            if kind == "context":
                self._digest.update_context(request)
                continue
            text = str(request.get("text", "")).strip()
            if not text:
                continue
            await self._turn(text, query, options)

    def _make_handler(self, sdk_tool, spec):
        bridge = self._bridge
        digest = self._digest

        @sdk_tool(spec.name, spec.description, spec.args_schema)
        async def _handler(args: dict[str, Any]) -> dict[str, Any]:
            result = await bridge.propose(spec.name, args or {})
            ok = result.get("status") == "ok"
            digest.add_tool(spec.name, ok, str(result.get("reason", ""))[:80])
            _out({"type": "tool_result", "name": spec.name, "ok": ok,
                  "summary": result.get("reason", "")})
            body = {
                "status": result.get("status"),
                "reason": result.get("reason", ""),
                "detail": result.get("detail", {}),
            }
            return {"content": [{"type": "text",
                                 "text": json.dumps(body, ensure_ascii=False)}]}

        _handler.__name__ = spec.name
        return _handler

    async def _turn(self, text: str, query, options) -> None:
        message = self._digest.render(text)
        final_text = ""
        try:
            async for message in query(prompt=message, options=options):
                mtype = getattr(message, "type", "")
                if mtype == "assistant":
                    for block in getattr(message, "content", []) or []:
                        btype = getattr(block, "type", "")
                        if btype == "thinking" and getattr(block, "thinking", ""):
                            _out({"type": "thinking", "text": block.thinking})
                        elif btype == "text" and getattr(block, "text", ""):
                            _out({"type": "assistant", "text": block.text})
                elif mtype == "result":
                    final_text = str(getattr(message, "result", "") or "")
        except Exception as exc:
            final_text = f"循环出错：{exc}"
        if final_text:
            _out({"type": "final", "text": final_text})
            self._digest.add_assistant(final_text)


def main() -> None:
    allow_motion = "--allow-motion-tools" in sys.argv
    model = os.environ.get("ANTHROPIC_MODEL", "deepseek-chat")
    for i, arg in enumerate(sys.argv):
        if arg == "--model" and i + 1 < len(sys.argv):
            model = sys.argv[i + 1]

    from .context import SessionDigest

    digest = SessionDigest(
        memory_dir=os.environ.get("RAK_MEMORY_DIR", "data/episodes"),
        max_tokens=int(os.environ.get("RAK_DIGEST_MAX_TOKENS", "3000")),
    )
    loop = AgentLoop(allow_motion_tools=allow_motion, model=model, digest=digest,
                     persona_file=os.environ.get("RAK_PERSONA_FILE", ""))
    asyncio.run(loop.run())


if __name__ == "__main__":
    main()
