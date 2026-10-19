#!/usr/bin/env python3
"""Build a plate with modeled M5×0.8 holes via SolidCraft control channel, export JSON+STEP.

Usage (app already running with --control PORT on DISPLAY=:1 for Take Control)::

    python3 skills/solidcraft-modeling/scripts/export_plate_holes.py \\
        --port 7920 --out /opt/cursor/artifacts \\
        --name Plate50_4xM5 --size 50 --height 10 --inset 12.5

Drill every hole first, then modeled ``solid.thread`` on each bore (see
``skills/solidcraft-modeling/reference.md``). For a browser download on the user's
machine, commit the exports under ``downloads/`` and push; print raw.githubusercontent.com
URLs (this script only writes files — push is the agent's follow-up).
"""

from __future__ import annotations

import argparse
import json
import os
import socket
import sys
from pathlib import Path


def rpc(host: str, port: int, token: str, method: str, params: dict | None, rid: int) -> dict:
    req: dict = {"id": rid, "method": method, "token": token}
    if params is not None:
        req["params"] = params
    raw = (json.dumps(req) + "\n").encode()
    with socket.create_connection((host, port), timeout=180) as s:
        s.sendall(raw)
        buf = b""
        while b"\n" not in buf:
            chunk = s.recv(1 << 20)
            if not chunk:
                break
            buf += chunk
    if not buf:
        raise RuntimeError(f"empty reply for {method}")
    return json.loads(buf.decode())


def run(host: str, port: int, token: str, allow: list[str], cmd: str, params: dict, rid: int) -> dict:
    body = {"command": cmd, "params": params, "allow_paths": allow}
    r = rpc(host, port, token, "engine.execute", body, rid)
    if not r.get("ok"):
        raise RuntimeError(f"{cmd} failed: {r}")
    return r


def default_token(port: int) -> str:
    path = Path.home() / ".local/share/solidcraft/control" / f"{port}.token"
    return path.read_text().strip()


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--host", default="127.0.0.1")
    p.add_argument("--port", type=int, default=7920)
    p.add_argument("--token", default="")
    p.add_argument("--out", default="/opt/cursor/artifacts")
    p.add_argument("--name", default="Plate50_4xM5")
    p.add_argument("--size", type=float, default=50.0, help="square plate XY size (mm)")
    p.add_argument("--height", type=float, default=10.0)
    p.add_argument("--inset", type=float, default=12.5, help="hole centre inset from each edge")
    p.add_argument("--thread", default="M5", help="thread designation (tap drill from chart)")
    p.add_argument("--tap-radius", type=float, default=2.1, help="wall pick offset; M5 tap Ø4.2 → 2.1")
    args = p.parse_args()

    token = args.token or default_token(args.port)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    allow = [str(out), "/tmp", "/opt/cursor/artifacts"]

    a, b = args.inset, args.size - args.inset
    centers = [(a, a), (b, a), (a, b), (b, b)]
    z_top = args.height
    z_mid = args.height / 2.0
    rid = 1

    run(args.host, args.port, token, allow, "file.new", {"name": args.name}, rid)
    rid += 1
    run(
        args.host,
        args.port,
        token,
        allow,
        "solid.box",
        {"length": args.size, "width": args.size, "height": args.height, "body_name": "Plate"},
        rid,
    )
    rid += 1
    for cx, cy in centers:
        run(
            args.host,
            args.port,
            token,
            allow,
            "solid.hole",
            {
                "position": [cx, cy, z_top],
                "thread": args.thread,
                "depth": args.height,
                "type": "drilled",
                "tip_angle": 118,
            },
            rid,
        )
        rid += 1
    for cx, cy in centers:
        run(
            args.host,
            args.port,
            token,
            allow,
            "solid.thread",
            {
                "face": [cx, cy + args.tap_radius, z_mid],
                "designation": args.thread,
                "modeled": True,
            },
            rid,
        )
        rid += 1

    measure = run(args.host, args.port, token, allow, "inspect.measure", {}, rid)
    rid += 1
    print(json.dumps(measure.get("result", measure), indent=2))

    json_path = out / f"{args.name}.json"
    step_path = out / f"{args.name}.step"
    run(
        args.host,
        args.port,
        token,
        allow,
        "file.export",
        {"path": str(json_path), "format": "json"},
        rid,
    )
    rid += 1
    run(
        args.host,
        args.port,
        token,
        allow,
        "file.export",
        {"path": str(step_path), "format": "step"},
        rid,
    )
    rid += 1

    print(f"wrote {json_path} ({json_path.stat().st_size} bytes)")
    print(f"wrote {step_path} ({step_path.stat().st_size} bytes)")
    print(
        "For a local browser download: commit under downloads/, push, and share "
        "https://raw.githubusercontent.com/<owner>/<repo>/<sha>/downloads/<file>"
    )
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as e:
        print(f"error: {e}", file=sys.stderr)
        sys.exit(1)
