"""Command line for the brick wall fit + L0/L1 physics check (CPython only).

    uv run physics_check/physics_check.py run job.json result.json [--no-cra]

Reads the job written by the Grasshopper component (``physics_check_gh.py``),
runs steps 2-7 of ``physics_check_workflow.md`` (the ``pc`` package next to this
file), writes result.json and prints the report.  Exit code 0 when L0 and L1
both pass, 1 otherwise.  ``--no-cra`` runs L0 only, without compas_cra.
"""

from __future__ import print_function

import json
import sys

from pc.loop import run_check


def cli(argv):
    import argparse

    ap = argparse.ArgumentParser(description="Brick wall fit + L0/L1 physics check loop")
    sub = ap.add_subparsers(dest="cmd")
    r = sub.add_parser("run", help="run job.json and write result.json")
    r.add_argument("job")
    r.add_argument("result")
    r.add_argument("--no-cra", action="store_true", help="L0 only, skip compas_cra")
    args = ap.parse_args(argv)
    if args.cmd != "run":
        ap.print_help()
        return 2
    with open(args.job) as fh:
        job = json.load(fh)
    res = run_check(job, use_cra=not args.no_cra)
    with open(args.result, "w") as fh:
        json.dump(res, fh, indent=1)
    print(res["report"])
    return 0 if res.get("ok") else 1


if __name__ == "__main__":
    sys.exit(cli(sys.argv[1:]))
