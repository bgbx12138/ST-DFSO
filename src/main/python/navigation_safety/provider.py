"""Existing TableEntry211-compatible RPC to the normal fixed safety function."""
from pathlib import Path
import argparse
import base64
import json
import os
import sys
import hashlib
from collections import Counter

if __package__ in (None, ""):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from navigation_safety.unified_response import SafetyFunction


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cache", type=Path, required=True)
    ap.add_argument("--jsonl-server", action="store_true")
    ap.add_argument("--tsv-rpc", action="store_true")
    ap.add_argument("--input-json", type=Path)
    ap.add_argument("--output", type=Path)
    ap.add_argument("--contract", action="store_true")
    ap.add_argument("--diagnostics", type=Path)
    args = ap.parse_args()
    with SafetyFunction(args.cache, diagnostics=args.diagnostics) as function:
        if args.contract:
            print(json.dumps(function.contract(), ensure_ascii=False, allow_nan=False))
            return
        if args.jsonl_server:
            if not args.tsv_rpc:
                ap.error("The registered Java transport requires --tsv-rpc")
            print("R1C\tREADY\t" + function.revision, flush=True)
            trace = os.environ.get("FSA_RESPONSE_TRACE")
            calls = os.environ.get("FSA_AVAILABILITY_CALLS")
            counts, unique = Counter(), set()
            call_count = 0
            try:
                for line in sys.stdin:
                    if not line.strip():
                        continue
                    request = json.loads(line)
                    if request.get("command") == "close":
                        break
                    phase_path = os.environ.get("FSA_AVAILABILITY_PHASE")
                    phase = Path(phase_path).read_text("utf-8").strip() if phase_path else "TEST"
                    context_path = os.environ.get("FSA_REQUEST_CONTEXT211")
                    context = (json.loads(Path(context_path).read_text("utf-8-sig"))
                               if context_path and Path(context_path).exists() else None)
                    before = function.stats["physical_solve_attempts"]
                    try:
                        if (phase == "SEARCH_EXECUTE" and context is not None
                                and float(context["remaining_seconds"]) <= 0):
                            physical = function._base.physics_input(request)
                            key = hashlib.sha256(function._base.encode(physical).encode()).hexdigest()
                            value = dict(status="MISSING_RESPONSE", reason="GLOBAL_SEARCH_DEADLINE_FIXED_FUNCTION",
                                sd=None, cost=None, input_sha256=key, physics_input=physical,
                                provenance=dict(physical_parent=function._identity["physical_parent"],
                                    weighted_parent=function._identity["weighted_parent"],
                                    physical_solve_attempted=False, physical_unsafe_inferred=False,
                                    control_stop_211="GLOBAL_SEARCH_DEADLINE_FIXED_FUNCTION"))
                        else:
                            value = function.evaluate(request)
                    except Exception as error:
                        value = dict(status="CONTRACT_FAILURE", reason=repr(error),
                            sd=None, cost=None, input_sha256="UNKNOWN", revision=function.revision,
                            provenance=dict(physical_solve_attempted=function.stats["physical_solve_attempts"] > before,
                                            physical_unsafe_inferred=False))
                    value["provenance"]["rpc_request_id"] = request.get("rpc_request_id")
                    call_count += 1
                    unique.add(value.get("input_sha256"))
                    screen = request.get("read_only_screen220", False)
                    counts[(phase, screen, value["status"])] += 1
                    if trace:
                        with Path(trace).open("a", encoding="utf-8", newline="\n") as f:
                            f.write(json.dumps(value, ensure_ascii=False, allow_nan=False) + "\n")
                            f.flush()
                    if calls:
                        row = dict(call=call_count,phase=phase,read_only_screen220=screen,status=value["status"],
                            input_sha256=value.get("input_sha256"),physics_input=value.get("physics_input"),
                            sd=value.get("sd"),cost=value.get("cost"),control_stop=value["provenance"].get("control_stop_211"),
                            physical_solve_attempted=value["provenance"]["physical_solve_attempted"],
                            caller_context=context,revision=function.revision)
                        with Path(calls).open("a", encoding="utf-8", newline="\n") as f:
                            f.write(json.dumps(row, ensure_ascii=False, allow_nan=False) + "\n")
                    fields = ["R1C", value["status"],
                        base64.b64encode(value.get("reason", "").encode()).decode(),
                        ",".join(map(str, value["sd"])) if value.get("sd") is not None else "",
                        value.get("input_sha256") or "UNKNOWN", value.get("solver_sha256", "UNKNOWN"),
                        str(value.get("seed", 20261018)), "NONE", "true",
                        str(value["cost"]) if value.get("cost") is not None else "null",
                        "1" if value.get("cache_hit") else "0", str(value.get("wall_seconds", 0)),
                        str(value.get("python_cpu_seconds", 0)), function.revision,
                        base64.b64encode(json.dumps(value["provenance"], ensure_ascii=False).encode()).decode()]
                    print("\t".join(fields), flush=True)
            finally:
                if calls:
                    with Path(calls).with_suffix(".summary.json").open("x", encoding="utf-8") as f:
                        json.dump(dict(status="NORMAL_FIXED_COMPONENT_CLOSED",calls=call_count,unique_inputs=len(unique),
                            counts=[dict(phase=k[0],read_only_screen220=k[1],status=k[2],count=v) for k,v in counts.items()],
                            new_physical_solves=function.stats["physical_solve_attempts"],original_cache_writes=0,
                            owned_runtime_cache=str(function.cache_path),revision=function.revision,
                            scene_specific_table_required=False),f,ensure_ascii=False,allow_nan=False)
                        f.flush();os.fsync(f.fileno())
        else:
            if not args.input_json:
                ap.error("--input-json required outside server mode")
            value = function.evaluate(json.loads(args.input_json.read_text("utf-8-sig")))
            text = json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2)
            if args.output:
                with args.output.open("x", encoding="utf-8", newline="\n") as f:
                    f.write(text + "\n")
            print(text)


if __name__ == "__main__":
    main()
