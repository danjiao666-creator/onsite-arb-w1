"""Compare paired OnSite results without approving a planner for promotion.

Python 3.8+, standard library only. Source/environment declarations are recorded
as evidence, not treated as verified merely because a manifest exists.
"""
import argparse
from collections import Counter, defaultdict
import hashlib
import json
import math
from pathlib import Path
import statistics
import sys


ARRIVAL = "到达"
COLLISION = "碰撞"
NORMAL_OUTCOMES = {ARRIVAL, COLLISION, "超时", "出界"}


class ComparisonError(ValueError):
    """The two inputs cannot support an exact paired comparison."""


def _number(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value))


def _index(document, side):
    if not isinstance(document, dict) or not isinstance(document.get("rows"), list):
        raise ComparisonError("{} must contain a rows list".format(side))
    if not document["rows"]:
        raise ComparisonError("{} has no scenario rows".format(side))
    indexed = {}
    for index, row in enumerate(document["rows"]):
        if not isinstance(row, dict):
            raise ComparisonError("{} row {} is not an object".format(side, index))
        name = row.get("name")
        if not isinstance(name, str) or not name.strip():
            raise ComparisonError("{} row {} has no valid name".format(side, index))
        if name in indexed:
            raise ComparisonError("{} duplicate scenario: {}".format(side, name))
        if not isinstance(row.get("end"), str) or not row["end"].strip():
            raise ComparisonError("{} scenario {} has no valid end".format(side, name))
        indexed[name] = row
    return indexed


def _has_error(row):
    return bool(row.get("error")) or row["end"] not in NORMAL_OUTCOMES


def _arrived(row):
    return row["end"] == ARRIVAL and not _has_error(row)


def _outcomes(rows):
    return {"n": len(rows), "outcomes": dict(sorted(Counter(r["end"] for r in rows).items())),
            "error_count": sum(_has_error(r) for r in rows)}


def _latency(rows):
    per_scene = []
    missing = []
    valid = []
    reports = []
    for row in rows:
        raw = row.get("latency")
        reasons = []
        if not isinstance(raw, dict):
            reasons.append("latency object missing")
        else:
            for key in ("mean", "p95", "maximum"):
                if not _number(raw.get(key)) or raw[key] < 0:
                    reasons.append("{} missing or invalid".format(key))
            if not reasons and (raw["mean"] > raw["maximum"] or raw["p95"] > raw["maximum"]):
                reasons.append("latency summary exceeds maximum")
            for key in ("p99", "first_call", "control_period_ms", "overtime_ratio"):
                if key in raw and (not _number(raw[key]) or raw[key] < 0):
                    reasons.append("{} invalid".format(key))
            for key in ("cycles", "ge200", "overtime_count", "raised_calls"):
                if key in raw and (not isinstance(raw[key], int) or isinstance(raw[key], bool)
                                   or raw[key] < (1 if key == "cycles" else 0)):
                    reasons.append("{} invalid".format(key))
            if "p99" in raw and _number(raw["p99"]) and not reasons:
                if not raw["p95"] <= raw["p99"] <= raw["maximum"]:
                    reasons.append("p99 inconsistent with p95/maximum")
        if reasons:
            missing.append({"name": row["name"], "reasons": reasons})
        else:
            valid.append(raw)
        reports.append(raw.get("passed") if isinstance(raw, dict) else None)
        per_scene.append({"name": row["name"], "raw": raw, "complete": not reasons})
    reported_fail = any(value is False for value in reports)
    all_reported_pass = (not missing and not any(_has_error(r) for r in rows)
                         and all(value is True for value in reports))
    return {
        "n": len(rows), "complete": not missing, "missing_or_invalid": missing,
        "source_reported_status": ("incomplete" if missing else "failure" if reported_fail
                                   else "pass" if all_reported_pass else "unknown"),
        "acceptance_status": "unknown",
        "acceptance_reason": "No timing contract or execution conditions are verified by this comparator.",
        "all_source_rows_report_pass": all_reported_pass,
        "max_scene_p95_ms": max((r["p95"] for r in valid), default=None),
        "maximum_ms": max((r["maximum"] for r in valid), default=None),
        "ge200_total": (sum(r["ge200"] for r in valid)
                        if len(valid) == len(rows) and all("ge200" in r for r in valid) else None),
        "pooled_percentiles": None,
        "aggregation_note": "Scene percentiles are not pooled percentiles; no cross-side timing ratio is inferred.",
        "per_scene": per_scene,
    }


def _score_pair(name, before, after):
    old = before.get("score") if isinstance(before.get("score"), dict) else None
    new = after.get("score") if isinstance(after.get("score"), dict) else None
    delta = {}
    if old is not None and new is not None:
        if _number(old.get("comfort")) and _number(new.get("comfort")):
            delta["comfort"] = new["comfort"] - old["comfort"]
        od, nd = old.get("comfort_detail"), new.get("comfort_detail")
        if isinstance(od, dict) and isinstance(nd, dict):
            delta["comfort_detail"] = {key: nd[key] - od[key] for key in sorted(od.keys() & nd.keys())
                                       if _number(od[key]) and _number(nd[key])}
    return {"name": name, "baseline": old, "candidate": new,
            "comfort_delta": delta or None}


def compare_documents(baseline, candidate):
    """Build a report; reject duplicate, incomplete, or mislabeled scene pairs."""
    before, after = _index(baseline, "baseline"), _index(candidate, "candidate")
    if before.keys() != after.keys():
        raise ComparisonError("scenario sets differ; missing from candidate={}; extra in candidate={}".format(
            sorted(before.keys() - after.keys()), sorted(after.keys() - before.keys())))
    names = sorted(before)
    roads = defaultdict(list)
    road_unknown = []
    for name in names:
        a, b = before[name].get("roadtype"), after[name].get("roadtype")
        if a and b and a != b:
            raise ComparisonError("roadtype differs for {}: {!r} vs {!r}".format(name, a, b))
        if not a or not b:
            road_unknown.append(name)
        if (a is not None and not isinstance(a, str)) or (b is not None and not isinstance(b, str)):
            raise ComparisonError("invalid roadtype for {}".format(name))
        roads[a or b or "未知"].append(name)
    common, invalid_times, scores = [], [], []
    for name in names:
        a, b = before[name], after[name]
        if _arrived(a) and _arrived(b):
            old_t, new_t = a.get("t_end"), b.get("t_end")
            ratio = (new_t / old_t if _number(old_t) and _number(new_t)
                     and old_t > 0 and new_t > 0 else None)
            if _number(ratio):
                common.append({"name": name, "baseline_t_end": old_t,
                               "candidate_t_end": new_t, "time_ratio": ratio})
            else:
                invalid_times.append({"name": name, "baseline_t_end": old_t,
                                      "candidate_t_end": new_t, "reason": "Both times and their ratio must be finite and positive."})
        scores.append(_score_pair(name, a, b))
    br, cr = [before[n] for n in names], [after[n] for n in names]
    changes = [{"name": n, "baseline_end": before[n]["end"], "candidate_end": after[n]["end"],
                "baseline_error": before[n].get("error"), "candidate_error": after[n].get("error")}
               for n in names if before[n]["end"] != after[n]["end"]
               or _has_error(before[n]) != _has_error(after[n])]
    return {
        "schema": "onsite_paired_comparison_v1", "comparison_status": "complete",
        "paired_scenarios": names,
        "total": {"baseline": _outcomes(br), "candidate": _outcomes(cr)},
        "by_roadtype": {road: {"baseline": _outcomes([before[n] for n in group]),
                               "candidate": _outcomes([after[n] for n in group])}
                        for road, group in sorted(roads.items())},
        "roadtype_missing_on_either_side": road_unknown,
        "gained": [n for n in names if _arrived(after[n]) and not _arrived(before[n])],
        "lost": [n for n in names if _arrived(before[n]) and not _arrived(after[n])],
        "new_collisions": [n for n in names if after[n]["end"] == COLLISION and before[n]["end"] != COLLISION],
        "new_errors": [n for n in names if _has_error(after[n]) and not _has_error(before[n])],
        "outcome_changes": changes,
        "common_arrivals": {"n": len(common) + len(invalid_times), "valid_time_pairs": common,
                            "invalid_time_pairs": invalid_times,
                            "median_time_ratio": statistics.median([r["time_ratio"] for r in common]) if common else None},
        "scores": {"note": "Original score fields are preserved. No score is recomputed or used to offset regressions.",
                   "per_scene": scores},
        "latency": {"baseline": _latency(br), "candidate": _latency(cr)},
        "evidence": {"fixed_source_identity": "unknown", "same_inputs": "unknown",
                     "same_environment": "unknown",
                     "note": "Matching scenario names do not prove identical input contents or runtime conditions."},
        "promotion": {"approved": False, "status": "not_evaluated",
                      "reason": "Comparison only; verified fixed source identity, identical inputs/environment, and an explicit acceptance contract are required."},
    }


def _read_json(path):
    def reject_constant(value):
        raise ComparisonError("non-finite JSON constant {} in {}".format(value, path))
    try:
        return json.loads(path.read_text(encoding="utf-8-sig"), parse_constant=reject_constant)
    except (OSError, ValueError) as exc:
        raise ComparisonError("Cannot read {}: {}".format(path, exc)) from exc


def _artifact_evidence(path):
    evidence = {"results_path": str(path.resolve()),
                "results_sha256": hashlib.sha256(path.read_bytes()).hexdigest(),
                "manifest": None, "manifest_status": "missing"}
    manifest_path = path.parent / "manifest.json"
    if manifest_path.exists():
        try:
            manifest = _read_json(manifest_path)
            if not isinstance(manifest, dict):
                raise ComparisonError("manifest is not an object")
            evidence.update(manifest_status="declared_unverified", manifest={
                key: manifest[key] for key in ("sources", "python", "numpy", "head", "jobs", "planner", "scenes")
                if key in manifest})
            evidence["manifest_sha256"] = hashlib.sha256(manifest_path.read_bytes()).hexdigest()
        except ComparisonError as exc:
            evidence.update(manifest_status="invalid", manifest_error=str(exc))
    return evidence


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--baseline", required=True, type=Path)
    parser.add_argument("--candidate", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path, help="Output JSON file")
    args = parser.parse_args(argv)
    try:
        if args.output.resolve() in (args.baseline.resolve(), args.candidate.resolve()):
            raise ComparisonError("output must not overwrite either input")
        report = compare_documents(_read_json(args.baseline), _read_json(args.candidate))
        report["artifacts"] = {"baseline": _artifact_evidence(args.baseline),
                               "candidate": _artifact_evidence(args.candidate)}
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    except (ComparisonError, OSError) as exc:
        print("Comparison failed: {}".format(exc), file=sys.stderr)
        return 2
    print("Compared {} scenarios; gained={}, lost={}, new_collisions={}, new_errors={}. Promotion not evaluated.".format(
        len(report["paired_scenarios"]), len(report["gained"]), len(report["lost"]),
        len(report["new_collisions"]), len(report["new_errors"])))
    print(str(args.output.resolve()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
