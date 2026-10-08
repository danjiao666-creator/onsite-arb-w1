"""Audit a complete 130-scene candidate against v1 and r5 without running planners.

Writes only the candidate's release_review.json. Every snapshot is checked against
its own manifest; no inactive-feature equivalence or arithmetic identity is assumed.
Complete-act timing is recomputed, but separate runs do not prove equal host load.
"""

import argparse
from collections import Counter
import gzip
import hashlib
import json
import math
from pathlib import Path
import re
import sys
import zipfile

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[1]
V1 = ROOT / "baselines/arb_w1_v1"
R5 = HERE / "runs/dynamic_route_r5_devreg130"
sys.dont_write_bytecode = True
sys.path.insert(0, str(ROOT / "bench"))
from compare_baseline import ComparisonError, compare_documents  # noqa: E402
from final_audit import calls_latency, difference, first_difference, normalize  # noqa: E402


class ReviewError(ValueError):
    """Malformed or incomplete artifacts cannot support a release review."""


def read(path):
    def reject(value):
        raise ReviewError("Nonfinite JSON {} in {}".format(value, path))

    try:
        result = json.loads(path.read_text(encoding="utf-8-sig"), parse_constant=reject)
    except (OSError, ValueError) as exc:
        raise ReviewError("Cannot read {}: {}".format(path, exc)) from exc
    if not isinstance(result, dict):
        raise ReviewError("Expected JSON object: {}".format(path))
    return result


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def safe_relative(root, name):
    name = name.replace("\\", "/")
    if not name or ":" in name or Path(name).is_absolute() or ".." in Path(name).parts:
        raise ReviewError("Unsafe artifact path: {}".format(name))
    result = (root / name).resolve()
    try:
        result.relative_to(root.resolve())
    except ValueError as exc:
        raise ReviewError("Artifact path escapes root: {}".format(name)) from exc
    return result


def zip_identity(path):
    with zipfile.ZipFile(path) as archive:
        names = [n for n in archive.namelist() if not n.endswith("/")]
        normalized = [n.replace("\\", "/") for n in names]
        if len(normalized) != len(set(normalized)):
            raise ReviewError("Duplicate archive entries: {}".format(path))
        for name in normalized:
            safe_relative(ROOT, name)
        return {
            norm: hashlib.sha256(archive.read(name)).hexdigest()
            for name, norm in zip(names, normalized)
        }


def load_bundle(root, folder, expected):
    required = [root / "manifest.json", root / "sets.json", root / "sources.zip"]
    required += [
        folder / name
        for name in (
            "results.json",
            "manifest.json",
            "verification.json",
            "summary.json",
            "runtime_sources.zip",
        )
    ]
    missing = [str(p) for p in required if not p.is_file()]
    if missing:
        raise ReviewError("Run incomplete; missing final artifacts: {}".format(missing))
    source = read(root / "manifest.json")
    manifest, verification, summary, results = (
        read(folder / name)
        for name in (
            "manifest.json",
            "verification.json",
            "summary.json",
            "results.json",
        )
    )
    rows = results.get("rows")
    if not isinstance(rows, list) or any(not isinstance(r, dict) for r in rows):
        raise ReviewError("Missing result rows: {}".format(folder))
    names = [r.get("name") for r in rows]
    scenes = manifest.get("scenes", [])
    own_sets = read(root / "sets.json")
    own_names = own_sets.get("DEV", []) + own_sets.get("REG", [])
    for label, values in (
        ("results", names),
        ("manifest", scenes),
        ("sets", own_names),
    ):
        if (
            any(not isinstance(n, str) for n in values)
            or len(values) != 130
            or len(values) != len(set(values))
            or set(values) != set(expected)
        ):
            raise ReviewError(
                "{} lacks the exact unique frozen DEV/REG130: {}".format(label, folder)
            )
    if summary.get("count") != 130 or summary.get("complete_dev_reg") is not True:
        raise ReviewError("Run lacks a completed DEV/REG130 summary: {}".format(folder))
    missing_logs = [
        name for name in names if not (folder / (name + ".json.gz")).is_file()
    ]
    if missing_logs:
        raise ReviewError("Missing scene logs: {}".format(missing_logs))
    return dict(
        root=root,
        folder=folder,
        source=source,
        manifest=manifest,
        verification=verification,
        summary=summary,
        results=results,
        rows={r["name"]: r for r in rows},
    )


def identity(bundle, current_hashes):
    root, folder = bundle["root"], bundle["folder"]
    sm, m, v = bundle["source"], bundle["manifest"], bundle["verification"]
    expected = normalize(sm["sources"])
    actual = {
        p.relative_to(root / "source").as_posix(): sha(p)
        for p in (root / "source").rglob("*.py")
    }
    archive = zip_identity(root / "sources.zip")
    imports = v.get("imported_stack_sources", {})
    import_issues = []
    for module, value in imports.items():
        path = Path(value).resolve()
        try:
            rel = path.relative_to((root / "source").resolve()).as_posix()
        except ValueError:
            import_issues.append(
                dict(module=module, path=value, reason="outside own snapshot")
            )
            continue
        if actual.get(rel) != expected.get(rel) or rel not in expected:
            import_issues.append(
                dict(
                    module=module,
                    path=value,
                    reason="imported source identity mismatch",
                )
            )
    inputs = normalize(m["inputs"])
    now = {}
    for name in inputs:
        path = safe_relative(ROOT, name)
        if name not in current_hashes:
            current_hashes[name] = sha(path) if path.is_file() else None
        now[name] = current_hashes[name]
    harness = normalize(m["harness_sources"])
    harness_archive = zip_identity(folder / "runtime_sources.zip")
    source_checks = dict(
        manifest_differences=difference(expected, actual),
        archive_entry_differences=difference(expected, archive),
        archive_sha256=sha(root / "sources.zip"),
        archive_hash_matches=sha(root / "sources.zip") == sm.get("archive_sha256"),
        runner_archive_hash_matches=m.get("archive_sha256") == sm.get("archive_sha256"),
        runner_sources_match=normalize(m.get("frozen_planner_sources", {})) == expected,
        sets_hash_matches=sha(root / "sets.json") == sm.get("sets_sha256"),
        imported_count=len(imports),
        import_issues=import_issues,
        runner_reported_unchanged=v.get("frozen_planner_unchanged"),
    )
    harness_checks = dict(
        archive_entry_differences=difference(harness, harness_archive),
        archive_sha256=sha(folder / "runtime_sources.zip"),
        recorded_archive_hash=m.get("runtime_archive_sha256"),
        archive_hash_matches=(
            sha(folder / "runtime_sources.zip") == m["runtime_archive_sha256"]
            if m.get("runtime_archive_sha256")
            else None
        ),
        runner_reported_unchanged=v.get("sources_unchanged"),
        current_worktree_comparison="not required; archived runtime is authoritative",
    )
    cache = dict(
        before_identity_recorded="map_cache_before" in m, records=[], archive_issues=[]
    )
    if "map_cache_before" in m:
        before = m["map_cache_before"]
        after = v.get("map_cache_after", {})
        archived = zip_identity(folder / "map_cache_before.zip")
        wanted = {
            item["path"].replace("\\", "/"): item["sha256"]
            for item in before.values()
            if item.get("sha256")
        }
        cache["archive_entry_differences"] = difference(wanted, archived)
        cache["archive_hash_matches"] = sha(folder / "map_cache_before.zip") == m.get(
            "map_cache_archive_sha256"
        )
        for scene, item in before.items():
            post = after.get(scene, {})
            existed = item.get("sha256") is not None
            cache["records"].append(
                dict(
                    scene=scene,
                    before=item,
                    after=post,
                    existing_unchanged=(item == post) if existed else None,
                )
            )
        cache["runner_reported_existing_unchanged"] = v.get(
            "existing_map_caches_unchanged"
        )
    else:
        cache["limitation"] = (
            "v1 cache archive is post-run only; before-run identity was not recorded"
        )
    checks = dict(
        source_valid=(
            not source_checks["manifest_differences"]
            and not source_checks["archive_entry_differences"]
            and source_checks["archive_hash_matches"]
            and source_checks["runner_archive_hash_matches"]
            and source_checks["runner_sources_match"]
            and source_checks["sets_hash_matches"]
            and bool(imports)
            and not import_issues
            and v.get("frozen_planner_unchanged") is True
        ),
        inputs_valid=(
            not difference(inputs, now) and v.get("inputs_unchanged") is True
        ),
        harness_valid=(
            not harness_checks["archive_entry_differences"]
            and harness_checks["archive_hash_matches"] is not False
            and v.get("sources_unchanged") is True
        ),
        cache_valid=(
            not cache["before_identity_recorded"]
            or (
                not cache["archive_entry_differences"]
                and cache["archive_hash_matches"]
                and all(r["existing_unchanged"] is not False for r in cache["records"])
                and v.get("existing_map_caches_unchanged") is True
            )
        ),
        run_identity_valid=(
            sm.get("baseline_id")
            == m.get("baseline_id")
            == bundle["summary"].get("baseline_id")
            and sm.get("planner") == m.get("planner")
            and bundle["summary"].get("runtime_identity_unchanged") is True
        ),
    )
    return dict(
        checks=checks,
        valid=all(checks.values()),
        source=source_checks,
        inputs=dict(
            current_differences=difference(inputs, now),
            runner_reported_unchanged=v.get("inputs_unchanged"),
        ),
        harness=harness_checks,
        map_caches=cache,
        artifacts={
            name: sha(folder / name)
            for name in (
                "results.json",
                "manifest.json",
                "verification.json",
                "summary.json",
            )
        },
    )


def almost_same(a, b):
    if isinstance(a, (int, float)) and isinstance(b, (int, float)):
        return math.isclose(a, b, rel_tol=1e-10, abs_tol=1e-9)
    return a == b


def load_scene(bundle, name):
    path = bundle["folder"] / (name + ".json.gz")

    def reject(value):
        raise ReviewError("Nonfinite JSON {} in {}".format(value, path))

    with gzip.open(str(path), "rt", encoding="utf-8") as stream:
        data = json.load(stream, parse_constant=reject)
    if not isinstance(data, dict) or any(
        not isinstance(data.get(k), list) for k in ("frames", "cycles", "act_calls")
    ):
        raise ReviewError("Invalid scene log: {}".format(path))
    row = bundle["rows"][name]
    result = data.get("result", {})
    issues = []
    if any(
        result.get(k) != row.get(k)
        for k in ("name", "end", "error", "t_end", "max_t", "dt", "score", "latency")
    ):
        issues.append("per-scene result differs from results index")
    frames, cycles, calls = data["frames"], data["cycles"], data["act_calls"]
    if not frames:
        issues.append("empty frame log")
    elif frames[-1].get("t") != row.get("t_end"):
        issues.append("terminal frame time differs from result t_end")
    if not row.get("error") and len(frames) != len(cycles) + 1:
        issues.append("normal run requires one terminal frame beyond action cycles")
    returned = [c for c in calls if c.get("status") == "returned"]
    if len(cycles) != len(returned):
        issues.append("returned act calls differ from recorded actions")
    if [c.get("t") for c in returned] != [c.get("t") for c in cycles]:
        issues.append("act-call/action timestamps differ")
    if [c.get("t") for c in cycles] != [f.get("t") for f in frames[: len(cycles)]]:
        issues.append("frame/action timestamps differ")
    for kind, records in (("frames", frames), ("cycles", cycles), ("calls", calls)):
        times = [r.get("t") for r in records]
        if any(not isinstance(t, (int, float)) or not math.isfinite(t) for t in times):
            issues.append(kind + " has invalid timestamps")
        elif any(b <= a for a, b in zip(times, times[1:])):
            issues.append(kind + " timestamps are not strictly increasing")
    for c in cycles:
        action = c.get("action")
        if (
            not isinstance(action, list)
            or len(action) != 2
            or any(
                not isinstance(x, (int, float)) or not math.isfinite(x) for x in action
            )
        ):
            issues.append("invalid action")
            break
    dt = row.get("dt")
    if not isinstance(dt, (int, float)) or not math.isfinite(dt) or dt <= 0:
        raise ReviewError("Invalid control dt: {}".format(name))
    latency = calls_latency(calls, dt, row.get("error"))
    if not latency["valid"]:
        issues.append("invalid complete-act timing")
    for key in (
        "mean",
        "p95",
        "p99",
        "maximum",
        "first_call",
        "cycles",
        "overtime_count",
        "overtime_ratio",
        "raised_calls",
        "passed",
    ):
        if not almost_same(result.get("latency", {}).get(key), latency.get(key)):
            issues.append("latency summary mismatch: " + key)
    return data, dict(name=name, **latency), issues, sha(path)


def trajectory_pair(old, new, name):
    a = [dict(t=c["t"], action=c["action"]) for c in old["cycles"]]
    b = [dict(t=c["t"], action=c["action"]) for c in new["cycles"]]
    da = [{k: v for k, v in c.items() if k != "ms"} for c in old["cycles"]]
    db = [{k: v for k, v in c.items() if k != "ms"} for c in new["cycles"]]
    aligned = [(x, y) for x, y in zip(a, b) if x["t"] == y["t"]]
    return dict(
        name=name,
        reference_frames=len(old["frames"]),
        candidate_frames=len(new["frames"]),
        reference_cycles=len(a),
        candidate_cycles=len(b),
        states_equal=old["frames"] == new["frames"],
        actions_equal=a == b,
        decisions_equal=da == db,
        first_state_difference=first_difference(old["frames"], new["frames"]),
        first_action_difference=first_difference(a, b),
        first_decision_difference=first_difference(da, db),
        aligned_cycles=len(aligned),
        differing_aligned_action_cycles=sum(
            x["action"] != y["action"] for x, y in aligned
        ),
        maximum_aligned_action_difference=max(
            (
                max(abs(u - v) for u, v in zip(x["action"], y["action"]))
                for x, y in aligned
            ),
            default=0.0,
        ),
    )


def review(tag):
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]*", tag):
        raise ReviewError("--tag must be a simple run directory name")
    candidate_root = HERE / "runs" / tag
    if candidate_root.resolve() == R5.resolve():
        raise ReviewError("Refusing to overwrite the sealed r5 release review")
    sets = read(V1 / "sets.json")
    expected = sets.get("DEV", []) + sets.get("REG", [])
    if len(expected) != 130 or len(set(expected)) != 130:
        raise ReviewError("Frozen reference sets are not unique DEV/REG130")
    bundles = {
        "v1": load_bundle(V1, V1 / "validation/devreg_20261007_r1", expected),
        "r5": load_bundle(R5, R5 / "validation", expected),
        "candidate": load_bundle(
            candidate_root, candidate_root / "validation", expected
        ),
    }
    current_hashes = {}
    identities = {
        side: identity(bundle, current_hashes) for side, bundle in bundles.items()
    }
    comparisons = {
        side: compare_documents(
            bundles[side]["results"], bundles["candidate"]["results"]
        )
        for side in ("v1", "r5")
    }
    integrity = []
    latency = {side: [] for side in bundles}
    pooled = {side: [] for side in bundles}
    logs = {side: {} for side in bundles}
    equality = {side: [] for side in ("v1", "r5")}
    exceptions = []
    for name in expected:
        scenes = {}
        for side, bundle in bundles.items():
            data, timing, issues, digest = load_scene(bundle, name)
            scenes[side] = data
            latency[side].append(timing)
            logs[side][name] = digest
            pooled[side].extend(c["ms"] for c in data["act_calls"])
            integrity.extend(dict(side=side, name=name, issue=i) for i in issues)
            raised = [c for c in data["act_calls"] if c.get("status") == "raised"]
            if data["result"].get("error") or raised:
                exceptions.append(
                    dict(
                        side=side,
                        name=name,
                        error=data["result"].get("error"),
                        raised_calls=raised,
                    )
                )
        for side in ("v1", "r5"):
            equality[side].append(
                trajectory_pair(scenes[side], scenes["candidate"], name)
            )
    timing = {}
    for side, values in latency.items():
        samples = np.asarray(pooled[side], float)
        timing[side] = dict(
            pass_count=sum(v["passed"] for v in values),
            fail_count=sum(not v["passed"] for v in values),
            per_scene=values,
            failures=[v["name"] for v in values if not v["passed"]],
            pooled=dict(
                n=len(samples),
                mean=float(samples.mean()),
                p95=float(np.percentile(samples, 95)),
                p99=float(np.percentile(samples, 99)),
                maximum=float(samples.max()),
            ),
        )
        b = bundles[side]
        if dict(Counter(r["end"] for r in b["rows"].values())) != b["summary"].get(
            "outcomes"
        ):
            integrity.append(
                dict(side=side, issue="final outcome summary disagrees with rows")
            )
        if timing[side]["pass_count"] != b["summary"].get(
            "measured_latency_pass_count"
        ):
            integrity.append(
                dict(
                    side=side, issue="final latency pass count disagrees with raw calls"
                )
            )
    for side, comp in comparisons.items():
        bm, cm = bundles[side]["manifest"], bundles["candidate"]["manifest"]
        inputs = difference(normalize(bm["inputs"]), normalize(cm["inputs"]))
        comp["verified_evidence"] = dict(
            input_differences=inputs,
            source_changes=difference(
                normalize(bundles[side]["source"]["sources"]),
                normalize(bundles["candidate"]["source"]["sources"]),
            ),
            runtime_source_differences=difference(
                normalize(bm["harness_sources"]), normalize(cm["harness_sources"])
            ),
            same_python_numpy=all(
                bm.get(k) == cm.get(k) for k in ("python", "executable", "numpy")
            ),
        )
        comp["frame_comparison"] = equality[side]
        comp["equivalence_counts"] = dict(
            scenes_states_equal=sum(r["states_equal"] for r in equality[side]),
            scenes_actions_equal=sum(r["actions_equal"] for r in equality[side]),
            scenes_decisions_equal=sum(r["decisions_equal"] for r in equality[side]),
        )
        comp["latency_changes"] = dict(
            newly_failing=sorted(
                set(timing["candidate"]["failures"]) - set(timing[side]["failures"])
            ),
            recovered=sorted(
                set(timing[side]["failures"]) - set(timing["candidate"]["failures"])
            ),
        )
    evidence_valid = (
        all(i["valid"] for i in identities.values())
        and not integrity
        and all(
            not c["verified_evidence"]["input_differences"]
            for c in comparisons.values()
        )
    )
    driving_passed = (
        evidence_valid
        and not any(e["side"] == "candidate" for e in exceptions)
        and all(
            not any(c[k] for k in ("lost", "new_collisions", "new_errors"))
            for c in comparisons.values()
        )
    )
    report = dict(
        schema="arb_dual_reference_release_review_v1",
        tag=tag,
        candidate=bundles["candidate"]["source"]["baseline_id"],
        scope=__doc__,
        paired_scene_count=130,
        identity=identities,
        comparisons=comparisons,
        record_integrity_issues=integrity,
        exceptions=exceptions,
        latency=timing,
        log_sha256=logs,
        verification=dict(
            evidence_valid=evidence_valid,
            driving_regression_passed=driving_passed,
            observed_all_scene_timing_passed=evidence_valid
            and timing["candidate"]["pass_count"] == 130,
        ),
        environment={
            side: dict(
                jobs=b["manifest"].get("jobs"),
                python=b["manifest"].get("python"),
                numpy=b["manifest"].get("numpy"),
                environment=b["manifest"].get("environment"),
                timing_scope=b["manifest"].get("timing_scope"),
                competing_before=b["manifest"].get("competing_python_before"),
                competing_after=b["verification"].get("competing_python_after"),
            )
            for side, b in bundles.items()
        },
        timing_limitation="Complete act including raised calls; separate runs and boundary process snapshots do not prove exclusive or equal host load. No causal speedup ratio or real-time certification is inferred.",
        safety_limitation="No new collisions and preserved successes are paired regression criteria, not a safety certification. Existing failures remain explicit.",
        promotion=dict(
            approved=False,
            default_promotion=False,
            status="review_only_no_automatic_promotion",
        ),
    )
    output = candidate_root / "release_review.json"
    output.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return report, output


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--tag", required=True)
    args = parser.parse_args(argv)
    try:
        report, output = review(args.tag)
    except (
        ReviewError,
        ComparisonError,
        OSError,
        KeyError,
        TypeError,
        zipfile.BadZipFile,
    ) as exc:
        print("Candidate review failed: {}".format(exc), file=sys.stderr)
        return 2
    print(
        json.dumps(
            dict(
                output=str(output),
                verification=report["verification"],
                outcomes=report["comparisons"]["r5"]["total"]["candidate"],
                paired={
                    side: {
                        k: c[k]
                        for k in ("gained", "lost", "new_collisions", "new_errors")
                    }
                    for side, c in report["comparisons"].items()
                },
                measured_latency_pass={
                    side: x["pass_count"] for side, x in report["latency"].items()
                },
            ),
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0 if report["verification"]["evidence_valid"] else 2


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    sys.exit(main())
