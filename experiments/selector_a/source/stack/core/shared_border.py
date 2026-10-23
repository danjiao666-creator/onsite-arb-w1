"""Exact parser-owned border reconciliation reused from stack/traffic/roads.py.
No global buffering or inferred road coverage.
"""
import numpy as np
from shapely.geometry import Polygon

def _border_span(group, side):
    """Identify one parser-owned physical border and its complete parameter span.

    Different lane-width breakpoints may discretize this same Border differently.
    Missing provenance, partial spans and direction changes are not inferred from
    geometric proximity. In particular, this does not join opposite-direction
    lanes across their centre boundary.
    """
    pieces = getattr(group, "parametric_lanes", ())
    if not pieces:
        return None
    first = getattr(pieces[0], "border_group", None)
    border = getattr(first, side + "_border", None)
    reverse = getattr(pieces[0], "reverse", None)
    if border is None or reverse is None:
        return None
    spans = []
    for piece in pieces:
        bg = getattr(piece, "border_group", None)
        if (getattr(bg, side + "_border", None) is not border
                or getattr(piece, "reverse", None) != reverse):
            return None
        try:
            start = float(getattr(bg, side + "_border_offset", None))
            end = start + float(getattr(piece, "length", None))
        except (TypeError, ValueError):
            return None
        if not np.isfinite([start, end]).all() or end <= start:
            return None
        spans.append((start, end))
    spans.sort()
    lo, hi = spans[0]
    for start, end in spans[1:]:
        if start != hi:
            return None
        hi = end
    return border, lo, hi, reverse

def _shared_border_vertices(discrete):
    """Use the owning inner lane's outer polyline on both sides of that border.

    Reuses the parser's existing samples, including its one-sided breakpoint
    semantics. No resampling, buffering, hole filling or permission inference.
    All outer vertices and centre paths remain the original parser output.
    """
    replacements, records = {}, []
    for key, lane in discrete.items():
        group = getattr(lane, "_parametric_lane_group", None)
        owner_key = str(getattr(group, "inner_neighbour", None))
        owner = discrete.get(owner_key)
        owner_group = getattr(owner, "_parametric_lane_group", None)
        if (
            owner is None
            or not getattr(group, "inner_neighbour_same_direction", False)
            or str(getattr(owner_group, "outer_neighbour", None)) != key
        ):
            continue
        inner = _border_span(group, "inner")
        outer = _border_span(owner_group, "outer")
        if (
            inner is None or outer is None
            or inner[0] is not outer[0] or inner[1:] != outer[1:]
        ):
            continue
        old = np.asarray(lane.left_vertices, float)
        shared = np.asarray(owner.right_vertices, float)
        if (
            old.ndim != 2 or shared.ndim != 2
            or old.shape[1] != 2 or shared.shape[1] != 2
            or len(old) < 2 or len(shared) < 2
            or not np.isfinite(old).all() or not np.isfinite(shared).all()
            or not np.array_equal(old[[0, -1]], shared[[0, -1]])
            or np.array_equal(old, shared)
        ):
            continue
        before = Polygon(np.vstack((old, lane.right_vertices[::-1])))
        after = Polygon(np.vstack((shared, lane.right_vertices[::-1])))
        if not before.is_valid or not after.is_valid:
            continue
        replacements[key] = shared
        records.append(dict(
            lane=key, side="left", owner=owner_key, owner_side="right",
            parameter_span=list(inner[1:3]), reverse=bool(inner[3]),
            original_samples=len(old), shared_samples=len(shared),
            provenance="same parser Border object, full parameter span and reciprocal same-direction neighbours",
        ))
    return replacements, records
