"""Build smooth geometry for an existing consecutive lateral-edge requirement."""
import math

import numpy as np


def consecutive_lateral_chain(rp, sequence):
    edges = rp.lateral_edges(sequence)
    if len(edges) < 2 or edges != list(range(edges[0], edges[-1]+1)):
        return None
    return edges[0], edges[-1]


def route_cusps(route):
    from .geometry import wrap
    delta = np.diff(route.path.pts, axis=0)
    heading = np.arctan2(delta[:, 1], delta[:, 0])
    turns = np.abs(wrap(np.diff(heading)))
    return np.flatnonzero(turns > math.pi/2.)


def build_compound_routes(rp, route, start, heading, rect, fractions):
    """Use the declared adjacent-lane chain, retaining its source and final topology."""
    from .geometry import dedup, wrap
    from .route import _smoothstep5

    sequence = route.lane_seq
    chain = consecutive_lateral_chain(rp, sequence)
    if chain is None:
        return [], dict(reason='no_consecutive_lateral_chain')
    first, last = chain
    if set(rp.lateral_edges(sequence)).issubset(route.spans) and not len(route_cusps(route)):
        return [], dict(reason='existing_geometry_realizes_all_transitions')
    source_states = rp._corridor(sequence[:first+1])
    target_states = list(sequence[last+1:])
    source = rp._path(rp._poly(source_states), step=.5)
    target = rp._path(rp._poly(target_states), step=.5)
    anchor_station, _ = source.project(*start)
    grid = np.arange(max(0., anchor_station+3.), source.length, .5)
    if not len(grid):
        return [], dict(reason='source_has_no_remaining_corridor')
    points = np.column_stack(source.xy(grid))
    target_grid = np.arange(0., target.length, .5)
    target_points = np.column_stack(target.xy(target_grid))
    distances = np.linalg.norm(points[:, None]-target_points[None], axis=-1)
    nearest = np.argmin(distances, axis=1)
    gap = distances[np.arange(len(grid)), nearest]
    angles = np.abs(wrap(target.heading(target_grid[nearest])-source.heading(grid)))
    # The graph already requires every adjacent lateral edge in this chain.
    # This bound limits geometric search; the exact road gate is unchanged.
    width = (last-first+1)*rp.LAT_MAX
    usable = (gap > .5)&(gap <= width)&(angles < math.radians(30.))
    blocks, beginning = [], None
    for index, flag in enumerate(np.r_[usable, False]):
        if flag and beginning is None:
            beginning = index
        elif not flag and beginning is not None:
            blocks.append((beginning, index))
            beginning = None
    if not blocks:
        return [], dict(reason='no_directed_parallel_compound_corridor')
    lo_index, hi_index = max(blocks, key=lambda block: block[1]-block[0])
    lo, hi = float(grid[lo_index]), float(grid[hi_index-1])
    nominal = max(25., max(rp.v_ref, 5.)*rp.TRANSITION_S)
    results, rejected = [], []
    for fraction in fractions:
        a = lo+float(fraction)*max(0., hi-lo-8.)
        length = min(nominal, hi-a)
        if length < 8.:
            rejected.append(dict(fraction=fraction, reason='insufficient_compound_corridor'))
            continue
        target_station, _ = target.project(*source.xy(a))
        length = min(length, target.length-target_station)
        if length < 8.:
            continue
        travel = np.r_[np.arange(0., length, .5), length]
        weight = _smoothstep5(travel/length)[:, None]
        blend = (1.-weight)*np.column_stack(source.xy(a+travel))+weight*np.column_stack(
            target.xy(target_station+travel))
        before = np.column_stack(source.xy(np.arange(0., a, .5)))
        after = np.column_stack(target.xy(np.arange(target_station+length+.5, target.length, .5)))
        poly = dedup(np.vstack((before, blend, after)))
        candidate = rp.finish(poly, start, heading, rect, sequence, 'graph')
        if candidate.issues or len(route_cusps(candidate)):
            rejected.append(dict(fraction=fraction, reason=list(candidate.issues) or 'compound_route_cusp'))
            continue
        candidate.frac = {edge: float(fraction) for edge in range(first, last+1)}
        candidate.compound_edges = tuple(range(first, last+1))
        begin, _ = candidate.path.project(*blend[0])
        end, _ = candidate.path.project(*blend[-1], s_hint=begin, back=1., fwd=400.)
        candidate.spans[first] = (begin, end)
        candidate.span_xy = {first: (blend[0].copy(), blend[-1].copy())}
        candidate.compound_geometry = dict(source_start=a, source_end=a+length,
            target_start=target_station, target_end=target_station+length,
            verified_graph_edges=list(candidate.compound_edges), fraction=float(fraction))
        results.append(candidate)
    return results, dict(reason='consecutive_lateral_geometry', graph_edges=list(range(first, last+1)),
        source_states=[list(state) for state in source_states], target_states=[list(state) for state in target_states],
        parallel_start=lo, parallel_end=hi, candidate_count=len(results), rejected=rejected)


def install_compound(planner):
    routes, evidence = build_compound_routes(planner.rp, planner.route, planner._start,
                                              planner._heading, planner.rect, planner.FRACS)
    planner.compound_route_evidence = evidence
    if not routes:
        return False
    edge = routes[0].compound_edges[0]
    planner.cands = [(route, route.s0, edge, route.frac[edge]) for route in routes]
    planner.route, planner.s = routes[0], routes[0].s0
    planner._dynamic_active = True
    planner._route_diagnostics.update(active=True, reason='compound_lateral_geometry',
        candidate_count=len(routes), initial_distinct_candidates=len(routes))
    return True


def commit_compound(planner):
    route = planner.route
    if not hasattr(route, 'compound_edges'):
        return False
    first = route.compound_edges[0]
    for edge in route.compound_edges:
        planner.fixed[edge] = tuple(route.span_xy[first][0])
    planner.cands = [(route, planner.s, None, route.frac[first])]
    planner.compound_route_evidence.update(committed_edges=list(route.compound_edges),
        committed_fraction=route.frac[first], committed_geometry=route.compound_geometry)
    return True
