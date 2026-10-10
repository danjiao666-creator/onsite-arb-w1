"""Actual recorded full-body checks under the supplied mapped-road union."""
import numpy as np


def audit_frames(frames, boundary):
    if not frames or boundary.area is None:
        return dict(checked=False, reason='empty_trace_or_missing_map')
    states = np.array([[frame[key] for key in ('x', 'y', 'yaw', 'v', 'a', 'rot')] for frame in frames])
    road = boundary.inspect(states[None], frames[0]['L'], frames[0]['W'])
    areas = boundary.outside_area(states, frames[0]['L'], frames[0]['W'])
    covered = np.asarray(road['covered'][0], dtype=bool)
    first_outside = int(road['first_outside_step'][0])
    recovered = np.flatnonzero(areas <= 1e-7)
    first_recovery = int(recovered[0]) if len(recovered) else None
    allowance = np.full(len(areas), areas[0])
    if first_recovery is not None:
        allowance[first_recovery:] = 0.
    grows = np.flatnonzero(areas > allowance+1e-7)
    episodes, start = [], None
    for index in range(len(frames)+1):
        outside = index < len(frames) and not covered[index]
        if outside and start is None:
            start = index
        elif not outside and start is not None:
            episodes.append(dict(start_t=frames[start]['t'], end_t=frames[index-1]['t'],
                frames=index-start, max_exterior_area=float(areas[start:index].max()),
                initial_overhang=start == 0))
            start = None
    maximum = int(np.argmax(areas))
    return dict(checked=road['checked'], first_body_outside_t=frames[first_outside]['t']
        if first_outside >= 0 else None, first_exterior_growth_t=frames[int(grows[0])]['t']
        if len(grows) else None, initial_exterior_area=float(areas[0]),
        initial_exterior_recovered=first_recovery is not None,
        initial_recovery_t=frames[first_recovery]['t'] if first_recovery is not None else None,
        initial_exterior_unrecovered=bool(areas[0] > 1e-7 and first_recovery is None),
        exterior_allowance_policy='Initial exterior may not grow until recovery; any positive exterior after recovery is new',
        max_exterior_area=float(areas[maximum]), max_exterior_t=frames[maximum]['t'],
        max_exterior_state={key: frames[maximum][key] for key in ('x', 'y', 'yaw', 'v', 'a', 'rot')},
        outside_frame_count=int(road['outside_count'][0]), total_frames=len(frames), episodes=episodes,
        scope='Actual full body on supplied map grid; separate from official replay outcome')
