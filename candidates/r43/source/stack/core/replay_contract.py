"""Checked-control limits and operation order of the official replay."""
import numpy as np
from numba import njit

ACC_LIMIT = 9.8
JERK_LIMIT = 49.0
STEER_LIMIT = .699
STEER_RATE_LIMIT = 1.397
WHEELBASE_FACTOR = 1.7
COMMAND_STEER_RATE_FACTOR = .98


@njit
def checked_action(dt, speed, previous_acc, measured_steer, command_acc, command_steer):
    # Stop correction precedes jerk and magnitude clipping in check_action.
    acc, rot = command_acc, command_steer
    if not np.isnan(previous_acc):
        if speed + command_acc * dt < 0.:
            acc = -speed / dt
        else:
            jerk = (command_acc - previous_acc) / dt
            if abs(jerk) > JERK_LIMIT:
                acc = previous_acc + min(max(jerk, -JERK_LIMIT), JERK_LIMIT) * dt
    if not np.isnan(measured_steer):
        rate = (command_steer - measured_steer) / dt
        if abs(rate) > STEER_RATE_LIMIT:
            rot = measured_steer + min(max(rate, -STEER_RATE_LIMIT), STEER_RATE_LIMIT) * dt
    return (min(max(acc, -ACC_LIMIT), ACC_LIMIT),
            min(max(rot, -STEER_LIMIT), STEER_LIMIT))


def execution_limits_diagnostic(cfg, limits):
    return {
        'official': {'acceleration': [-ACC_LIMIT, ACC_LIMIT], 'jerk': JERK_LIMIT,
                     'steering': [-STEER_LIMIT, STEER_LIMIT], 'steer_rate': STEER_RATE_LIMIT,
                     'wheelbase_factor': WHEELBASE_FACTOR, 'position_integration': 'old_speed_yaw_euler',
                     'round_decimals': 3, 'stop_before_jerk': True},
        'controller': {'acceleration': [limits.acc_min, limits.acc_max],
                       'steering': [-limits.steer_max, limits.steer_max],
                       'command_steer_rate': limits.steer_rate * COMMAND_STEER_RATE_FACTOR,
                       'low_speed_hold_threshold': .3, 'own_jerk_limit': None,
                       'steer_memory': 'previous_command'},
        'main_profile': {'dt': cfg.dt, 'normal_jerk': cfg.jerk, 'emergency_jerk': cfg.jerk_emerg,
                         'emergency_jerk_exceeds_official': bool(cfg.jerk_emerg > JERK_LIMIT),
                         'envelope_applied_after_jerk': True,
                         'position_integration': 'speed_dt_plus_half_acc_dt_squared',
                         'ego_occupancy': 'ideal_path_heading_and_exponential_lateral_error'},
        'guard': {'checked_action': 'official_order', 'steer_memory': 'previous_command',
                  'initial_steer': 'measured', 'position_integration': 'old_speed_yaw_euler',
                  'round_decimals': 3, 'main_profile_execution_equivalence': False},
    }
