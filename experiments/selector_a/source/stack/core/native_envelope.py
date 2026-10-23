"""Compile only the speed envelope's scalar backwards recurrence."""
import math
from numba import njit


@njit
def _backwards(cap, deceleration, exponent):
    envelope = cap.copy()
    for index in range(len(envelope)-2,-1,-1):
        envelope[index] = min(envelope[index], math.sqrt(envelope[index+1] ** exponent + 2*deceleration*1.))
    return envelope


def backwards(cap, deceleration):
    # NumPy scalar power uses pow. A runtime exponent prevents the compiled
    # integer square from becoming a multiplication with different rounding.
    return _backwards(cap, deceleration, 2.)
