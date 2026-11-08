"""Public-copy policy selected by the owner: 75 mm, not field revalidated."""
import math
MAX_INSERTION_DEPTH_MM = 75.0
INSERTION_DEPTH_M = MAX_INSERTION_DEPTH_MM / 1000.0

def checked_depth_mm(value):
    if isinstance(value, bool):
        raise ValueError('Insertion depth must be a distance in millimetres')
    depth = float(value)
    if not math.isfinite(depth) or not 0 < depth <= MAX_INSERTION_DEPTH_MM:
        raise ValueError('Insertion depth must be greater than 0 and no more than 75 mm')
    return depth
