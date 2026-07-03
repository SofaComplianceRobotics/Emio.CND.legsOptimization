import math
from typing import Tuple

def helix_point(t: float,
                radius: float = 1.0,
                turns: float = 3.0,
                height: float = 5.0,
                center: list[float,float,float] = [0.0, 0.0, 0.0],
                start_angle: float = 0.0) -> Tuple[float,float,float]:
    """
    t: 0 -> start, 1 -> end
    returns (x,y,z)
    """
    if t < 0: t = 0
    if t > 1: t = 1
    angle = 2 * math.pi * turns * t + start_angle
    x = center[0] + radius * math.cos(angle)
    y = center[1] + radius * math.sin(angle)
    z = center[2] + height * t
    return [x, y, z]

# Example:
# point at halfway along a helix of radius=2, 4 turns, total height=10
p = helix_point(0, radius=2.0, turns=4.0, height=10.0)
print(p)