import mathutils
from typing import Dict
from .bitmath import ones


def hex_color(hexcode: int) -> mathutils.Color:
    return mathutils.Color((
        (hexcode >> 16) / ones(8),
        ((hexcode >> 8) & ones(8)) / ones(8),
        (hexcode & ones(8)) / ones(8)
    ))


TEST_BITMASK = True
VERIFY_REACHABILITY = True
CHECK_BOUNDS = True
COLOR_SHALLOW = hex_color(0xD00070)
COLOR_DEEP = hex_color(0x0032A0)

# The keys here match the (1-based) indices in global/dynamics/surfaces.bed
COLOR_TARMAC = mathutils.Color((.2, .2, .2))
SURFACE_COLORS: Dict[int, mathutils.Color] = {
    # NoCollision
    1: mathutils.Color((1, 0, 1)),
    # Tarmac (Road)
    2: COLOR_TARMAC,
    # Tarmac Mark (Road) - identical physics as 2 -> same color
    3: COLOR_TARMAC,
    # Asphalt (Road) - identical physics as 2 -> same color
    4: COLOR_TARMAC,
    # Asphalt Mark (Road) - identical physics as 2 -> same color
    5: COLOR_TARMAC,
    # Cement Mark (Road) - identical physics as 2 -> same color
    6: COLOR_TARMAC,
    # Cement Mark (Road) - identical physics as 2 -> same color
    7: COLOR_TARMAC,
    # TODO: more colors
}
