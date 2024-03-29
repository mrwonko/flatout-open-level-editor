from .mod_reload import reload_modules
reload_modules(locals(), __package__, [], [".bitmath"])  # nopep8

import mathutils
from typing import Dict
from .bitmath import ones


def hex_color(hexcode: int) -> mathutils.Color:
    return mathutils.Color((
        (hexcode >> 16) / ones(8),
        ((hexcode >> 8) & ones(8)) / ones(8),
        (hexcode & ones(8)) / ones(8)
    ))


# import

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
BITMASK_COLORS: Dict[int, mathutils.Color] = {
    0b1011: mathutils.Color((0.7, 0.7, 0.7)),
    0b0011: mathutils.Color((0, 0.7, 0.7)),
    0b0001: mathutils.Color((0, 0, 0.7)),
}

# export

# Since we apply a uniform scale to each axis, the result is more precise if the map is centered.
# If we use less than this much of the available coordinate space, warn.
COORDINATE_UTILIZATION_WARN_THRESHOLD = 0.75

# if the number of triangles in a node drops to or below this threshold, it must become a leaf
# (it may already become a leaf earlier due to other heuristics)
FORCE_LEAF_THRESHOLD = 5  # TODO: how high should this be?
# a node's score must be worse than this for us to consider a leaf instead
MAX_LEAF_SCORE = -1.5
# We only consider splits with at least this ratio,
# i.e. both partitions (sub-trees) must contain at least this fraction of triangles.
# This limits the maximum depth of the tree.
MIN_PARTITION_RATIO = 0.25

# For debugging: checks bounds for correctness on each step while building the tree.
# Very expensive.
VERIFY_TREE_BOUNDS = False
