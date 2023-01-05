

from dataclasses import dataclass
from typing import NewType, Tuple


PackedMaterial = NewType("PackedMaterial", int)


@dataclass
class Triangle:
    # the low 6 bit are the node flags, the rest are triangle-specific
    flags: PackedMaterial
    vert_indices: Tuple[int, int, int]
