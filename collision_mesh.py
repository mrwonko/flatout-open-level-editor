

from dataclasses import dataclass
from typing import NewType, Tuple


# bits 0-5 = surface
# bits 8-13 and 16-17 = flags
PackedMaterial = NewType("PackedMaterial", int)


@dataclass
class Triangle:
    # the low 6 bit are the node flags, the rest are triangle-specific
    flags: PackedMaterial
    vert_indices: Tuple[int, int, int]
