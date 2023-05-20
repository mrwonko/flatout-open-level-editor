

from dataclasses import dataclass
from typing import NewType, Tuple


# bits 0-5 = surface
# bits 8-13 and 16-17 = flags
PackedMaterial = NewType("PackedMaterial", int)


@dataclass
class Triangle:
    # triangle-specific surface index and flags
    flags: PackedMaterial
    vert_indices: Tuple[int, int, int]
