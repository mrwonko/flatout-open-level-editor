

from dataclasses import dataclass
from typing import NewType, Tuple


# bits 0-5 = surface
# bits 8-13 might be brightness information
# bits 16-17 = flags, maybe connected to shadows and echo
PackedMaterial = NewType("PackedMaterial", int)


@dataclass
class Triangle:
    # triangle-specific surface index and flags
    flags: PackedMaterial
    vert_indices: Tuple[int, int, int]
