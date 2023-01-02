from dataclasses import dataclass
import mathutils


@dataclass
class AABB:
    min: mathutils.Vector
    max: mathutils.Vector

    def copy(self) -> "AABB":
        return AABB(self.min.copy(), self.max.copy())
