from dataclasses import dataclass
from mathutils import Vector


def vector_min(ls: Vector, rs: Vector) -> Vector:
    """
    Selects the component-wise minimums of two vectors.

    >>> vector_min(Vector((1, -5, 69)), Vector((-1, 0, 420)))
    Vector((-1.0, -5.0, 69.0))
    """
    return Vector([l if l < r else r for l, r in zip(ls, rs)])


def vector_max(ls: Vector, rs: Vector) -> Vector:
    """
    Selects the component-wise maximums of two vectors.

    >>> vector_max(Vector((1, -5, 69)), Vector((-1, 0, 420)))
    Vector((1.0, 0.0, 420.0))
    """
    return Vector([l if l > r else r for l, r in zip(ls, rs)])


def vector_abs(v: Vector) -> Vector:
    """
    Calculates the component-wise absolute values.

    >>> vector_abs(Vector((-42, 0, 69)))
    Vector((42.0, 0.0, 69.0))
    """
    return Vector([abs(e) for e in v])


@dataclass
class AABB:
    min: Vector
    max: Vector

    def copy(self) -> "AABB":
        return AABB(self.min.copy(), self.max.copy())

    def union(self, other: "AABB") -> "AABB":
        return AABB(
            min=vector_min(self.min, other.min),
        )


if __name__ == "__main__":
    # to test, run in the Blender text editor
    import doctest
    doctest.testmod(verbose=True)
