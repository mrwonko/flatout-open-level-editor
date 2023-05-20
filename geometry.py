from dataclasses import dataclass
from enum import Enum
import operator
from typing import Callable, Iterable
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


class Axis(Enum):
    X = 0
    Y = 1
    Z = 2


class BoundKind(Enum):
    LOWER = 0
    UPPER = 1

    @property
    def inverse(self) -> "BoundKind":
        return BoundKind(1-self.value)

    @property
    def max(self) -> Callable[[int, int], int]:
        """
        Returns the function that returns the larger of two bounds,
        i.e. the one which encompasses the other.
        For a lower bound, this is the even lower bound.
        """
        return min if self == BoundKind.LOWER else max

    @property
    def contains(self) -> Callable[[int, int], bool]:
        return operator.ge if self == BoundKind.LOWER else operator.le


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

    @staticmethod
    def around(coordinates: Iterable[Vector]) -> "AABB":
        if len(coordinates) == 0:
            raise ValueError(
                "need at least one coordinate to create a bounding box")
        minimum = Vector(coordinates[0])
        maximum = Vector(minimum)
        for co in coordinates[1:]:
            for axis in range(3):
                minimum[axis] = min(co[axis], minimum[axis])
                maximum[axis] = max(co[axis], maximum[axis])
        return AABB(minimum, maximum)

    def bounds(self, bound_kind: BoundKind) -> Vector:
        return self.min if bound_kind == BoundKind.LOWER else self.max

    def bound(self, axis: Axis, bound_kind: BoundKind) -> int:
        # I kind of feel like this is the wrong place to round,
        # but I think Vector just isn't meant for ints...
        return round(self.bounds(bound_kind)[axis.value])

    def length_on(self, axis: Axis) -> int:
        return self.max[axis.value] - self.min[axis.value]

    def with_bound(self, axis: Axis, bound_kind: BoundKind, bound: int) -> "AABB":
        if bound_kind == BoundKind.LOWER:
            new_min = Vector(self.min)
            new_min[axis.value] = bound
            return AABB(new_min, Vector(self.max))
        else:
            new_max = Vector(self.max)
            new_max[axis.value] = bound
            return AABB(Vector(self.min), new_max)

    def extend(self, other: "AABB"):
        """
        Enlarges this AABB to encompass another.
        """
        for axis in Axis:
            self.min[axis.value] = min(
                self.min[axis.value], other.min[axis.value])
            self.max[axis.value] = max(
                self.max[axis.value], other.max[axis.value])


if __name__ == "__main__":
    # to test, run in the Blender text editor
    import doctest
    doctest.testmod(verbose=True)
