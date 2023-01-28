from .mod_reload import reload_modules
reload_modules(locals(), __package__, ["cdb2", "config", "collision_mesh"], [".bitmath", ".geometry", ".list"])  # nopep8

from .bitmath import ones
from dataclasses import dataclass
from typing import Callable, Dict, Generator, Iterable, List, NewType, Optional, Set, Tuple, Union, cast
from .geometry import AABB, Axis, BoundKind, vector_abs, vector_max, vector_min
from .list import LinkedList
from . import cdb2, collision_mesh, config
from mathutils import Vector
import bpy


class ExportError(RuntimeError):
    def __init__(self, message: str) -> None:
        super()
        self.message = message


def find_collision_meshes() -> List[bpy.types.Object]:
    try:
        col = bpy.data.collections['collision']
    except KeyError as e:
        raise ExportError(f"no \"collision\" collection found") from e
    return [o for o in col.all_objects.values() if isinstance(o.data, bpy.types.Mesh)]


def common_bounding_box(objects: Iterable[bpy.types.Object]) -> AABB:
    """
    Returns the axis-aligned bounding box of all given objects in world space.

    At least one of the given objects must have at least one face.
    """
    min: Optional[Vector] = None
    max: Optional[Vector] = None
    for obj in objects:
        mesh = cast(bpy.types.Mesh, obj.data)
        # iterate over polygons instead of vertices to skip loose vertices
        for poly in mesh.polygons:
            for vert_index in poly.vertices:
                vert = mesh.vertices[vert_index]
                local_co = vert.co
                world_co = obj.matrix_world @ local_co
                min = world_co if min is None else vector_min(min, world_co)
                max = world_co if max is None else vector_max(max, world_co)
    if min == None or max == None:
        raise ExportError("collision mesh has no faces")
    return AABB(min=min, max=max)


report_func = Callable[[Union[Set[int], Set[str]], str], None]


def check_coordinate_space_utilization(report: report_func, bounds: AABB) -> Vector:
    """
    Reports a warning if the bounds are too far off-center.
    Since coordinates get mapped into an int32 range with a single factor per axis,
    if the model extends further in one direction, coordinate space is wasted.
    Returns the maximum absolute bounds, from which the scaling factor derives.
    """
    max_abs_bounds = vector_max(vector_abs(bounds.min), vector_abs(bounds.max))
    abs_center = vector_abs((bounds.min + bounds.max) / 2)
    violators: List[str] = []
    for axis in range(3):
        try:
            utilization = 1 - (abs_center[axis] / max_abs_bounds[axis])
        except ZeroDivisionError:
            # This can only happen when all vertices have X=0 or Y=0 or Z=0,
            # and that would be a pretty useless collision mesh anyway...
            continue
        if utilization < config.COORDINATE_UTILIZATION_WARN_THRESHOLD:
            violators.append("XYZ"[axis])
    print(f"{violators=}")
    if len(violators) > 0:
        axes = f"{violators[0]} axis" if len(violators) == 1 \
            else f"{', '.join(violators)} axes"
        report(
            {'WARNING'},
            f"Using less than {100 * config.COORDINATE_UTILIZATION_WARN_THRESHOLD:.0f}% of the {axes} coordinate space, try centering the map to improve the collision mesh precision")
    return max_abs_bounds


# In keeping with the original collision meshes, we use the same bound for both negative and positive values,
# even though negative values can technically extend one further.
MAX_ABS_COORDINATE = (1 << (cdb2.COORDINATE_BITS - 1)) - 1


def calculate_axis_multipliers(report: report_func, collision_meshes: Iterable[bpy.types.Object]) -> Vector:
    if len(collision_meshes) == 0:
        raise ExportError("no meshes found in \"collision\" collection")

    bounds = common_bounding_box(collision_meshes)
    print(f"{bounds=}")
    max_abs_bounds = check_coordinate_space_utilization(
        report=report,
        bounds=bounds)
    return Vector([MAX_ABS_COORDINATE/e for e in max_abs_bounds])


vec3_scaled = NewType("vec3_scaled", Tuple[int, int, int])


class VertexEncoder:
    """
    Scales vertex coordinates into the 16 bit range using the axis multipliers.
    """

    def __init__(self, axis_multipliers: Vector) -> None:
        self._axis_multipliers = axis_multipliers
        self._mapping: Dict[vec3_scaled, int] = {}
        self._vertices: List[vec3_scaled] = []

    def upsert(self, world_vert: Vector) -> int:
        scaled_vert = self._scale_vert(world_vert)
        try:
            return self._mapping[scaled_vert]
        except KeyError:
            idx = len(self._vertices)
            self._vertices.append(scaled_vert)
            self._mapping[scaled_vert] = idx
            return idx

    @property
    def vertices(self) -> List[vec3_scaled]:
        return self._vertices

    def _scale_vert(self, vert: Vector) -> vec3_scaled:
        res = vec3_scaled(
            tuple(round(self._axis_multipliers[i]*vert[i]) for i in range(3)))
        # sanity check: ensure we're within the valid range
        assert all(c >= -MAX_ABS_COORDINATE and c <= MAX_ABS_COORDINATE for c in res), \
            f"scaling {vert=} by multiplier={self._axis_multipliers} resulted in {res}, which exceeds [{-MAX_ABS_COORDINATE}, {MAX_ABS_COORDINATE}]"
        return res


def triangle_area(a: Vector, b: Vector, c: Vector) -> float:
    e1 = b - a
    e2 = c - a
    return e1.cross(e2).length / 2


@dataclass
class Triangle:
    # the data relevant for writing the file
    collision: collision_mesh.Triangle
    # the cached bounding box, which is used for building the collision tree
    aabb: AABB
    # Mutable partition flag.
    # We maintain six indices into the triangles, one for each axis and bound, to quickly enumerate all sensible partitions.
    # These indices get partitioned recursively as we build the collision tree, using this flag.
    extract: bool = False


class TriangleEncoder:
    def __init__(self, vertex_encoder: VertexEncoder) -> None:
        self._vertex_encoder = vertex_encoder
        self._tris: List[Triangle] = []

    def append(self, verts: Tuple[Vector, Vector, Vector], flags: collision_mesh.PackedMaterial) -> bool:
        """
        Encodes a triangle into the fixed-point coordinate system used by the file.
        Returns whether the triangle was valid.
        Degenerate triangles (i.e. with collinear vertices) are invalid and get dropped.
        This can happen due to the rounding involved in quantizing the coordinates.
        """
        vert_indices: Tuple[int, int, int] = tuple(
            self._vertex_encoder.upsert(v) for v in verts)
        degenerate = triangle_area(
            *(Vector(self._vertex_encoder.vertices[idx]) for idx in vert_indices)) == 0
        if degenerate:
            # It's somewhat unfortunate that in this case, the vertices still get upserted.
            # Ideally, those insertions should be rolled back, or we may save unused vertices.
            # But that should be relative harmless, and the situation should be avoided anyway.
            return False
        self._tris.append(Triangle(collision=collision_mesh.Triangle(
            flags=flags, vert_indices=vert_indices), aabb=AABB.around(verts)))
        return True

    @property
    def triangles(self) -> List[Triangle]:
        return self._tris


@dataclass
class DegenerateTriangle:
    object: bpy.types.Object
    polygon_index: int


def encode_flags(surface: int, flags: Iterable[bool]) -> collision_mesh.PackedMaterial:
    packed_flags = 0
    for i, f in enumerate(flags):
        if f:
            packed_flags |= 1 << i
    loflags = packed_flags & ones(6)
    hiflags = (packed_flags >> 6)
    return collision_mesh.PackedMaterial((surface - 1) | (loflags << 8) | (hiflags << 8+8))


def encode_triangles(axis_multipliers: Vector, collision_meshes: Iterable[bpy.types.Object]) -> Tuple[List[vec3_scaled], List[Triangle], List[DegenerateTriangle]]:
    vertex_encoder = VertexEncoder(axis_multipliers=axis_multipliers)
    triangle_encoder = TriangleEncoder(vertex_encoder=vertex_encoder)
    degenerates: List[DegenerateTriangle] = []
    for obj in collision_meshes:
        mesh = cast(bpy.types.Mesh, obj.data)
        for tri in mesh.loop_triangles:
            mat: bpy.types.Material = mesh.materials[tri.material_index]
            assert len(tri.vertices) == 3
            # convert back from 1-based surfaces.bed index to 0-based
            ok = triangle_encoder.append(
                verts=tuple(obj.matrix_world @
                            mesh.vertices[idx].co for idx in tri.vertices),
                flags=encode_flags(
                    surface=mat.fo2.collision_surface, flags=mat.fo2.collision_flags),
            )
            if not ok:
                degenerates.append(DegenerateTriangle(
                    object=obj, polygon_index=tri.polygon_index))
    return vertex_encoder.vertices, triangle_encoder.triangles, degenerates


"""
ideas/thoughts on/sketch for building the r-tree
    along each axis:
    sort tris once each by mins and maxs
    or maybe just by their center
efficiently query:
    how many mins/maxs are on either side of a given value on an axis,
    or rather: bisect where for most even split into partitions?
    then choose axis with smallest overlap? Is that a good heuristic?
    keep descending until when?
    at least until leafs are small enough (127 tris?)
    and then still while partition overlap is low
efficiently update:
    partition sorted lists into two new lists for child nodes, with overlap (!?!), retaining order
    along each axis
    sort for efficient hitbox check (huh, that's what I need anyway)
    probably once by lower and once by upper bound along each axis,
    with the ability to derive new partitions
    do a binary search for the ideal point to partition
    TODO: what does that entail?
    things to consider:
    number of elements left, right and overlapping
    reduction of search space of either child
    weigh such that 10%-10% is better than 50%-50% is better than 10%-90%
    score using the smaller reduction (e.g. when splitting into 70%/40%, the score is 70%) and look for greatest such reduction,
    breaking ties based on the other reduction
    partition into left, right and overlap
    determine bounds (based on parent bounds and overlap bounds?)
    how to efficiently do the partitioning, while allowing overlap?
I'll have to duplicate the overlap,
so a linked list would get copied lots and seems unsuitable.
but a (balanced?) tree might do?
But how do I partition efficiently?
I want to iterate through one axis and update the others, so do I want intrusive containers?
How would an intrusive container look, that can be split into two overlapping partitions through random (iterator) access?
I have some vague ideas, but they are complicated. Too complicated?
But _does_ it get simpler without intrusive containers?
The absolute order remains fixed. Can I just store the absolute index along each axis? But once I start partitioning, those become less useful?
    Actually, I got the overlap wrong.
It's always a perfect bisection, each tri goes in exactly one of the children.
But there are triangles which could potentially go into either one, I think?
So for these, one of them must be chosen (optimisation opportunity! but find a cheap heuristic).
So I partition a sorted sequence into two sorted sub-sequences.
Once for each axis.
To avoid allocations for the partition, I could fill a buffer from both ends.
But I then have to recursively do that for a sub-range of it, at which point double-buffering breaks down.
Do I want trees?
    Instead of trying to break down the set of all triangles,
I could also try to insertion sort it?
But it feels like those insertions will be prohibitively complex?
    I had the thought of doing a sweep down an axis,
but while one bound could be adjusted iteratively,
the other one would have to be re-calculated from all remaining elements each step?!?
But: can this ever be avoided?
        Inspired by Guttman:
    seed partitions with the extreme bounds (i.e. tris spanning the maximum bounds)
while unpartitioned triangles remain:
    find the triangle where the choice of partition matters the most (i.e. range grows most)
    assign it to the more suitable partition
    then I can do that on each axis and pick the one that reduces the search space the most.
        another sketch
    def
"""


class SortedTriangles:
    def __init__(self, by_axis_and_bound: Tuple[
        Tuple[LinkedList[Triangle], LinkedList[Triangle]],
        Tuple[LinkedList[Triangle], LinkedList[Triangle]],
        Tuple[LinkedList[Triangle], LinkedList[Triangle]],
    ]) -> None:
        """
        The same triangles sorted by their bounds along each axis.

        Each node in the tree sweeps along one axis from one direction and puts everything up to a point into one child.
        This partition retains the relative order of triangles, so we only need to sort them once.
        Then we can recursively partition the lists of triangles until they're small enough for a leaf.
        """
        l: Optional[int] = None
        for by_bound in by_axis_and_bound:
            for tris in by_bound:
                if l == None:
                    l = len(tris)
                else:
                    assert l == len(tris), ""
        self.__by_axis_and_bound = by_axis_and_bound

    def by_axis_and_bound(self, axis: Axis, bound_kind: BoundKind) -> LinkedList[Triangle]:
        return self.__by_axis_and_bound[axis.value][bound_kind.value]

    def extract_marked(self) -> "SortedTriangles":
        """
        Extracts all triangles marked with extract=True, removing them from this index.
        """
        old_len = len(self)
        extracted = SortedTriangles(
            by_axis_and_bound=tuple(
                tuple(tris.extract(lambda tri: tri.extract)
                      for tris in by_bound)
                for by_bound in self.__by_axis_and_bound
            ),
        )
        assert len(self) + len(extracted) == old_len
        return extracted

    def __len__(self) -> int:
        # the lengths of all elements ought to be identical
        return len(self.by_axis_and_bound(Axis.X, BoundKind.LOWER))

    def __iter__(self) -> Generator[Tuple[Axis, BoundKind, LinkedList[Triangle]], None, None]:
        for axis in Axis:
            for bound_kind in BoundKind:
                yield axis, bound_kind, self.by_axis_and_bound(axis, bound_kind)


def build_tree(sorted_tris: SortedTriangles, aabb: AABB) -> int:
    """
    .. image:: collision_export_tri_partition_algo.jpg
    """
    if len(sorted_tris) <= config.FORCE_LEAF_THRESHOLD:
        # TODO build leaf
        return 1

    @dataclass
    class PivotCandidate:
        index: int
        pivot: int
        inverse_pivot: int
        score: int

    @dataclass
    class Candidate:
        axis: Axis
        bound_kind: BoundKind
        min_tri_length: int
        pivot: PivotCandidate

        @property
        def score(self) -> float:
            # within an axis, we can compare absolute total child length,
            # but across axes we should compare relative child length
            # TODO: take min_tri_length/aabb.length_on(self.axis) into account?
            try:
                # this should be within [-2, 0]
                return self.pivot.score / aabb.length_on(self.axis)
            except ZeroDivisionError:
                # if this axis is already empty, it's as bad as two 100% children
                return -2

    best_candidate: Optional[Candidate] = None
    for axis, bound_kind, tris in sorted_tris:
        inverse_bound_kind = bound_kind.inverse
        outer_bound = aabb.bound(axis, bound_kind)
        inverse_outer_bound = aabb.bound(axis, bound_kind.inverse)

        def score(pivot: int, inverse_pivot: int) -> int:
            # We score by the combined size of the children,
            # hopefully that will quickly reduce the physical search space,
            # if not the number of elements...
            # But we have to invert the result so smaller children score more.
            # Note that this is based on the assumption that checks are equally likely to occur anywhere.
            # In practice, they probably mostly happen where lots of objects are?
            return -abs(outer_bound - inverse_pivot) - abs(inverse_outer_bound - pivot)

        def evaluate(index: int, pivot: int, inverse_pivot: int) -> PivotCandidate:
            return PivotCandidate(
                index=index,
                pivot=pivot,
                inverse_pivot=inverse_pivot,
                score=score(pivot, inverse_pivot)
            )

        tri_iter = iter(enumerate(tris))
        # For the elements before the pivot, we have to accumulate the inverse bound,
        # a running extremum.
        # For the elements after the pivot, we can use the bound of the first element,
        # thanks to the sorting.
        # We need at least one element in either partition.

        # First candidate: the first partition is just the first triangle.
        i, tri = next(tri_iter)
        # Since this is the only element, we can just use its inverse bound.
        inverse_pivot = tri.aabb.bound(axis, inverse_bound_kind)

        # While we're iterating over all triangles, let's also find the shortest one.
        # That tells us how much smaller our partitions might get.
        min_tri_length = tri.aabb.length_on(axis)

        # Now let's look at the other partition:
        i, tri = next(tri_iter)
        # Because the triangles are sorted by their bound, we just need to look at the first element's bound
        pivot = tri.aabb.bound(axis, bound_kind)
        best = evaluate(i, pivot, inverse_pivot)

        min_tri_length = min(min_tri_length, tri.aabb.length_on(axis))

        # Now move the pivot one step further.
        # For the inverse bound, that means we need to calculate the maximum of the two elements so far.
        inverse_pivot = inverse_bound_kind.max(
            inverse_pivot, tri.aabb.bound(axis, inverse_bound_kind))

        # search for better candidates
        for i, tri in tri_iter:
            pivot = tri.aabb.bound(axis, bound_kind)
            candidate = evaluate(i, pivot, inverse_pivot)
            if candidate.score > best.score:
                best = candidate

            min_tri_length = min(min_tri_length, tri.aabb.length_on(axis))

            # again, the new bound derives from the previous one and this tri
            inverse_pivot = inverse_bound_kind.max(
                inverse_pivot, tri.aabb.bound(axis, inverse_bound_kind))
        candidate = Candidate(
            axis=axis,
            bound_kind=bound_kind,
            min_tri_length=min_tri_length,
            pivot=best,
        )
        if best_candidate is None or candidate.score > best_candidate.score:
            best_candidate = candidate

    if len(sorted_tris) >= 100:
        print(f"{best_candidate.bound_kind.name} {best_candidate.pivot.index} of {len(sorted_tris)} tris on {best_candidate.axis.name} axis")
    # TODO: check if it makes sense to put a leaf here
    prev_len = len(sorted_tris)
    assert 0 < best_candidate.pivot.index < prev_len, \
        f"pivot index {best_candidate.pivot.index} violates 0 < index < len ({prev_len})"

    # mark selected triangles for extraction...
    for i, tri in enumerate(sorted_tris.by_axis_and_bound(best_candidate.axis, best_candidate.bound_kind)):
        tri.extract = i >= best_candidate.pivot.index
    # ... and extract them
    extracted = sorted_tris.extract_marked()
    assert len(sorted_tris) == best_candidate.pivot.index, \
        f"expected cut at element {best_candidate.pivot.index}/{prev_len}, not {len(sorted_tris)}"
    assert len(sorted_tris) < prev_len
    assert len(extracted) < prev_len

    d1 = build_tree(sorted_tris, aabb.with_bound(best_candidate.axis,
                                                 best_candidate.bound_kind.inverse, best_candidate.pivot.inverse_pivot))
    d2 = build_tree(extracted, aabb.with_bound(best_candidate.axis,
                                               best_candidate.bound_kind, best_candidate.pivot.pivot))
    # TODO: build inner node
    return max(d1, d2) + 1


def export_file(report: report_func, path: str) -> None:
    print(f"export to \"{path}\"")

    collision_meshes = find_collision_meshes()
    print(f"found {len(collision_meshes)} collision meshes")

    axis_multipliers = calculate_axis_multipliers(
        report=report,
        collision_meshes=collision_meshes)
    print(f"{axis_multipliers=}")

    verts, tris, degenerates = encode_triangles(
        axis_multipliers=axis_multipliers,
        collision_meshes=collision_meshes,
    )
    if (num_degenerates := len(degenerates)) > 0:
        # TODO: provide an operator for identifying degenerate triangles
        report(
            {'WARNING'},
            f"{num_degenerates} triangle{'s' if num_degenerates > 0 else ''} ignored due to being too small to represent"
        )
    print(f"encoded {len(tris)} triangles using {len(verts)} vertices")
    if len(tris) == 0:
        report({'ERROR'}, "no valid triangles found")
        return
    aabb = tris[0].aabb.copy()
    print(f"{aabb=}")
    for tri in tris[1:]:
        aabb.extend(tri.aabb)
    sorted_tris = SortedTriangles(by_axis_and_bound=tuple(
        tuple(
            # By pre-sorting, we can later iterate through all possible partitions along this axis and bound in one linear pass.
            # We store the result in a linked list, because we can partition that without any allocations while retaining its order,
            # so we can recursively partition the list into a tree.
            LinkedList.of(sorted(
                tris,
                key=lambda tri: (
                    tri.aabb.bound(axis, bound_kind),
                    tri.aabb.bound(axis, bound_kind.inverse)
                ),
                reverse=bound_kind == BoundKind.UPPER,
            ))
            for bound_kind in BoundKind
        )
        for axis in Axis
    ))
    print("triangles pre-sorted")
    depth = build_tree(sorted_tris, aabb)
    print(
        f"built tree with detph {depth} for {len(tris)} triangles")


class ExportOperator(bpy.types.Operator):
    bl_idname = "export_scene.fo2_track_cbd2_gen"
    bl_label = "Export FlatOut 2 track_cdb2.gen"

    # gets set by the file select window - internal Blender Magic or whatever.
    filepath: bpy.props.StringProperty(
        name="File Path", description="File path used for exporting the track_cdb2.gen file", maxlen=1024, default="")

    def execute(self, context):
        try:
            export_file(report=self.report, path=self.properties.filepath)
        except ExportError as e:
            self.report({'ERROR'}, e.message)
        return {'FINISHED'}

    def invoke(self, context, event):
        # sets self.properties.filename and runs self.execute()
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}


def export_menu_func(self, context):
    self.layout.operator(ExportOperator.bl_idname,
                         text="FlatOut 2 track_cdb2.gen")


def register():
    bpy.utils.register_class(ExportOperator)
    bpy.types.TOPBAR_MT_file_export.append(export_menu_func)


def unregister():
    bpy.types.TOPBAR_MT_file_export.remove(export_menu_func)
    bpy.utils.unregister_class(ExportOperator)
