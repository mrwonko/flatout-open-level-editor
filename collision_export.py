from .mod_reload import reload_modules
reload_modules(locals(), __package__, ["cdb2", "config"], [".bitmath", ".collision_mesh", ".geometry"])  # nopep8

from .bitmath import ones
from .collision_mesh import PackedMaterial, Triangle
from dataclasses import dataclass
from typing import Callable, Dict, Iterable, List, NewType, Optional, Set, Tuple, Union, cast
from .geometry import AABB, vector_abs, vector_max, vector_min
from . import cdb2, config
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


class TriangleEncoder:
    def __init__(self, vertex_encoder: VertexEncoder) -> None:
        self._vertex_encoder = vertex_encoder
        self._tris: List[Triangle] = []

    def append(self, verts: Tuple[Vector, Vector, Vector], flags: PackedMaterial) -> bool:
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
        self._tris.append(Triangle(flags=flags, vert_indices=vert_indices))
        return True

    @property
    def triangles(self) -> List[Triangle]:
        return self._tris


@dataclass
class DegenerateTriangle:
    object: bpy.types.Object
    polygon_index: int


def encode_flags(surface: int, flags: Iterable[bool]) -> PackedMaterial:
    packed_flags = 0
    for i, f in enumerate(flags):
        if f:
            packed_flags |= 1 << i
    loflags = packed_flags & ones(6)
    hiflags = (packed_flags >> 6)
    return PackedMaterial((surface - 1) | (loflags << 8) | (hiflags << 8+8))


def encode_triangles(axis_multipliers: Vector, collision_meshes: Iterable[bpy.types.Object]) -> Tuple[List[vec3_scaled], List[Triangle], List[DegenerateTriangle]]:
    vertex_encoder = VertexEncoder(axis_multipliers=axis_multipliers)
    triangle_encoder = TriangleEncoder(vertex_encoder=vertex_encoder)
    degenerates: List[DegenerateTriangle] = []
    for obj in collision_meshes:
        mesh = cast(bpy.types.Mesh, obj.data)
        for tri in mesh.loop_triangles:
            mat: bpy.types.Material = mesh.materials[tri.material_index]
            # convert back from 1-based surfaces.bed index to 0-based
            ok = triangle_encoder.append(
                verts=(obj.matrix_world @
                       mesh.vertices[idx].co for idx in tri.vertices),
                flags=encode_flags(
                    surface=mat.fo2.collision_surface, flags=mat.fo2.collision_flags),
            )
            if not ok:
                degenerates.append(DegenerateTriangle(
                    object=obj, polygon_index=tri.polygon_index))
    return vertex_encoder.vertices, triangle_encoder.triangles, degenerates


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
