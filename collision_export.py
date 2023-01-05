from .mod_reload import reload_modules
reload_modules(locals(), __package__, ["cdb2", "config"], [".geometry"])  # nopep8

from typing import Callable, Iterable, List, Optional, Set, Union, cast
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
    return Vector([e/MAX_ABS_COORDINATE for e in max_abs_bounds])


def export_file(report: report_func, path: str) -> None:
    print(f"export to \"{path}\"")

    collision_meshes = find_collision_meshes()
    print(f"found {len(collision_meshes)} collision meshes")

    axis_multipliers = calculate_axis_multipliers(
        report=report,
        collision_meshes=collision_meshes)
    print(f"{axis_multipliers=}")


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
