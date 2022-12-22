import bpy
import io
import mathutils
import os
import struct
from typing import List, Optional, Tuple

bl_info = {
    "name": "track_cdb2 format",
    "blender": (3, 4, 0),
    "category": "Import-Export",
    "description": "Support for FlatOut 2's track_cdb2.gen track collision format",
}

FILE_ID = b'!vq\x98'
VERSION = 0
HEADER_SIZE = 64
NODE_SIZE = 8
ONLY_EDGES = False
COLOR_SHALLOW = mathutils.Color((0xD0/0xFF, 0x00/0xFF, 0x70/0xFF))
COLOR_DEEP = mathutils.Color((0x00/0xFF, 0x32/0xFF, 0xA0/0xFF))

def color_lerp(f: float, c1: mathutils.Color, c2: mathutils.Color) -> mathutils.Color:
    """Linear interpolation between c1 (f=0) and c2 (f=1)"""
    return (c1 * (1.0-f)) + (c2 * f)

class AABB:
    def __init__(self, min: mathutils.Vector, max: mathutils.Vector) -> None:
        self.min = min
        self.max = max
    
    def copy(self) -> "AABB":
        return AABB(self.min.copy(), self.max.copy())

class Node:
    def __init__(self, data: bytes) -> None:
        assert(len(data) == NODE_SIZE)
        self._lo: int
        self._lo, = struct.unpack("I", data[:4])
        if self.is_leaf:
            self._hi: int
            self._hi, = struct.unpack("I", data[4:])
        else:
            self._max: int
            self._min: int
            self._max, self._min = struct.unpack("2h", data[4:])

    @property
    def axis(self) -> int:
        """0-3, where 3=leaf"""
        return (self._lo & ((1<<2)-1))
    @property
    def is_leaf(self) -> bool:
        return self.axis == 3
    @property
    def children(self) -> Tuple[int, int]:
        """Only for non-leafs: indices of child nodes"""
        assert(not self.is_leaf)
        offset = self._lo >> 8
        index = offset // NODE_SIZE
        return index, index+1
    @property
    def num_primitives(self) -> int:
        assert(self.is_leaf)
        return self._hi & ((1<<7)-1)
    @property
    def leaf_kind(self) -> int:
        assert(self.is_leaf)
        return (self._lo >> 6) & ((1<<3)-1)
    @property
    def max(self) -> int:
        """
        If this is not a leaf, the high dword is actually two words.
        This is the first of those words: the upper bound of this axis range.
        Do not call on leafs.
        """
        assert(not self.is_leaf)
        return self._max
    @property
    def min(self) -> int:
        """
        If this is not a leaf, the high dword is actually two words.
        This is the second of those words: the lower bound of this axis range.
        Do not call on leafs.
        """
        assert(not self.is_leaf)
        return self._min

def import_file(filename: str, enable_debug_visualization: bool = False):
    file_stats = os.stat(filename)
    file_size = file_stats.st_size
    with open(filename, "rb") as f:
        # Header
        file_id: bytes
        version: int
        file_id, version = struct.unpack("<4si", f.read(2*4))
        if file_id != FILE_ID:
            raise ValueError(f'bad magic header, want {FILE_ID}, got {file_id}')
        if version != VERSION:
            # note that I'm not 100% certain this is a version number,
            # but it's always 0 in my experience.
            raise ValueError(f'bad file version, want {VERSION}, got {version}')
        # scaled level AABB
        mins = struct.unpack("<3i", f.read(3*4))
        maxs = struct.unpack("<3i", f.read(3*4))
        # conversion from the scaled short-coordinates used in the aabb-tree
        # to the float game coordinates matching the visuals
        # this appears to be the inverse (1/x) of the axis_multipliers below
        # (though which of the two is the inverse is up for debate...)
        axis_multipliers = struct.unpack("<3f", f.read(3*4))
        # incoming game coordinates get multiplied by this,
        # it's the inverse of the axis_multipliers above.
        # These values are chosen such that the result is in [-32767, 32767]
        # (the range of a signed short*, which is what the checks in the tree use)
        # * technically, signed short also includes -32768,
        #   but canal1/a still only goes down to -32767.
        #   I appreciate the symmetry.
        #   Simplifies the calculation, if only slightly.
        #   Though I suspect the game would be able to handle -32768, too.
        #   But let's not generate anything the original tracks don't use,
        #   nobody ever tested that.
        inverse_axis_multipliers = struct.unpack("<3f", f.read(3*4))
        ofs_tree = HEADER_SIZE
        ofs_indices: int
        ofs_primitives: int
        ofs_indices, ofs_primitives = struct.unpack("<2I", f.read(2*4))
        # offsets are relative to end of the header
        ofs_indices += HEADER_SIZE
        ofs_primitives += HEADER_SIZE

        len_tree = ofs_indices - ofs_tree
        len_indices = ofs_primitives - ofs_indices
        len_primitives = file_size - ofs_primitives
        assert(f.tell() == ofs_tree)

        # Tree
        nodes: List[Node] = []
        while f.tell() + NODE_SIZE <= ofs_indices:
            node = Node(f.read(NODE_SIZE))
            if not node.is_leaf:
                assert (node.children[0] + 1) * NODE_SIZE <= len_tree, f"leaf {len(nodes)} first child {node.children[0]} out of range {len_tree//NODE_SIZE}"
                assert (node.children[1] + 1) * NODE_SIZE <= len_tree, f"leaf {len(nodes)} second child {node.children[1]} out of range {len_tree//NODE_SIZE}"
            nodes.append(node)

        print(f"""file info:
{mins=}
{maxs=}
{inverse_axis_multipliers=}
{axis_multipliers=}
{ofs_tree=}
{len_tree=}
{ofs_indices=}
{len_indices=}
{ofs_primitives=}
{len_primitives=}
{file_size=}
{len(nodes)=}

{enable_debug_visualization=}
""")
        if enable_debug_visualization:
            debug_collection = bpy.data.collections.new('debug_collection')
            bpy.context.scene.collection.children.link(debug_collection)

            def depth(node: Node) -> int:
                if node.is_leaf:
                    return 0
                c1, c2 = node.children
                return 1 + max(depth(nodes[c1]), depth(nodes[c2]))
            max_depth = depth(nodes[0])

            def generate_debug_visualization(bounds: AABB, node_index: int, depth: int, parent: Optional[bpy.types.Object]) -> None:
                if node_index < 0 or node_index >= len(nodes):
                    print(f"unexpected {node_index=}")
                    return

                node = nodes[node_index]
                if node.is_leaf:
                    # We could visualise the final bounds here,
                    # but it would be redundant?
                    # Better to show the children, once available.
                    return
                axis = node.axis
                inside_bounds = bounds.copy()
                inside_bounds.min[axis] = node.min
                inside_bounds.max[axis] = node.max
                axis2 = (axis+1) % 3
                axis3 = (axis2+1) % 3
                mins = [
                    inside_bounds.min[axis],
                    inside_bounds.min[axis2],
                    inside_bounds.min[axis3],
                ]
                maxs = [
                    inside_bounds.max[axis],
                    inside_bounds.max[axis2],
                    inside_bounds.max[axis3],
                ]
                verts = [
                    [mins[0], mins[1], mins[2]],
                    [mins[0], maxs[1], mins[2]],
                    [mins[0], maxs[1], maxs[2]],
                    [mins[0], mins[1], maxs[2]],
                    
                    [maxs[0], mins[1], mins[2]],
                    [maxs[0], maxs[1], mins[2]],
                    [maxs[0], maxs[1], maxs[2]],
                    [maxs[0], mins[1], maxs[2]],
                ]
                # FIXME: every level seems to have a different rotation
                # rotate vertices to correct axis
                # verts = [vert[axis:] + vert[:axis] for vert in verts]
                verts = [vert[-axis:] + vert[:-axis] for vert in verts]
                edges = [
                    (0, 1), (1, 2), (2, 3), (3, 0),
                    (4, 5), (5, 6), (6, 7), (7, 4),
                ] if ONLY_EDGES else []
                faces = [] if ONLY_EDGES else [
                    (0, 1, 2, 3),
                    (4, 5, 6, 7),
                ]
                color = color_lerp(depth/max_depth, COLOR_SHALLOW, COLOR_DEEP)

                inside_child, outside_child = node.children

                mesh_inside: bpy.types.Mesh = bpy.data.meshes.new(f"node{node_index}_inside")
                mesh_inside.from_pydata(verts, edges, faces)
                mesh_inside.update()
                obj_inside: bpy.types.Object = bpy.data.objects.new(f"node{node_index}_inside", mesh_inside)
                obj_inside.color = color[:]+(1.0,)
                debug_collection.objects.link(obj_inside)

                obj_outside: bpy.types.Object = bpy.data.objects.new(f"node{node_index}_outside", None)
                obj_outside.color = color[:]+(1.0,)
                debug_collection.objects.link(obj_outside)

                if parent is not None:
                    obj_inside.parent = parent
                    obj_outside.parent = parent
                print(f"generated {node_index=}")
                generate_debug_visualization(
                    bounds=inside_bounds,
                    node_index=inside_child,
                    depth=depth+1,
                    parent=obj_inside,
                )
                generate_debug_visualization(
                    bounds=bounds,
                    node_index=outside_child,
                    depth=depth+1,
                    parent=obj_outside,
                )

            generate_debug_visualization(
                bounds=AABB(mathutils.Vector(mins), mathutils.Vector(maxs)),
                node_index=0,
                depth=0,
                parent=None,
            )

        # apparently, the maximum referenced child is 8*num_nodes, which constitutes an off-by-one error?!? nevermind, the offset is excluding the header.

        # Indices
        assert(f.tell() == ofs_indices)

        # Primitives

class ImportOperator(bpy.types.Operator):
    bl_idname = "import_scene.fo2_track_cbd2_gen"
    bl_label = "Import FlatOut 2 track_cdb2.gen"

    #gets set by the file select window - internal Blender Magic or whatever.
    filepath: bpy.props.StringProperty(name="File Path", description="File path used for importing the track_cdb2.gen file", maxlen= 1024, default="")

    enable_debug_visualization: bpy.props.BoolProperty(name="Debug Visualization", description="Imports the collision tree. The tree is automatically rebuilt on export, this is only for debugging.", default=False)

    def execute(self, context):
        self.ImportStart()
        return {'FINISHED'}

    def invoke(self, context, event):
        #sets self.properties.filename and runs self.execute()
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def ImportStart(self):
        import_file(self.properties.filepath, enable_debug_visualization=self.properties.enable_debug_visualization)
        #self.report( { 'ERROR' }, f'import of {self.properties.filepath} not yet implemented')

def import_menu_func(self, context):
    self.layout.operator(ImportOperator.bl_idname, text="FlatOut 2 track_cdb2.gen")

def register():
    bpy.utils.register_class(ImportOperator)
    bpy.types.TOPBAR_MT_file_import.append(import_menu_func)
def unregister():
    bpy.types.TOPBAR_MT_file_import.remove(import_menu_func)
    bpy.utils.unregister_class(ImportOperator)
