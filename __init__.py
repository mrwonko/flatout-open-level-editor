import bpy
import io
import os
import struct
from typing import Set

bl_info = {
    "name": "track_cdb2 format",
    "blender": (3, 4, 0),
    "category": "Import-Export",
    "description": "Support for FlatOut 2's track_cdb2.gen track collision format",
}

FILE_ID = b'!vq\x98'
VERSION = 0
HEADER_SIZE = 64

def import_file(filename: str):
    file_stats = os.stat(filename)
    file_size = file_stats.st_size
    with open(filename, "rb") as f:
        # Header
        file_id: bytes
        version: int
        file_id, version = struct.unpack("4si", f.read(2*4))
        if file_id != FILE_ID:
            raise ValueError(f'bad magic header, want {FILE_ID}, got {file_id}')
        if version != VERSION:
            # note that I'm not 100% certain this is a version number,
            # but it's always 0 in my experience.
            raise ValueError(f'bad file version, want {VERSION}, got {version}')
        # scaled level AABB
        mins = struct.unpack("3i", f.read(3*4))
        maxs = struct.unpack("3i", f.read(3*4))
        # this appears to be the inverse (1/x) of the axis_multipliers below
        # (though which of the two is the inverse is up for debate...)
        inverse_axis_multipliers = struct.unpack("3f", f.read(3*4))
        # incoming coordinates get multiplied by this
        # the multipliers appear to be chosen such that the result is in [-32767, 32767]
        # (the range of a signed short, which is presumably what the checks in the tree use)
        axis_multipliers = struct.unpack("3f", f.read(3*4))
        ofs_tree = HEADER_SIZE
        ofs_indices: int
        ofs_primitives: int
        ofs_indices, ofs_primitives = struct.unpack("2I", f.read(2*4))
        len_tree = ofs_indices - ofs_tree
        len_indices = ofs_primitives - ofs_indices
        len_primitives = file_size - ofs_primitives
        assert(f.tell() == HEADER_SIZE)

        # Tree
        referenced_child_offsets: Set[int] = set()
        leaf_child_offsets: Set[int] = set()
        num_nodes = 0
        while f.tell() < ofs_indices:
            flags_and_child: int
            data1: int
            data2: int
            flags_and_child, data1, data2 = struct.unpack("I2h", f.read(4+2*2))
            flags = flags_and_child & 0x000000FF
            axis = flags & 0b00000011
            child_offset = flags_and_child >> 8
            if axis == 3:
                # this is a leaf
                leaf_child_offsets.add(child_offset)
            else:
                # this is a check along an axis
                if child_offset in referenced_child_offsets:
                    # apparently does not happen (and why would it)
                    print(f"child @ {child_offset} referenced repeatedly")
                referenced_child_offsets.add(child_offset)
            num_nodes += 1

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
{num_nodes=}
referenced child offsets in [{min(referenced_child_offsets, default=None)},{max(referenced_child_offsets, default=None)}]
referenced leaf offsets in [{min(leaf_child_offsets, default=None)},{max(leaf_child_offsets, default=None)}]
""")
        # apparently, the maximum referenced child is 8*num_nodes, which constitutes an off-by-one error?!?

        # Indices

        # Primitives

class ImportOperator(bpy.types.Operator):
    bl_idname = "import_scene.fo2_track_cbd2_gen"
    bl_label = "Import FlatOut 2 track_cdb2.gen"

    #gets set by the file select window - internal Blender Magic or whatever.
    filepath: bpy.props.StringProperty(name="File Path", description="File path used for importing the track_cdb2.gen file", maxlen= 1024, default="")

    def execute(self, context):
        self.ImportStart()
        return {'FINISHED'}

    def invoke(self, context, event):
        #sets self.properties.filename and runs self.execute()
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}

    def ImportStart(self):
        import_file(self.properties.filepath)
        #self.report( { 'ERROR' }, f'import of {self.properties.filepath} not yet implemented')

def import_menu_func(self, context):
    self.layout.operator(ImportOperator.bl_idname, text="FlatOut 2 track_cdb2.gen")

def register():
    bpy.utils.register_class(ImportOperator)
    bpy.types.TOPBAR_MT_file_import.append(import_menu_func)
def unregister():
    bpy.types.TOPBAR_MT_file_import.remove(import_menu_func)
    bpy.utils.unregister_class(ImportOperator)
