import bpy
import io
from dataclasses import dataclass
import mathutils
import os
import struct
from typing import Dict, List, Optional, Set, Tuple, TypeVar, Union

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
TEST_BITMASK = True
VERIFY_REACHABILITY = True
CHECK_BOUNDS = True
COLOR_SHALLOW = mathutils.Color((0xD0/0xFF, 0x00/0xFF, 0x70/0xFF))
COLOR_DEEP = mathutils.Color((0x00/0xFF, 0x32/0xFF, 0xA0/0xFF))

@dataclass
class Header:
    # magic FILE_ID bytes
    file_id: bytes
    version: int
    # bounding box around the level, scaled by axis_multipliers
    mins: Tuple[int, int, int]
    maxs: Tuple[int, int, int]
    # conversion from the scaled short-coordinates used in the aabb-tree
    # to the float game coordinates matching the visuals
    # this appears to be the inverse (1/x) of the multipliers below
    axis_multipliers: Tuple[float, float, float]
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
    inverse_axis_multipliers: Tuple[float, float, float]
    # offset in bytes after the end of the header to the packed triangle buffer.
    # see ofs_triangles for absolute offset.
    relative_ofs_triangles: int
    # offset in bytes after the end of the header to the vertex coordinates.
    # see ofs_vertices for absolute offset.
    relative_ofs_vertices: int
    # not part of the header itself, but related to the offsets above,
    # so they're stored together.
    file_size: int

    @staticmethod
    def from_reader(f: io.BufferedReader, file_size: int) -> "Header":
        """
        Parses the header from the given buffer.
        Raises a ValueError if the data does not look like a track_cdb2.gen header,
        or a struct.error if there is insufficient data in the reader.
        Afterwards, the reader will be positioned after the header,
        i.e. at the start of the AABB tree data.
        """
        assert f.tell() == 0, f"expected header at start of file, got offset {f.tell()}"
        res = Header(
            file_id=struct.unpack("<4s", f.read(4))[0],
            version=struct.unpack("<i", f.read(4))[0],
            mins=struct.unpack("<3i", f.read(3*4)),
            maxs=struct.unpack("<3i", f.read(3*4)),
            axis_multipliers=struct.unpack("<3f", f.read(3*4)),
            inverse_axis_multipliers=struct.unpack("<3f", f.read(3*4)),
            relative_ofs_triangles=struct.unpack("<I", f.read(4))[0],
            relative_ofs_vertices=struct.unpack("<I", f.read(4))[0],
            file_size=file_size,
        )
        assert f.tell() == HEADER_SIZE, f"should have consumed {HEADER_SIZE} bytes after header, not {f.tell()}"
        if res.file_id != FILE_ID:
            raise ValueError(f'bad magic header, want {FILE_ID}, got {res.file_id}')
        if res.version != VERSION:
            # note that I'm not 100% certain this is a version number,
            # but it's always 0 in my experience.
            raise ValueError(f'bad file version, want {VERSION}, got {res.version}')
        # if this ever happens, len_triangles and len_vertices need to be adjusted.
        assert res.relative_ofs_triangles <= res.relative_ofs_vertices, "this importer assumes the triangle data comes before the vertex data and needs to be adjusted to support this file"
        return res

    @property
    def ofs_tree(self) -> int:
        """Offset in the reader where the AABB tree data starts"""
        return HEADER_SIZE
    @property
    def ofs_triangles(self) -> int:
        """Offset in the reader where the packed triangle data starts."""
        return self.relative_ofs_triangles + HEADER_SIZE
    @property
    def ofs_vertices(self) -> int:
        """Offset in the reader where the vertex coordinate data starts."""
        return self.relative_ofs_vertices + HEADER_SIZE
    @property
    def len_tree(self) -> int:
        """Size of the AABB tree data in bytes."""
        # assumes triangles come directly after the tree
        return self.ofs_triangles - self.ofs_tree
    @property
    def len_triangles(self) -> int:
        """Size of the triangle data in bytes."""
        # assumes vertices come directly after the triangles
        return self.ofs_vertices - self.ofs_triangles
    @property
    def len_vertices(self) -> int:
        """Size of the vertex data in bytes."""
        # assumes vertices continue until the end of the file
        return self.file_size - self.ofs_vertices

def color_lerp(f: float, c1: mathutils.Color, c2: mathutils.Color) -> mathutils.Color:
    """Linear interpolation between c1 (f=0) and c2 (f=1)"""
    return (c1 * (1.0-f)) + (c2 * f)

T = TypeVar('T')

def y_up_to_z_up(vert: List[T]) -> List[T]:
    """
    convert from file format coordinates (Y-axis is up)
    to Blender coordinates (Z-axis is up)
    """
    return [
        vert[0],
        vert[2],
        -vert[1],
    ]

def scale_to_blender(vert: Union[Tuple[int, int, int], List[int]], multipliers: List[float]) -> List[float]:
    """
    convert from file format AABB scale (int16) to Blender coordinates
    """
    return [e*m for e, m in zip(vert, multipliers)]

@dataclass
class AABB:
    min: mathutils.Vector
    max: mathutils.Vector

    def copy(self) -> "AABB":
        return AABB(self.min.copy(), self.max.copy())

def ones(n: int) -> int:
    """Returns a bitmask where the lowest n bits are 1."""
    return (1<<n)-1

class Node:
    def __init__(self, data: bytes, index: int, header: Header) -> None:
        assert len(data) == NODE_SIZE, f'expected {NODE_SIZE} bytes, not {len(data)}'
        AXIS_BITS = 2
        # I'm not entirely sure if the nomenclature is correct here,
        # but I'm calling the first dword lo(w) and the second one hi(gh)
        lo: int
        lo, = struct.unpack("<I", data[:4])
        self.axis = lo & ones(AXIS_BITS)
        lo >>= AXIS_BITS
        # if we generated a debug visualisation, this is the node's parent.
        # attach collision geometry to the same parent.
        self.debug_parent: Optional[bpy.types.Object] = None
        if self.is_leaf:
            # 23b triangle start, 3b kind, 4b mask, 2b axis (=3)
            MASK_BITS = 4
            KIND_BITS = 3
            # 19b vertex offset, 6b flags, 7b triangle count
            COUNT_BITS = 7
            FLAGS_BITS = 6

            self.bitmask = lo & ones(MASK_BITS)
            lo >>= MASK_BITS
            self._kind = lo & ones(KIND_BITS)
            lo >>= KIND_BITS
            self._triangle_offset = lo
            hi: int
            hi, = struct.unpack("<I", data[4:])
            self._num_triangles = hi & ones(COUNT_BITS)
            hi >>= COUNT_BITS
            self._flags = hi & ones(FLAGS_BITS)
            hi >>= FLAGS_BITS
            self._vert_offset = hi
            self.triangles: Optional[List[Triangle]] = None
        else:
            # 24b child offset, 6b mask, 2b axis
            MASK_BITS = 6
            # 16b max, 16b min

            # I suspect that in practice, this is still <= 4 bit,
            # like for leafs, but the engine can also handle more
            self.bitmask = lo & ones(MASK_BITS)
            lo >>= MASK_BITS
            child0_offset = lo
            assert child0_offset % NODE_SIZE == 0, f'unexpected node offset {child0_offset} is not a multiple of {NODE_SIZE}'
            self._child0_index = child0_offset // NODE_SIZE
            self._max: int
            self._min: int
            self._max, self._min = struct.unpack("<2h", data[4:])
            assert (self.children[0] + 1) * NODE_SIZE <= header.len_tree, f"leaf {index} first child {self.children[0]} out of range {header.len_tree//NODE_SIZE}"
            assert (self.children[1] + 1) * NODE_SIZE <= header.len_tree, f"leaf {index} second child {self.children[1]} out of range {header.len_tree//NODE_SIZE}"

    @property
    def is_leaf(self) -> bool:
        """Axis 0-2 are inner nodes, axis 3 marks leafs."""
        return self.axis == 3
    @property
    def children(self) -> Tuple[int, int]:
        """Only for non-leafs: indices of child nodes"""
        assert not self.is_leaf, 'leafs have no children'
        return self._child0_index, self._child0_index+1
    @property
    def num_triangles(self) -> int:
        """Only for leafs: number of triangles contained"""
        assert self.is_leaf, 'only leafs have triangles'
        return self._num_triangles
    @property
    def triangle_offset(self) -> int:
        assert self.is_leaf, 'only leafs have triangles'
        return self._triangle_offset
    @property
    def leaf_kind(self) -> int:
        assert self.is_leaf, 'only leafs have kinds'
        return self._kind
    @property
    def leaf_flags(self) -> int:
        """Only for leafs: the 6 flag bits."""
        assert self.is_leaf, 'only leafs have flags'
        return self._flags
    @property
    def vert_offset(self) -> int:
        '''
        Only for leafs: fixed offset into the vertex buffer.
        Depending on the leaf_kind, this is only applied to some vertices.
        '''
        assert self.is_leaf, 'only leafs have vertex offset'
        return self._vert_offset
    @property
    def max(self) -> int:
        """
        If this is not a leaf, the high dword is actually two words.
        This is the first of those words: the upper bound of this axis range.
        Do not call on leafs.
        """
        assert not self.is_leaf, 'leafs have no maximum'
        return self._max
    @property
    def min(self) -> int:
        """
        If this is not a leaf, the high dword is actually two words.
        This is the second of those words: the lower bound of this axis range.
        Do not call on leafs.
        """
        assert not self.is_leaf, 'leafs have no minimum'
        return self._min

@dataclass
class Triangle:
    # the low 6 bit are the node flags, the rest are triangle-specific
    flags: int
    vert_indices: Tuple[int, int, int]

    def check_bounds(self, node_index: int, len_vert_coords: int) -> None:
        for axis, idx in enumerate(self.vert_indices):
            assert idx+2 < len_vert_coords, f"node {node_index} axis {axis} vertex index {idx} out of range (max {len_vert_coords-2})"

def test_bitmask(nodes: List[Node]) -> None:
    """
    The assumption is that the bitmask is inherited upwards:
    a node knows whether a certain type of triangle is inside.
    This check tests that assumption.
    """
    def check_tree_bitmasks(root_index: int, expected: int) -> None:
        root = nodes[root_index]
        assert root.bitmask & expected == root.bitmask, f"node {root_index} bitmask {root.bitmask:b} does not fit expected pattern {expected:b}"
        if root.is_leaf:
            return
        check_tree_bitmasks(root.children[0], root.bitmask)
        check_tree_bitmasks(root.children[1], root.bitmask)
    check_tree_bitmasks(0, (1<<32)-1)

def verify_reachability(nodes: List[Node]) -> None:
    """
    Verifies the assumption that the AABB tree data contains no orphans,
    i.e. that by traversing from the root, every node is visited.
    Raises an AssertionError if the assumption is wrong.
    """
    reachable = [False] * len(nodes)
    def check_reachability(root_index: int) -> None:
        reachable[root_index] = True
        root = nodes[root_index]
        if root.is_leaf:
            return
        check_reachability(root.children[0])
        check_reachability(root.children[1])
    check_reachability(0)
    unreachable = [i for i, r in enumerate(reachable) if not r]
    assert len(unreachable) == 0, 'found {len(unreachable)} orphan node(s), first ten: {unreachable[:10]}'

def check_bounds(nodes: List[Node], len_triangle_data: int, len_vertex_coords: int):
    """
    Verifies that all the indices/offsets referenced in the aabb tree are within bounds.
    Only checks the start of the referenced range.
    Without these optional checks, we might get slightly less helpful errors later on.
    """
    for node_index, node in enumerate(nodes):
        if node.is_leaf:
            assert node.triangle_offset < len_triangle_data, f"node {node_index} triangle_offset {node.triangle_offset} is out of range [0, {len_triangle_data}]"
            assert node.vert_offset < len_vertex_coords, f"node {node_index} vert_offset {node.vert_offset} is out of range [0, {len_vertex_coords}]"
        else:
            assert node.children[0] >= 0, f"node {node_index} inner child index {node.children[0]} is negative"
            assert node.children[1] < len(nodes), f"node {node_index} outer child index {node.children[1]} is above {len(nodes)}"

def generate_debug_visualisation(collection: bpy.types.Collection, nodes: List[Node], header: Header) -> None:
    """
    Visualises the AABB collision tree using a hierarchy of mesh objects.
    As a side-effect, Node.debug_parent may be set on the incoming nodes.

    FIXME: the result is wrong somehow. The AABBs don't _quite_ match the tris within.
    """
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
        node.debug_parent = parent
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
        # rotate vertices to correct axis
        verts = map(lambda vert: vert[-axis:] + vert[:-axis], verts)
        # apply scale & adjust axis
        verts = [
            y_up_to_z_up(scale_to_blender(v, header.axis_multipliers))
            for v in verts
        ]
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
        obj_inside.display_type = "WIRE"
        obj_inside.color = color[:]+(1.0,)
        collection.objects.link(obj_inside)

        obj_outside: bpy.types.Object = bpy.data.objects.new(f"node{node_index}_outside", None)
        obj_outside.display_type = "WIRE"
        obj_outside.color = color[:]+(1.0,)
        collection.objects.link(obj_outside)

        if parent is not None:
            obj_inside.parent = parent
            obj_outside.parent = parent
        # print(f"generated {node_index=}")
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
        bounds=AABB(mathutils.Vector(header.mins), mathutils.Vector(header.maxs)),
        node_index=0,
        depth=0,
        parent=None,
    )

class VertexMapping:
    def __init__(self, header: Header, vertex_coords: List[int]) -> None:
        self.header = header
        self.vertex_coords = vertex_coords
        self.mapped_verts: List[List[float]] = []
        self.index_mapping: Dict[int, int] = {}

    def lookup(self, idx: int) -> int:
        if idx in self.index_mapping:
            return self.index_mapping[idx]
        assert idx < len(self.vertex_coords), f"vertex index {idx} out of range, there are {len(self.vertex_coords)} vertices totalling {self.header.len_vertices} bytes"
        mapping = len(self.mapped_verts)
        # TODO simplify the vertex handling
        # it's confusing that we treat vertices as individual scalars up to here
        self.mapped_verts.append(
            y_up_to_z_up(
                scale_to_blender(
                    (
                        self.vertex_coords[idx],
                        self.vertex_coords[idx+1],
                        self.vertex_coords[idx+2],
                    ),
                    self.header.axis_multipliers,
                ),
            ),
        )
        self.index_mapping[idx] = mapping
        return mapping

def import_file(filename: str, enable_debug_visualization: bool = False):
    file_stats = os.stat(filename)
    file_size = file_stats.st_size
    with open(filename, "rb") as f:
        # Header

        header = Header.from_reader(f, file_size)

        # Tree

        nodes: List[Node] = []
        # Since the bitmask appears to vary in length between leafs and non-leafs,
        # this variable keeps track of what bits actually get used in practice.
        # So far (in canal1/a) it looks like only the 4 low bits get used.
        seen_bitmask: int = 0
        # Count how often each kind of leaf encoding is used.
        # This was primarily used to decide which one to implement first.
        kind_counts: Dict[int, int] = {}
        while f.tell() + NODE_SIZE <= header.ofs_triangles:
            node = Node(f.read(NODE_SIZE), len(nodes), header)
            # collect some debugging stats
            if node.is_leaf:
                kind_counts[node.leaf_kind] = kind_counts.get(node.leaf_kind, 0) + 1
            seen_bitmask |= node.bitmask
            nodes.append(node)

        if TEST_BITMASK:
            test_bitmask(nodes)

        if VERIFY_REACHABILITY:
            verify_reachability(nodes)

        collision_collection = bpy.data.collections.new('collision')
        bpy.context.scene.collection.children.link(collision_collection)
        if enable_debug_visualization:
            generate_debug_visualisation(
                collection=collision_collection,
                nodes=nodes,
                header=header,
            )

        # Triangles

        # This chunk of data contains per-triangle flags
        # and the indices of the relevant vertices
        # packed into individual bits in various ways
        assert f.tell() == header.ofs_triangles
        triangle_data = f.read(header.len_triangles)

        # Vertex coordinates

        assert f.tell() == header.ofs_vertices
        # The vertex data consists of 16 bit integers,
        # scaled using the axis_multipliers like the tree nodes.
        # It should be read in 3-tuples for x, y and z,
        # but apparently there can be gaps between these tuples,
        # so we can't read triplets here yet.
        vertex_coords = [
            struct.unpack("<h", f.read(2))[0]
            for i in range(0, header.len_vertices, 2)
        ]


        if CHECK_BOUNDS:
            check_bounds(
                nodes=nodes,
                len_triangle_data=len(triangle_data),
                len_vertex_coords=len(vertex_coords),
            )

        # at this point, we're done reading from the file

    # now that we have all necessary data in memory, we can build the geometry
    if not enable_debug_visualization:
        # usually, we combine everything into a single mesh,
        # because Blender doesn't like having lots of objects,
        # and because there is no useful meaning in those objects anyway
        # - except when we want to visualise the AABB tree,
        # then we want one object per node.
        vertex_mapping = VertexMapping(header=header, vertex_coords=vertex_coords)
        all_mapped_tris: List[List[int]] = []
    all_flags: Set[int] = set()
    for (node_index, node) in enumerate(nodes):
        if not node.is_leaf:
            continue
        triangles: List[Triangle] = []
        vert_offset = node.vert_offset
        iter = node.triangle_offset

        if node.leaf_kind == 0:
            # until I can figure out why I get out-of-bounds vertex indices here, I'm skipping the first triangle
            if False:
                triangles.append(Triangle(
                    flags=node.leaf_flags | (triangle_data[iter] & ones(6)) << 8 | (triangle_data[iter] >> 6) << 8+8,
                    vert_indices=(
                        vert_offset,
                        # FIXME this is out of bounds?!
                        triangle_data[iter+1] | triangle_data[iter+2] << 8 | (triangle_data[iter+3] & ones(3)) << 8+8,
                        triangle_data[iter+3] >> 3 | triangle_data[iter+4] << 5 | triangle_data[iter+5] << 5+8,
                    ),
                ))
                triangles[-1].check_bounds(node_index, len(vertex_coords))
            iter += 6
            # 0th triangle (above) is unconditional, start loop at 1st
            for i in range(1, node.num_triangles):
                triangles.append(Triangle(
                    flags=(triangle_data[iter+1] & ones(6)) | (triangle_data[iter] & ones(6)) << 8 | (triangle_data[iter] >> 6) << 8+8,
                    vert_indices=(
                        triangle_data[iter+1] >> 7 | triangle_data[iter+2] << 1 | triangle_data[iter+3] << 1+8 | (triangle_data[iter+4] & ones(2)) << 1+8+8,
                        triangle_data[iter+4] >> 2 | triangle_data[iter+5] << 6 | (triangle_data[iter+6] & ones(5)) << 6+8,
                        triangle_data[iter+6] >> 5 | triangle_data[iter+7] << 3 | triangle_data[iter+8] << 3+8,
                    ),
                ))
                triangles[-1].check_bounds(node_index, len(vertex_coords))
                iter += 9
        elif node.leaf_kind == 2:
            for i in range(node.num_triangles):
                triangles.append(Triangle(
                    flags=(triangle_data[iter+1] & ones(6)) | (triangle_data[iter] & ones(6)) << 8 | (triangle_data[iter] >> 6) << 8+8,
                    vert_indices=(
                        # because we only use a single byte of triangle data,
                        # we can only reference a range of 256 consecutive vertices
                        vert_offset + triangle_data[iter+2],
                        vert_offset + triangle_data[iter+3],
                        vert_offset + triangle_data[iter+4],
                    ),
                ))
                triangles[-1].check_bounds(node_index, len(vertex_coords))
                iter += 5
        # TODO: other kinds
        if len(triangles) > 0:
            # Blender uses per-object vertex buffers,
            # so we copy the relevant vertices to our own buffer
            if enable_debug_visualization:
                # to visualise the tree, use a separate object attached
                vertex_mapping = VertexMapping(header=header, vertex_coords=vertex_coords)

            mapped_tris = [
                [vertex_mapping.lookup(idx) for idx in tri.vert_indices]
                for tri in triangles
            ]
            all_flags.update(map(lambda t: t.flags, triangles))

            if enable_debug_visualization:
                mesh: bpy.types.Mesh = bpy.data.meshes.new(f"node{node_index}")
                mesh.from_pydata(
                    vertex_mapping.mapped_verts,
                    (),
                    mapped_tris,
                )
                mesh.update()
                obj: bpy.types.Object = bpy.data.objects.new(f"node{node_index}", mesh)
                collision_collection.objects.link(obj)
                if node.debug_parent is not None:
                    obj.parent = node.debug_parent
            else:
                all_mapped_tris.extend(mapped_tris)
    if not enable_debug_visualization:
        mesh: bpy.types.Mesh = bpy.data.meshes.new(f"collision")
        mesh.from_pydata(
            vertex_mapping.mapped_verts,
            (),
            all_mapped_tris,
        )
        mesh.update()
        obj: bpy.types.Object = bpy.data.objects.new(f"collision", mesh)
        collision_collection.objects.link(obj)

    # debug print for me, the developer :)
    print(f"""file info:
{header=}
{len(nodes)=}
seen_bitmask={seen_bitmask:b}
{kind_counts=}
{all_flags=} ({len(all_flags)} total)

{enable_debug_visualization=}
""")

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
