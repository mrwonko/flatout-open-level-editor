from .mod_reload import reload_modules
reload_modules(locals(), __package__, ["cdb2", "config"], [".geometry", ".bitmath", ".collision_mesh", ".shadowmap_import"])  # nopep8

import bpy
import io
from dataclasses import dataclass
from enum import Enum
import mathutils
import os
import struct
from typing import Dict, List, Optional, Set, Tuple, TypeVar, Union
from . import cdb2, config
from .geometry import AABB
from .bitmath import ones
from .collision_mesh import PackedMaterial, Triangle
from .shadowmap_import import import_shadowmap


@dataclass
class Header:
    # magic cdb2.FILE_ID bytes
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
    # References into this data use 23 bit offsets,
    # implicitly limiting the number of triangles to 8388608/3 = 2796202.
    relative_ofs_triangles: int
    # offset in bytes after the end of the header to the vertex coordinates.
    # see ofs_vertices for absolute offset.
    # References to the 3-tuples of 2-byte-elements herein use 19 bit indices,
    # implicitly limiting the number of vertices to 524288/3 = 174762.
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
        assert f.tell() == 0, \
            f"expected header at start of file, got offset {f.tell()}"
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
        assert f.tell() == cdb2.HEADER_SIZE, \
            f"should have consumed {cdb2.HEADER_SIZE} bytes after header, not {f.tell()}"
        if res.file_id != cdb2.FILE_ID:
            raise ValueError(
                f'bad magic header, want {cdb2.FILE_ID}, got {res.file_id}')
        if res.version != cdb2.VERSION:
            # note that I'm not 100% certain this is a version number,
            # but it's always 0 in my experience.
            raise ValueError(
                f'bad file version, want {cdb2.VERSION}, got {res.version}')
        # if this ever happens, len_triangles and len_vertices need to be adjusted.
        assert res.relative_ofs_triangles <= res.relative_ofs_vertices, \
            "this importer assumes the triangle data comes before the vertex data and needs to be adjusted to support this file"
        return res

    @property
    def ofs_tree(self) -> int:
        """Offset in the reader where the AABB tree data starts"""
        return cdb2.HEADER_SIZE

    @property
    def ofs_triangles(self) -> int:
        """Offset in the reader where the packed triangle data starts."""
        return self.relative_ofs_triangles + cdb2.HEADER_SIZE

    @property
    def ofs_vertices(self) -> int:
        """Offset in the reader where the vertex coordinate data starts."""
        return self.relative_ofs_vertices + cdb2.HEADER_SIZE

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
        vert[1],
    ]


def scale_to_blender(vert: Union[Tuple[int, int, int], List[int]], multipliers: List[float]) -> List[float]:
    """
    convert from file format AABB scale (int16) to Blender coordinates
    """
    return [e*m for e, m in zip(vert, multipliers)]


class Node:
    def __init__(self, data: bytes, index: int, header: Header) -> None:
        assert len(data) == cdb2.NODE_SIZE, \
            f'expected {cdb2.NODE_SIZE} bytes, not {len(data)}'
        # I'm not entirely sure if the nomenclature is correct here,
        # but I'm calling the first dword lo(w) and the second one hi(gh)
        lo: int
        lo, = struct.unpack("<I", data[:4])
        self.axis = lo & ones(cdb2.AXIS_BITS)
        lo >>= cdb2.AXIS_BITS
        # if we generated a debug visualisation, this is the node's parent.
        # attach collision geometry to the same parent.
        self.debug_parent: Optional[bpy.types.Object] = None
        if self.is_leaf:

            self.bitmask = lo & ones(cdb2.LEAF_MASK_BITS)
            lo >>= cdb2.LEAF_MASK_BITS
            self._kind = lo & ones(cdb2.LEAF_KIND_BITS)
            lo >>= cdb2.LEAF_KIND_BITS
            self._triangle_offset = lo  # 32-2-4-3 = 23 bit
            hi: int
            hi, = struct.unpack("<I", data[4:])
            self._num_triangles = hi & ones(cdb2.LEAF_TRIANGLE_COUNT_BITS)
            hi >>= cdb2.LEAF_TRIANGLE_COUNT_BITS
            self._flags = hi & ones(cdb2.LEAF_FLAGS_BITS)
            hi >>= cdb2.LEAF_FLAGS_BITS
            self._vert_offset = hi  # 19 bit
            self.triangles: Optional[List[Triangle]] = None
        else:

            # I suspect that in practice, this is still <= 4 bit,
            # like for leafs, but the engine can also handle more
            self.bitmask = lo & ones(cdb2.INNER_NODE_MASK_BITS)
            lo >>= cdb2.INNER_NODE_MASK_BITS
            child0_offset = lo
            assert child0_offset % cdb2.NODE_SIZE == 0, \
                f'unexpected node offset {child0_offset} is not a multiple of {cdb2.NODE_SIZE}'
            self._child0_index = child0_offset // cdb2.NODE_SIZE
            self._max: int
            self._min: int
            self._max, self._min = struct.unpack("<2h", data[4:])
            assert (self.children[0] + 1) * cdb2.NODE_SIZE <= header.len_tree, \
                f"leaf {index} first child {self.children[0]} out of range {header.len_tree//cdb2.NODE_SIZE}"
            assert (self.children[1] + 1) * cdb2.NODE_SIZE <= header.len_tree, \
                f"leaf {index} second child {self.children[1]} out of range {header.len_tree//cdb2.NODE_SIZE}"

    @property
    def is_leaf(self) -> bool:
        """Axis 0-2 are inner nodes, axis 3 marks leafs."""
        return self.axis == cdb2.LEAF_AXIS

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
        """
        Only for leafs: 23-bit offset into the packed triangle buffer.
        """
        assert self.is_leaf, 'only leafs have triangles'
        return self._triangle_offset

    @property
    def leaf_kind(self) -> int:
        """Only for leafs: Describes which of the 6 triangle encodings is used."""
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
        Only for leafs: 19-bit index into the vertex buffer.
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


def check_triangle_bounds(tri: Triangle, node: Node, node_index: int, len_vert_coords: int) -> None:
    for axis, idx in enumerate(tri.vert_indices):
        assert idx + 2 < len_vert_coords, \
            f"node {node_index} (kind {node.leaf_kind}) axis {axis} vertex index {idx} out of range (max {len_vert_coords-2})"


def test_bitmask(nodes: List[Node]) -> None:
    """
    The assumption is that the bitmask is inherited upwards:
    a node knows whether a certain type of triangle is inside.
    This check tests that assumption.
    """
    def check_tree_bitmasks(root_index: int, expected: int) -> None:
        root = nodes[root_index]
        assert root.bitmask & expected == root.bitmask, \
            f"node {root_index} bitmask {root.bitmask:b} does not fit expected pattern {expected:b}"
        if root.is_leaf:
            return
        check_tree_bitmasks(root.children[0], root.bitmask)
        check_tree_bitmasks(root.children[1], root.bitmask)
    check_tree_bitmasks(0, (1 << 32)-1)


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
    assert len(unreachable) == 0, \
        f'found {len(unreachable)} orphan node(s), first ten: {unreachable[:10]}'


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
            assert node.children[1] < len(nodes), \
                f"node {node_index} outer child index {node.children[1]} is above {len(nodes)}"


def generate_debug_visualisation(collection: bpy.types.Collection, nodes: List[Node], header: Header) -> None:
    """
    Visualises the AABB collision tree using a hierarchy of mesh objects.
    As a side-effect, Node.debug_parent may be set on the incoming nodes.
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
        outside_bounds = bounds.copy()
        outside_bounds.min[axis] = node.min
        inside_bounds.max[axis] = node.max

        def visualise_bounds(name: str, aabb: AABB, axis: int, color: mathutils.Color) -> bpy.types.Object:
            mins = aabb.min
            maxs = aabb.max
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
            verts = [
                y_up_to_z_up(scale_to_blender(v, header.axis_multipliers))
                for v in verts
            ]
            edges = []
            faces: List[Tuple[int, int, int, int]] = [
                (0, 1, 2, 3),
                (7, 6, 5, 4),
            ] if axis == 0 else [
                (0, 3, 7, 4),
                (1, 5, 6, 2),
            ] if axis == 1 else [
                (0, 4, 5, 1),
                (2, 6, 7, 3),
            ]
            mesh: bpy.types.Mesh = bpy.data.meshes.new(name)
            mesh.from_pydata(verts, edges, faces)
            mesh.update()
            obj: bpy.types.Object = bpy.data.objects.new(name, mesh)
            obj.display_type = "WIRE"
            obj.color = color[:]+(1.0,)
            collection.objects.link(obj)
            return obj
        color = color_lerp(
            depth/max_depth, config.COLOR_SHALLOW, config.COLOR_DEEP)
        obj_inside = visualise_bounds(
            name=f"node{node_index}_inside",
            aabb=inside_bounds,
            axis=axis,
            color=color,
        )
        obj_outside = visualise_bounds(
            name=f"node{node_index}_outside",
            aabb=outside_bounds,
            axis=axis,
            color=color,
        )
        if parent is not None:
            obj_inside.parent = parent
            obj_outside.parent = parent
        inside_child, outside_child = node.children
        generate_debug_visualization(
            bounds=inside_bounds,
            node_index=inside_child,
            depth=depth+1,
            parent=obj_inside,
        )
        generate_debug_visualization(
            bounds=outside_bounds,
            node_index=outside_child,
            depth=depth+1,
            parent=obj_outside,
        )

    generate_debug_visualization(
        bounds=AABB(mathutils.Vector(header.mins),
                    mathutils.Vector(header.maxs)),
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
        assert idx < len(self.vertex_coords), \
            f"vertex index {idx} out of range, there are {len(self.vertex_coords)} vertices totalling {self.header.len_vertices} bytes"
        mapping = len(self.mapped_verts)
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


class MaterialProperties(bpy.types.PropertyGroup):
    collision_surface: bpy.props.IntProperty(
        name="Surface",
        description="Reference into the Surfaces defined in global/dynamics/surfaces.bed",
        # Like Lua, we use 1-based indices.
        min=1,
        # In the file, we convert to 0-based indices, but here the maximum is 64, not 63.
        max=1 << 6,
        default=1,
    )
    collision_flags: bpy.props.BoolVectorProperty(
        name="Flags",
        # TODO: determine and document meaning
        description="Flags with unknown meaning. The first 6 seem to form a group, the final one defaults to 0 in some encodings.",
        size=6+2,
        # TODO: find good defaults
        default=[False]*(6+2),
        subtype="LAYER",
    )
    collision_bitmask: bpy.props.BoolVectorProperty(
        name="Bitmask",
        # TODO: determine and document meaning
        description="Flags with unknown meaning. Seem to be used for filtering on collision check.",
        size=cdb2.LEAF_MASK_BITS,
        default=[False]*cdb2.LEAF_MASK_BITS,
        subtype="LAYER",
    )


def material_properties_draw_func(self: bpy.types.Panel, context: bpy.types.Context):
    layout = self.layout
    ob = context.object
    box = layout.box()
    box.label(text="FlatOut 2 Collision")
    # FIXME: this check always fails
    if (fo2 := ob.active_material.fo2) == None:
        # TODO: show "add fo2 properties" button
        # box.operator()
        pass
    else:
        box.prop(ob.active_material.fo2, "collision_surface")
        box.prop(ob.active_material.fo2, "collision_flags")
        box.prop(ob.active_material.fo2, "collision_bitmask")


class MaterialColorKind(Enum):
    SURFACE_RAINBOW = 1
    SURFACE_FIXED = 2
    BITMASK_DERIVED = 3
    BITMASK_FIXED = 4
    FLAGS_1_TO_6 = 5
    FLAGS_1_TO_8 = 6
    FLAGS_HUE_VALUE = 7


def material_color(kind: MaterialColorKind, surface: int, bitmask: int, flags: int) -> Optional[mathutils.Color]:
    try:
        if kind == MaterialColorKind.SURFACE_RAINBOW:
            color = mathutils.Color()
            color.hsv = (surface / ones(6), 1, 1)
            return color
        elif kind == MaterialColorKind.SURFACE_FIXED:
            return config.SURFACE_COLORS[surface]
        elif kind == MaterialColorKind.BITMASK_DERIVED:
            color = mathutils.Color(
                ((bitmask >> 3) & 1, (bitmask >> 1) & 1, bitmask & 1))
            # use remaining bit for brightness, but from 50%-100% instead 0%-100%
            color.v = 0.5 + ((bitmask >> 2) & 1) / 2
            return color
        elif kind == MaterialColorKind.BITMASK_FIXED:
            return config.BITMASK_COLORS[bitmask]
        elif kind == MaterialColorKind.FLAGS_1_TO_6:
            color = mathutils.Color()
            color.hsv = (0, 0, (flags & ones(6)) / ones(6))
            return color
        elif kind == MaterialColorKind.FLAGS_1_TO_8:
            color = mathutils.Color()
            color.hsv = (0, 0, flags / ones(8))
            return color
        elif kind == MaterialColorKind.FLAGS_HUE_VALUE:
            color = mathutils.Color()
            color.hsv = ((flags >> 6) / ones(2), 1,
                         (flags & ones(6)) / ones(6))
            return color
        return None
    except KeyError:
        return mathutils.Color((0.3, 0.3, 0.3))


class MaterialManager:
    """
    Keeps track of created materials so we only create one material per flag-combination.
    """

    def __init__(self) -> None:
        self._materials: Dict[Tuple[PackedMaterial, int],
                              bpy.types.Material] = {}

    def get_or_create(self, kind: MaterialColorKind, packed_material: PackedMaterial, bitmask: int) -> bpy.types.Material:
        try:
            return self._materials[(packed_material, bitmask)]
        except KeyError:
            mat = self._create(kind, packed_material, bitmask)
            self._materials[(packed_material, bitmask)] = mat
            return mat

    def _create(self, kind: MaterialColorKind, packed_material: PackedMaterial, bitmask: int) -> bpy.types.Material:
        # we're using Lua-style 1-based indices here to match the indices in surfaces.bed
        surface = (packed_material & ones(6)) + 1
        loflags = (packed_material >> 8) & ones(6)
        hiflags = (packed_material >> 8+8) & ones(2)
        flags = hiflags << 6 | loflags
        # TODO find out what each flag means and decide how to display them.
        # For now, I group the flags like they are in the packed data.
        name = f"col_{surface}_{hiflags:>02b}_{loflags:>06b}_{bitmask:>04b}"
        # re-use existing material with matching name, if any
        if (mat := bpy.data.materials.get(name)) is not None:
            return mat
        mat = bpy.data.materials.new(name)
        # if available, assign a color
        if (col := material_color(kind=kind, surface=surface, bitmask=bitmask, flags=flags)) is not None:
            mat.diffuse_color = col[:]+(1.0,)
        # we use custom properties as defined below for FlatOut 2 specific data
        mat.fo2.collision_surface = surface
        mat.fo2.collision_flags = [flags & (1 << b) != 0 for b in range(8)]
        # TODO I might be better off mapping these to layers instead of materials
        mat.fo2.collision_bitmask = [
            bitmask & (1 << b) != 0 for b in range(cdb2.LEAF_MASK_BITS)]
        return mat

    @staticmethod
    def add_properties() -> None:
        bpy.utils.register_class(MaterialProperties)
        bpy.types.Material.fo2 = bpy.props.PointerProperty(
            type=MaterialProperties)
        bpy.types.MATERIAL_PT_custom_props.prepend(
            material_properties_draw_func)

    @ staticmethod
    def remove_properties() -> None:
        bpy.types.MATERIAL_PT_custom_props.remove(
            material_properties_draw_func)
        bpy.props.RemoveProperty(bpy.types.Material, attr="fo2")
        bpy.utils.unregister_class(MaterialProperties)


class MeshMaterialManager:
    """
    Keeps track of the materials used by a mesh.
    """

    def __init__(self, mesh: bpy.types.Mesh, material_manager: MaterialManager) -> None:
        self._mesh = mesh
        self._material_manager = material_manager
        self._materials: Dict[Tuple[PackedMaterial, int], int] = {}

    def fetch(self, kind: MaterialColorKind, packed_material: PackedMaterial, bitmask: int) -> int:
        """
        If the mesh already uses this material, returns its index.
        Otherwise, uses the MaterialManager to get or create the material,
        adds it to the mesh's materials, and returns its new index.
        """
        try:
            return self._materials[(packed_material, bitmask)]
        except KeyError:
            mat = self._material_manager.get_or_create(
                kind, packed_material, bitmask)
            assert mat is not None
            # I don't really understand how adding new materials in Blender works
            # Apparently the object material slots get created automatically
            # as materials get added to the object's mesh.
            idx = len(self._mesh.materials)
            self._materials[(packed_material, bitmask)] = idx
            self._mesh.materials.append(mat)
            return idx


def import_file(
        filename: str,
        material_color_kind: MaterialColorKind,
        enable_debug_visualization: bool = False,
        with_shadowmap: bool = False,
):
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
        while f.tell() + cdb2.NODE_SIZE <= header.ofs_triangles:
            node = Node(f.read(cdb2.NODE_SIZE), len(nodes), header)
            # collect some debugging stats
            if node.is_leaf:
                kind_counts[node.leaf_kind] = kind_counts.get(
                    node.leaf_kind, 0) + 1
            seen_bitmask |= node.bitmask
            nodes.append(node)

        if config.TEST_BITMASK:
            test_bitmask(nodes)

        if config.VERIFY_REACHABILITY:
            verify_reachability(nodes)

        collision_collection = bpy.data.collections.get('collision')
        if collision_collection is None:
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

        if config.CHECK_BOUNDS:
            check_bounds(
                nodes=nodes,
                len_triangle_data=len(triangle_data),
                len_vertex_coords=len(vertex_coords),
            )

        # at this point, we're done reading from the file

    # now that we have all necessary data in memory, we can build the geometry
    material_manager = MaterialManager()
    if not enable_debug_visualization:
        # usually, we combine everything into a single mesh,
        # because Blender doesn't like having lots of objects,
        # and because there is no useful meaning in those objects anyway
        # - except when we want to visualise the AABB tree,
        # then we want one object per node.
        vertex_mapping = VertexMapping(
            header=header,
            vertex_coords=vertex_coords,
        )
        all_mapped_tris: List[List[int]] = []
        all_material_indices: List[int] = []
        mesh: bpy.types.Mesh = bpy.data.meshes.new(f"collision")
        mesh_material_manager = MeshMaterialManager(mesh, material_manager)
    all_flags: Set[PackedMaterial] = set()
    for (node_index, node) in enumerate(nodes):
        if not node.is_leaf or node.num_triangles == 0:
            continue
        triangles: List[Triangle] = []
        # 19 bit
        vert_offset = node.vert_offset
        iter = node.triangle_offset

        def get(i: int = 0) -> int:
            """
            Iterator lookup with offset.
            This function name is admittedly overly generic,
            but the main concern here is brevity and simplicity.
            """
            return triangle_data[iter+i]

        if node.leaf_kind == 0:
            triangles.append(Triangle(
                # 6 + 6 + 2 bit
                flags=PackedMaterial(
                    node.leaf_flags | (get(0) & ones(6)) << 8 | (get(0) >> 6) << 8+8),
                vert_indices=(
                    # 19 bit
                    vert_offset,
                    # 19 bit
                    get(1) | get(2) << 8 | (get(3) & ones(3)) << 8+8,
                    # 21 bit (!?)
                    get(3) >> 3 | get(4) << 5 | get(5) << 5+8
                ),
            ))
            check_triangle_bounds(
                triangles[-1], node, node_index, len(vertex_coords))
            iter += 6
            # 0th triangle (above) is unconditional, start loop at 1st
            for i in range(1, node.num_triangles):
                triangles.append(Triangle(
                    # 6 + 6 + 2 bit
                    flags=PackedMaterial(
                        (get(1) & ones(6)) | (get(0) & ones(6)) << 8 | (get(0) >> 6) << 8+8),
                    vert_indices=(
                        # 19 bit
                        get(1) >> 7 | get(2) << 1 | get(3) << 1 + \
                        8 | (get(4) & ones(2)) << 1+8+8,
                        # 19 bit
                        get(4) >> 2 | get(5) << 6 | (get(6) & ones(5)) << 6+8,
                        # 19 bit
                        get(6) >> 5 | get(7) << 3 | get(8) << 3+8,
                    ),
                ))
                check_triangle_bounds(triangles[-1], node,
                                      node_index, len(vertex_coords))
                iter += 9
        elif node.leaf_kind == 1:
            triangles.append(Triangle(
                # 6 + 6 + 2 bit
                flags=PackedMaterial(
                    node.leaf_flags | (get(0) & ones(6)) << 8 | (get(0) >> 6) << 8+8),
                vert_indices=(
                    # 19 bit
                    vert_offset,
                    # 19 bit
                    get(1) | get(2) << 8 | (get(3) & ones(3)) << 8+8,
                    # 21 bit (!?)
                    get(3) >> 3 | get(4) << 5 | get(5) << 5+8
                ),
            ))
            check_triangle_bounds(
                triangles[-1], node, node_index, len(vertex_coords))
            iter += 6
            for i in range(1, node.num_triangles):
                triangles.append(Triangle(
                    # 6 + 6 + 1 bit
                    flags=PackedMaterial(
                        node.leaf_flags | (get(0) & ones(6)) << 8 | ((get(0) >> 6) & ones(1)) << 8+8),
                    vert_indices=(
                        # 19 bit
                        get(0) >> 7 | get(1) << 1 | get(2) << 1 + \
                        8 | (get(3) & ones(2)) << 1+8+8,
                        # 19 bit
                        get(3) >> 2 | get(4) << 6 | (get(5) & ones(5)) << 6+8,
                        # 19 bit
                        get(5) >> 5 | get(6) << 3 | get(7) << 3+8,
                    ),
                ))
                check_triangle_bounds(
                    triangles[-1], node,  node_index, len(vertex_coords))
                iter += 8
        elif node.leaf_kind == 2:
            for i in range(node.num_triangles):
                triangles.append(Triangle(
                    # 6 + 6 + 2 bit
                    flags=PackedMaterial(
                        (get(1) & ones(6)) | (get(0) & ones(6)) << 8 | (get(0) >> 6) << 8+8),
                    vert_indices=(
                        # because we only use a single byte of triangle data,
                        # we can only reference a range of 256 consecutive vertices
                        # 8 bit
                        vert_offset + get(2),
                        # 8 bit
                        vert_offset + get(3),
                        # 8 bit
                        vert_offset + get(4),
                    ),
                ))
                check_triangle_bounds(triangles[-1],
                                      node,
                                      node_index,
                                      len(vertex_coords))
                iter += 5
        elif node.leaf_kind == 3:
            for i in range(node.num_triangles):
                triangles.append(Triangle(
                    # 6 + 6 + 2 bit
                    flags=PackedMaterial(
                        node.leaf_flags | (get(0) & ones(6)) << 8 | (get(0) >> 6) << 8+8),
                    vert_indices=(
                        # 8 bit
                        vert_offset + get(1),
                        # 8 bit
                        vert_offset + get(2),
                        # 8 bit
                        vert_offset + get(3),
                    ),
                ))
                check_triangle_bounds(triangles[-1], node,
                                      node_index, len(vertex_coords))
                iter += 4
        elif node.leaf_kind == 4:
            for i in range(node.num_triangles):
                triangles.append(Triangle(
                    # 6 + 6 + 1 bit
                    flags=PackedMaterial(
                        node.leaf_flags | (get(0) & ones(6)) << 8 | ((get(0) >> 6) & ones(1)) << 8+8),
                    vert_indices=(
                        # 12 bit
                        vert_offset + \
                        (get(0) >> 7 | get(1) << 1 | (get(2) & ones(2)) << 1+8),
                        # 12 bit
                        vert_offset + (get(2) >> 2 | (get(3) & ones(5)) << 6),
                        # 11 bit
                        vert_offset + (get(3) >> 5 | get(4) << 3),
                    ),
                ))
                check_triangle_bounds(triangles[-1], node,
                                      node_index, len(vertex_coords))
                iter += 5
        elif node.leaf_kind == 5:
            for i in range(node.num_triangles):
                triangles.append(Triangle(
                    # 6 + 6 + 2 bit
                    flags=PackedMaterial(
                        node.leaf_flags | (get(0) & ones(6)) << 8 | (get(0) >> 6) << 8+8),
                    vert_indices=(
                        # 5 bit
                        vert_offset + (get(1) & ones(5)),
                        # 5 bit
                        vert_offset + (get(1) >> 5 | (get(2) & ones(2)) << 3),
                        # 6 bit
                        vert_offset + (get(2) >> 2),
                    ),
                ))
                check_triangle_bounds(triangles[-1],
                                      node,
                                      node_index,
                                      len(vertex_coords))
                iter += 3
        else:
            raise ValueError(
                f"invalid kind {node.leaf_kind} on leaf {node_index}")
        if len(triangles) > 0:
            # Blender uses per-object vertex buffers,
            # so we copy the relevant vertices to our own buffer
            if enable_debug_visualization:
                # to visualise the tree, use a separate object attached
                vertex_mapping = VertexMapping(
                    header=header,
                    vertex_coords=vertex_coords)
                mesh: bpy.types.Mesh = bpy.data.meshes.new(f"node{node_index}")
                mesh_material_manager = MeshMaterialManager(
                    mesh, material_manager)

            mapped_tris = [
                [vertex_mapping.lookup(idx) for idx in tri.vert_indices]
                for tri in triangles
            ]
            # flip triangles
            mapped_tris = [list(reversed(tri)) for tri in mapped_tris]

            # set up materials
            material_indices = [
                mesh_material_manager.fetch(material_color_kind, t.flags, node.bitmask) for t in triangles]
            all_flags.update(map(lambda t: t.flags, triangles))

            if enable_debug_visualization:
                mesh.from_pydata(
                    vertex_mapping.mapped_verts,
                    (),
                    mapped_tris,
                )
                mesh.update()
                for tri, mat in enumerate(material_indices):
                    mesh.polygons[tri].material_index = mat
                obj: bpy.types.Object = bpy.data.objects.new(
                    f"node{node_index}", mesh)
                collision_collection.objects.link(obj)
                if node.debug_parent is not None:
                    obj.parent = node.debug_parent
            else:
                all_mapped_tris.extend(mapped_tris)
                all_material_indices.extend(material_indices)
    if not enable_debug_visualization:
        mesh.from_pydata(
            vertex_mapping.mapped_verts,
            (),
            all_mapped_tris,
        )
        mesh.update()
        for tri, mat in enumerate(all_material_indices):
            mesh.polygons[tri].material_index = mat
        # TODO: I could create one face map for each flag, to more easily see faces with each flag
        obj: bpy.types.Object = bpy.data.objects.new(f"collision", mesh)
        collision_collection.objects.link(obj)

    # shadowmap import is done from here because we suspect that it is tied to the collision coordinate system,
    # in which case we'll need data from the header. But analysis of that is incomplete.
    if with_shadowmap:
        import_shadowmap(filename=os.path.join(
            os.path.dirname(filename),
            '..', 'lighting', 'shadowmap_w2.dat'))

    # debug print for me, the developer :)
    # possibly reference to global/dynamics/surfaces.bed? (1-49)
    material_byte0s: Set[int] = set()
    # probably bitflags, most values from 0-63 occur
    material_byte1s: Set[int] = set()
    for material in all_flags:
        material_byte0s.add(material & ones(8))
        material_byte1s.add((material >> 8) & ones(8))
    print(f"""file info:
{header=}
{len(nodes)=}
seen_bitmask={seen_bitmask:b}
{kind_counts=}
{all_flags=} ({len(all_flags)} total)
{sorted(material_byte0s)=}
{sorted(material_byte1s)=}

{enable_debug_visualization=}
""")


class ImportOperator(bpy.types.Operator):
    bl_idname = "import_scene.fo2_track_cbd2_gen"
    bl_label = "Import FlatOut 2 track_cdb2.gen"

    # gets set by the file select window - internal Blender Magic or whatever.
    filepath: bpy.props.StringProperty(
        name="File Path", description="File path used for importing the track_cdb2.gen file", maxlen=1024, default="")

    import_shadowmap: bpy.props.BoolProperty(
        name="Shadowmap", description="Imports the ../lighting/shadowmap_w2.dat file. There's an export bug that causes the car's brightness to be incorrect, maybe this can help understand it. Only loads the image, does not (yet) place it in the level", default=False)

    enable_debug_visualization: bpy.props.BoolProperty(
        name="Debug Visualization", description="Imports the collision tree. The tree is automatically rebuilt on export, this is only for debugging", default=False)

    material_color_kind: bpy.props.EnumProperty(
        name="Material Color",
        description="How should the color for the generated materials be chosen? This is purely visual, and irrelevant for the export",
        items=[
            (MaterialColorKind.SURFACE_RAINBOW.name, "Surface Rainbow",
             "Select hue based on surface index, which references surfaces.bed",
             MaterialColorKind.SURFACE_RAINBOW.value),
            (MaterialColorKind.SURFACE_FIXED.name, "Surface Hand Selection",
             "Use manually chosen colors by surface ID. Not all surface IDs supported", MaterialColorKind.SURFACE_FIXED.value),
            (MaterialColorKind.BITMASK_DERIVED.name, "Bitmask Derived",
             "Derive colors from the collision filter mask bits, which are used for filtering",
             MaterialColorKind.BITMASK_DERIVED.value),
            (MaterialColorKind.BITMASK_FIXED.name, "Bitmask Hand Selection",
             "Use hand-picked colors for certain the collision filter mask bits, which are used for filtering",
             MaterialColorKind.BITMASK_FIXED.value),
            (MaterialColorKind.FLAGS_1_TO_6.name, "Flags 1-6",
             "Select value based on integer interpretation of the lowest 6 flags", MaterialColorKind.FLAGS_1_TO_6.value),
            (MaterialColorKind.FLAGS_1_TO_8.name, "Flags 1-8",
             "Select value based on integer interpretation of the flags", MaterialColorKind.FLAGS_1_TO_8.value),
            (MaterialColorKind.FLAGS_HUE_VALUE.name, "Flags Hue+Value",
             "Select value based on integer interpretation of the lowest 6 flags, and hue based on the other 2",
             MaterialColorKind.FLAGS_HUE_VALUE.value),
        ],
        default=MaterialColorKind.SURFACE_RAINBOW.value,
    )

    def execute(self, context):
        import_file(
            self.properties.filepath,
            material_color_kind=MaterialColorKind[self.properties.material_color_kind],
            enable_debug_visualization=self.properties.enable_debug_visualization,
            with_shadowmap=self.properties.import_shadowmap,
        )
        return {'FINISHED'}

    def invoke(self, context, event):
        # sets self.properties.filename and runs self.execute()
        context.window_manager.fileselect_add(self)
        return {'RUNNING_MODAL'}


def import_menu_func(self, context):
    self.layout.operator(ImportOperator.bl_idname,
                         text="FlatOut 2 track_cdb2.gen")


def register():
    bpy.utils.register_class(ImportOperator)
    MaterialManager.add_properties()
    bpy.types.TOPBAR_MT_file_import.append(import_menu_func)


def unregister():
    bpy.types.TOPBAR_MT_file_import.remove(import_menu_func)
    MaterialManager.remove_properties()
    bpy.utils.unregister_class(ImportOperator)
