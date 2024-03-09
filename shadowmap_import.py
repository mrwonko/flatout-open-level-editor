import bpy

import struct


def import_shadowmap(
        filename: str,
        # TODO: extents? or how do we know where to place it?
):
    width = 512
    height = 512
    with open(filename, "rb") as f:
        data = struct.unpack(f"{width*height}B", f.read(width*height))
    img = bpy.data.images.new("shadowmap", width, height)
    stride = 4
    # scale from [0, 255] to [0.0, 1.0]
    img.pixels = [b for x in data for b in (x/255, x/255, x/255, 1.0)]
    img.update()
    # TODO: place in map
