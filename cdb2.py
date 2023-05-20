FILE_ID = b'!vq\x98'
VERSION = 0
HEADER_SIZE = 64
NODE_SIZE = 8


# AABB tree node encoding
AXIS_BITS = 2
LEAF_AXIS = 3  # normal axes are 0-2, 3 means leaf

# AABB tree leafs
# 23b triangle start, 3b kind, 4b mask, 2b axis (=3)
LEAF_MASK_BITS = 4
LEAF_KIND_BITS = 3
LEAF_TRIANGLE_OFS_BITS = 32 - AXIS_BITS - LEAF_MASK_BITS - LEAF_KIND_BITS  # 23
# 19b vertex offset, 6b flags, 7b triangle count
LEAF_TRIANGLE_COUNT_BITS = 7
MAX_TRIANGLE_COUNT = (1 << LEAF_TRIANGLE_COUNT_BITS) - 1
LEAF_FLAGS_BITS = 6
LEAF_VERTEX_OFS_BITS = 32 - LEAF_TRIANGLE_COUNT_BITS - LEAF_FLAGS_BITS  # 19

# AABB tree inner nodes
# 24b child offset, 6b mask, 2b axis
INNER_NODE_MASK_BITS = 6
INNER_NODE_CHILD_OFS_BITS = 32 - AXIS_BITS - INNER_NODE_MASK_BITS  # 24
# 16b max, 16b min

# coordinates are signed 16 bit integers
COORDINATE_BITS = 16
