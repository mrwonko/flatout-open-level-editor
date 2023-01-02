FILE_ID = b'!vq\x98'
VERSION = 0
HEADER_SIZE = 64
NODE_SIZE = 8


# AABB tree node encoding
AXIS_BITS = 2  # 3 means leaf

# AABB tree leafs
# 23b triangle start, 3b kind, 4b mask, 2b axis (=3)
LEAF_MASK_BITS = 4
LEAF_KIND_BITS = 3
# 19b vertex offset, 6b flags, 7b triangle count
LEAF_TRIANGLE_COUNT_BITS = 7
LEAF_FLAGS_BITS = 6

# AABB tree inner nodes
# 24b child offset, 6b mask, 2b axis
INNER_NODE_MASK_BITS = 6
# 16b max, 16b min
