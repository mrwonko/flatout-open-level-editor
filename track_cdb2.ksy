meta:
  id: track_cdb2
  file-extension: track_cdb2.gen
  endian: le
  bit-endian: le
seq:
- id: header
  type: header
  size: 64
- id: nodes
  type: nodes
  size: header.ofs_indices
- id: indices
  type: indices
  size: header.ofs_primitives - header.ofs_indices
- id: primitives
  type: primitives

types:
  header:
    seq:
    - id: magic
      contents: [0x21, 0x76, 0x71, 0x98, 0x00, 0x00, 0x00, 0x00]
    - id: mins
      type: vec3s4
    - id: maxs
      type: vec3s4
    - id: axis_factors
      type: vec3f4
    - id: inverse_axis_factors
      type: vec3f4
    - id: ofs_indices
      type: u4
    - id: ofs_primitives
      type: u4
  nodes:
    seq:
    - id: entries
      type: node
      repeat: eos
  node:
    seq:
    - id: first_dword
      type: u4
    - id: max # presumably?
      type: s2
      if: first_dword & 3 != 3
    - id: min # presumably?
      type: s2
      if: first_dword & 3 != 3
      # valid:
      #   expr: _ <= max # apparently does not always hold true
    - id: vertex_offset
      type: b19
      if: first_dword & 3 == 3
    - id: leaf_flags
      type: b6
      if: first_dword & 3 == 3
    - id: num_triangles
      type: b7
      if: first_dword & 3 == 3
    instances:
      child0_index:
        # this is actually the offset divided by 8
        value: first_dword >> 11
        if: first_dword & 3 != 3
      child1_index:
        value: (first_dword >> 11)+1
        if: first_dword & 3 != 3
      node_mask:
        value: (first_dword >> 2) & 63
        if: first_dword & 3 != 3
      axis:
        value: first_dword & 3
        if: first_dword & 3 != 3
      triangle_begin:
        value: first_dword >> 9
        if: first_dword & 3 == 3
      leaf_kind:
        value: (first_dword >> 6) & 7
        if: first_dword & 3 == 3
      leaf_mask:
        value: (first_dword >> 2) & 15
        if: first_dword & 3 == 3
  indices:
    seq:
    - id: entries
      type: index
      repeat: eos
  index:
    seq:
    - id: tbd
      type: u2 # TBD
  primitives:
    seq:
    - id: entries
      type: primitive
      repeat: eos
  primitive:
    seq:
    - id: tbd
      type: s2 # TBD
  vec3s4:
    seq:
    - id: elements
      type: s4
      repeat: expr
      repeat-expr: 3
  vec3f4:
    seq:
    - id: elements
      type: f4
      repeat: expr
      repeat-expr: 3
