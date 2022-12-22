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
  size: header.ofs_indices - 64
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
    - id: child
      type: b24
    - id: flags
      type: b6
    - id: axis
      type: b2
      doc: Values 0-2 describe the axis to check against, value 3 marks a leaf
    - id: max # presumably?
      type: s2
    - id: min # presumably?
      type: s2
      # valid:
      #   expr: _ <= max # apparently does not always hold true
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
