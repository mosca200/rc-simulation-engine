# GLB Scene-Node Foundation (CPU presentation layer)

Status: implemented renderer-only slice on `integration/current@c1e1c77`.
Branch: `feature/glb-scene-node-foundation`.
Scope: `crates/renderer/src/glb.rs` (+ `lib.rs` exports, workspace `gltf`
feature `names`). No changes to `gpu.rs`, `shader.wgsl`, terrain, vegetation,
physics, replay or determinism.

Purpose: let the CPU-side GLB loader understand the real glTF scene graph
(scene → node → children → transform → mesh) so authored Blender assets with
object placement, hierarchy and shared meshes can be consumed by the future
GPU integration (ACRO HERO AIRCRAFT PRODUCTION) without flattened meshes.

## Scene selection

- The declared default scene (`scene` key) is used when present.
- Without a default scene, a single-scene asset loads that scene (unambiguous).
- Without a default scene and with more than one scene the loader fails with
  `GlbLoadError::MissingDefaultScene { scene_count }`: silently loading every
  global mesh is explicitly rejected.

## Traversal

- Recursive pre-order: scene roots in document order, each node before its
  children, children in document order. Deterministic by construction.
- A node visited twice (malformed cyclic hierarchy) fails with
  `GlbLoadError::CycleInNodeHierarchy { node_index }`.
- Nodes without a mesh act as pure transform groups and still propagate their
  world transform to descendants.

## Transform semantics

- `node.transform()` is interpreted as either an explicit glTF matrix
  (column-major in the gltf crate, `matrix[column][row]`) or a TRS
  decomposition composed as `T * R * S`.
- Local matrices convert to the renderer row-major `Mat4`
  (`rows[row][column] = columns[column][row]`).
- World transform is `parent_world * local`, matching `Mat4::mul` and
  `transform_homogeneous` (column-vector convention).
- Rotation quaternions `(x, y, z, w)` are normalized when finite and non-zero;
  non-finite or zero-norm quaternions, and any non-finite composed matrix,
  fail with `GlbLoadError::NonFiniteNodeTransform { node_index }`.
- glTF local space is untouched: this slice does not touch the NED/FRD
  simulation contract; transforms live only in the presentation layer.

## Instances and mesh deduplication

- `GlbAsset` now carries three views:
  - `primitives`: historical flat list (active meshes in ascending glTF mesh
    index order, primitives in document order) for existing consumers
    (`gpu.rs` batches, `GlbArticulationPlan` primitive indices, vegetation
    asset decoding). Unchanged for assets whose scenes reference every mesh
    with identity node transforms (Acro G3C-B GLB, PV1 vegetation GLBs).
  - `meshes: Vec<GlbMesh>`: geometry + material definitions, one entry per
    glTF mesh reachable from the active scene. Meshes outside the active
    scene are not decoded at all.
  - `instances: Vec<GlbSceneInstance>`: one entry per node mesh reference in
    traversal order, with `node_index`, optional `node_name` (Blender object
    names survive via the gltf `names` feature), `mesh_index` (slot into
    `meshes`) and the composed `world_transform`.
- Two nodes referencing the same mesh produce two instances and one mesh:
  geometry and embedded textures are decoded once (texture decode is also
  cached per glTF texture index across materials).
- Geometry is never baked with node transforms: placement stays in the
  instance so the GPU layer can instance shared meshes.

## Backward compatibility

- `load_glb_asset(path)` and `load_glb_bytes(data, label)` keep their
  signatures and error behaviour; `load_glb_mesh` still merges `primitives`.
- Production regression test
  `acro_production_glb_keeps_identity_instances_and_full_mesh_coverage`
  pins: identity instance transforms, full mesh coverage by instances and
  `primitives == concat(mesh primitives)` for the shipped Acro GLB.
- Articulation (`GlbArticulationPlan`) keeps mapping flat primitive indices;
  the ascending-mesh ordering preserves those indices for existing assets.

## Deliberately deferred (next GPU integration slice)

- Consuming `instances`/`world_transform` in `gpu.rs` (instanced draws,
  per-node object uniforms).
- Combining scene instances with `GlbArticulationPlan` hinges on the GPU side.
- `normalTexture`, `metallicRoughnessTexture`, occlusion/emissive, general
  alpha modes, skinning, animations, morph targets.
- Non-triangle primitive modes remain skipped.

## Verification

- 15 new unit tests cover: default scene, single scene without default,
  missing default with multiple scenes, root translation, nested parent+child
  composition, rotation quaternion, scale, column-major matrix, two nodes →
  same mesh → two instances, unused mesh exclusion, deterministic pre-order,
  `load_glb_asset`/`load_glb_bytes` equivalence, zero-quaternion rejection,
  cycle rejection, Acro production regression.
- Full gate on the branch: `cargo fmt --all -- --check`, `cargo check
  --workspace --all-targets`, `cargo clippy --workspace --all-targets --
  -D warnings`, `cargo test --workspace --all-targets`,
  `cargo build --workspace --release` — all PASS.
