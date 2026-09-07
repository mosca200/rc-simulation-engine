# G3D — Produzione vegetazione (fondazione)

Slice base: `4eae95b`. Crate toccato: `crates/renderer`. Nessuna modifica a
physics, determinismo, interpolazione o fingerprint: placement, culling e LOD
sono presentation-only, in puro Rust (nessuna dipendenza wgpu), deterministici
e coperti da test unitari. Non esiste ancora alcun codice GPU che consumi il
mondo: **l'integrazione nel renderer (`gpu.rs`) è la slice successiva**.

> **Stato:** la slice di integrazione renderer è completa (commit
> `G3D: integrate production vegetation renderer`). I §1–§8 descrivono la
> foundation; i §11–§18 documentano l'integrazione GPU, la separazione dalla
> scenery legacy e i contatori runtime. Il §9 elenca i gap residui aggiornati.

## 1. Cosa consegna questa slice

Due nuovi moduli pubblici + re-export da `lib.rs`:

- `vegetation_assets.rs` — asset procedurali: 2 specie × 2 varianti a 3 LOD,
  mesh generate deterministicamente, fattori PBR, bounding sphere, export GLB.
- `vegetation.rs` — il mondo: lista istanze deterministica (FlyingField) e
  pipeline di selezione per-frame (frustum + distance culling, LOD con
  isteresi, compattazione e raggruppamento per batch).

L'integrazione GPU (buffer istanze, draw instanced, sostituzione degli alberi
G2A di `scenery`) è esplicitamente rinviata: questa slice stabilisce e
testa il *contratto* dati che il renderer consumerà.

## 2. Placement (FlyingField)

`flying_field_layout(assets, seed, ground_y, half_extent, runway_clearance)`
è una pura funzione del seed (PRNG SplitMix64, default `DEFAULT_VEGETATION_SEED
= 42`, coerente col precedente `scenery::DEFAULT_TREE_SEED`). Due zone:

| Zona | Struttura | Spacing minimo | Scale |
|---|---|---|---|
| Near (campo) | 10 cluster a centri fissi, membri 10–16, scatter 14 m | 7.5 m | 0.85–1.25 |
| Boundary (anello) | 16 slot angolari, apertura 15%, raggio 160–230 m | 6.0 m | 0.75–1.15 |

- **Pairwise spacing garantito** tra ogni coppia accettata (entrambe le zone),
  verifica O(n²) in `placement_is_valid` (usata dai test e dal fail-safe).
- Il rettangolo di sicurezza runway + clearance è escluso; il lato flightline
  (+X) resta più rado, l'opposto più denso.
- **Budget totale 150–320 istanze** (`TARGET_MIN_INSTANCES`/`TARGET_MAX_INSTANCES`):
  guard attiva via `debug_assert!` in `VegetationWorld::flying_field` e
  vincolata dal test `placement_count_is_within_the_field_budget`.
- Seme diverso ⇒ layout diverso; stesso seme ⇒ liste bit-identiche (position,
  yaw, scale, asset, tint) — testato.

## 3. LOD e culling

`VegetationLodConfig` centralizza la policy (mai costanti sparse in shader):

| Soglia | Valore |
|---|---|
| LOD0 → LOD1 | 55 m |
| LOD1 → LOD2 | 130 m |
| Distance cull | 340 m |
| Banda isteresi | 10% |

- L'angolo più lontano della fascia boundary (230 m dal centro + offset camera
  ~30 m) resta dentro il budget di cull: nessuna istanza sparisce nel range
  operativo.
- Isteresi: passare a un LOD più economico richiede `threshold × (1 + band)`,
  tornare indietro `threshold × (1 − band)` — niente thrashing orbitando una
  soglia (test `lod_hysteresis_does_not_thrash_across_a_boundary`).
- Frustum: 6 piani normalizzati estratti dalla view-projection con clip WebGPU
  `z ∈ [0, w]` (left/right/bottom/top da righe 0/1 vs riga 3, near = riga 2,
  far = riga 3 − riga 2); test sfera-piano classico. Lo stato LOD iniziale è
  stimato dalla distanza dal centro campo + altezza camera tipica (2 m) così
  il primo frame parte in classe corretta.

## 4. Hot path per-frame (`update_visibility`)

**Zero allocazioni**: tutti gli scratch (visible compattato, sorted scratch,
per-group ranges, tabella LOD correnti, bounds debug) sono preallocati alla
costruzione e riusati via clear + swap, mai `Vec` freschi. Passi:

1. Frustum + distance cull con contatori separati (`culled_frustum`,
   `culled_distance`) — i debug mode e le stats distinguono le cause.
2. Assegnazione LOD con isteresi; gruppo = `asset × LOD_COUNT + lod`
   (12 gruppi, 4 asset × 3 LOD).
3. Prefix sum → ranges per gruppo, counting-sort nello scratch → istanze
   contigue per (asset, LOD), pronte per il draw instanced per mesh.
4. `VegetationFrameStats`: total, visible, culled (distanza/frustum), LOD
   counts, draw call stimate scena/ombra.

## 5. Layout istanza GPU

`VegetationGpuInstance` — 48 byte, `repr(C)`, bytemuck Pod:

```text
offset 0  position_yaw : xyz + yaw (rad)
offset 16 scale_tint   : scale uniforme + tint rgb (× ~±8%)
offset 32 lod_class    : LOD class, asset index, riservati
```

Lo shader compone `T = translate · rotY(yaw) · scale` al vertice: **nessuna
matrice 4×4 per istanza** su CPU o GPU. `instance_capacity()` prealloca a
multipli di 64 (stabile tra frame).

## 6. Asset procedurali

`VegetationAssetSet::production()` — 4 asset, deterministici, generati in
metri render-local con base a Y = 0:

- **Decidue** (A/B): tronco rastremato + 3 stub di ramo + 6/5 lobi sferici
  schiacciati, ombreggiature per-vertice deterministiche.
- **Conifere** (A/B): tronco rastremato + pila di tier (4 cono-olivastre,
  5 abete stretto) + cono apice; la variante B usa le sue 5 tier al LOD0
  (le varianti differiscono in topologia, non solo in scala).

Budget triangoli (vincolati dai test):

| Livello | Decidua | Conifera | Spec |
|---|---|---|---|
| LOD0 | 600 ria / 536 rb | 200 / 240 | totale produzione < 4000 |
| LOD1 ratio | 43% / 48% | 60% / 50% | 35–60% di LOD0 |
| LOD2 ratio | 16% | 27% / 23% | 8–28% di LOD0 |

PBR per parte: metallic 0 su tutto, roughness 0.85 bark / 0.65 foliage;
corteccia marrone-dominante, fogliame verde-dominante (test).

## 7. Export GLB

`export_glb(asset)` serializza il LOD0 (bark + foliage) in GLB 2.0 **byte-
deterministico** (JSON con chiavi ordinate): vertex interleaved con stride 48
(posizione 12 + normali 12 + color 16 + padding 8 — il padding è necessario
perché gli accessor leggono a offset `48·i`), accessor POSITION/NORMAL/COLOR_0
VEC3/VEC3/VEC4 FLOAT, indici Uint32, materiali metallic-roughness. Il round-trip
attraverso il loader di produzione (`load_glb_asset`) è testato per ogni asset:
stesso conteggio vertici/indici e stessi fattori materiali.

## 8. Contratti

- Puro Rust, nessuna dipendenza wgpu nei moduli vegetazione: tutto è unit-test
  (36 test vegetazione, bit-deterministico per struttura).
- Nessun valore di vegetazione torna a physics, aerodinamica, masse, collisioni
  o controlli: il mon- do è read-only rispetto allo snapshot di simulazione.
- Le costanti di policy (LOD, cull, budget, hysteresis) vivono una sola volta e
  sono riusate da produzione e test — mai duplicate.

## 9. Limitazioni note / confine futuro

- **LOD3 billboard**: gap residuo documentato (`LOD_COUNT = 3`). L'integrazione
  renderer include solo i tre LOD mesh; il billboard richiede acquisizione
  alpha/impostor non proporzionata a questa slice.
- **Animazione vento / texturing fogliame**: nessun wind animation né texture
  canopy; i colori per-vertice e i fattori PBR sono il contenuto visivo.
- L'asset set è fisso in produzione; i varianti tuning sono hard-coded.

## 10. Verifica foundation

```text
cargo test -p renderer vegetation            # 36 test vegetazione
cargo test -p renderer                       # suite renderer completa
cargo clippy -p renderer --all-targets -- -D warnings
cargo fmt --all -- --check
```

## 11. Integrazione renderer (G3D produttivo)

Il normale FlyingField runtime (preset `SceneryPreset::FlyingField`) ora
costruisce `VegetationWorld::flying_field` e il set GPU persistente
`GpuVegetation` al costruttore del renderer, e disegna gli alberi con draw
instanced per batch — i vecchi coni/dome G2A/G2E sono spariti dal production
path (v. §16).

### 11.1 Risorse GPU persistenti (`gpu.rs`)

| Componente | Quando | Dettaglio |
|---|---|---|
| `VegetationGpuMesh` ×24 (4 asset × 3 LOD × 2 parti) | costruttore | vertex/index buffer statici per (asset, LOD, bark/foliage), `material_index` |
| `instance_buffer` | costruttore | preallocato COPY_DST, `instance_capacity()` × 48 byte |
| materiali bark/foliage (white texture + `part_metallic`/`part_roughness`) | costruttore | gruppo 3; roughness 0.85 bark / 0.65 foliage, metallic 0 |
| `VegetationUniform` (gruppo 4) | costruttore | selettore debug 16 byte, riscritto solo al cambio modalità |
| `pipeline` (`vs_vegetation`/`fs_vegetation`) | costruttore | lit HDR (Rgba16Float), slot 0 mesh + slot 1 instanza |
| `shadow_pipeline` (`vs_vegetation_shadow`) | costruttore | depth-only, stesso depth bias G2B |

Nessuna texture, sampler, pipeline, bind group, shader o Vec con nuova capacity
viene creato per frame. Il frame loop riscrive solo il contenuto del buffer
istanze via `queue.write_buffer` e registra `draw_indexed(0..count, 0, start..start+n)`.

### 11.2 Formato istanza

`VegetationGpuInstance` (48 byte, `repr(C)`, Pod): `position_yaw` (xyz + yaw),
`scale_tint` (scale + tint rgb), `lod_class` (classe LOD + asset index). Il
vertex shader ricostruisce `T = translate · rotY(yaw) · scale` senza mat4 per
istanza (inline nel WGSL, normals corrette perché rotazione pura + scale
uniforme).

### 11.3 Frame loop (`WgpuRenderer::render`)

1. `world.update_visibility(eye, vp)` — zero allocazioni (scratch riusati).
2. `queue.write_buffer(instance_buffer, visible)` — solo se visibili > 0.
3. Scene pass: per ogni gruppo (asset × LOD) attivo, bark + foliage con i
   rispettivi materiali e l'intervallo istanze dal `batch_ranges` — il numero
   di draw dipende dai batch, mai dal numero di alberi (max 24 draw).
4. Shadow pass: stessi draw ma solo gruppi LOD0/LOD1 (LOD2 non casta).
5. Modalità `Culling`: log periodico (ogni 90 frame) di total/visible/culled/
   LOD counts/draw calls/byte upload/CPU ms via `tracing::info!`.

### 11.4 Debug

CLI `--vegetation-debug final|lod|culling|bounds`. `Lod` colorizza per classe
LOD deterministico nel fragment (verde/giallo/arancio); `Culling` non cambia
l'output ma logga i contatori; `Bounds` = percorso produttivo (gap, §9).
Getter pubblici: `vegetation_stats()`, `vegetation_instance_capacity()`,
`vegetation_last_instance_bytes()`.

## 12. Costo draw (max teorici FlyingField)

- Scene: gruppi attivi (≤ 12) × 2 parti = ≤ 24 draw.
- Shadow: gruppi LOD0/1 attivi (≤ 8) × 2 parti = ≤ 16 draw.
- Con frustum+culling reali i gruppi attivi scendono molto sotto il massimo;
  i contatori effettivi sono esposti via `vegetation_stats()` e nel log
  `Culling` (v. §17 report runtime).

## 13. Allocazioni per-frame (audit)

- Consentito e usato: `clear()+push()` su scratch preallocati, write su buffer
  persistenti, stack locals, `Instant` per il timing di log.
- Mai nel frame: `Vec::new()`/`collect`/`sort` con allocazioni/HashMap/String/
  `format!`/creazione risorse GPU. `update_visibility` è zero-allocation
  (§4 della foundation); il renderer non aggiunge nulla per frame.

## 14. Test integrazione

- CPU/strutturali (`gpu.rs::vegetation_tests`): mesh index contiguo senza
  duplicati e coincidente con il flattening `group*PART_COUNT+part`;
  uniform 16 byte; instance 48 byte; draw call = batch attivi × parti (mai
  per-albero); byte upload = visibili × 48; entry point WGSL + attributi
  istanza presenti; `fs_vegetation` senza tonemap/gamma propri ma con
  `lit_pbr_response` + `apply_distance_fog`.
- GPU `#[ignore]` (headless, `-- --ignored`): assemblaggio produttivo reale
  (`build_gpu_vegetation` + pipeline reali + mesh reali):
  - istanze a transform distinte proiettano a centroidi schermo distinti;
  - tutte le classi LOD0/1/2 renderizzano e il costo schermo decresce;
  - lo shadow caster istanziato scrive la silhouette nel depth map.

## 15. Fisica / fingerprint

Nessuna modifica a physics, aerodinamica, masse, contatti, propulsione,
controlli o flight core: la vegetazione è interamente presentation, read-only
rispetto allo snapshot. I test replay determinism (`crates/replay/tests/`)
verificano il fingerprint invariato.

## 16. Rimozione placeholder legacy

`FlyingFieldParams.legacy_tree_placeholders` (default `false`): il preset
production NON genera più `TreeTrunk`/`TreeCanopy`/`BoundaryVegetation` —
la scenery statica contiene solo runway, markings, fence, pilot markers, pali
e windsock (760 triangoli). I generatori G2A/G2E restano nel sorgente solo
come fallback dev/test raggiungibile via flag esplicito (`legacy_tree_scene()`),
mai dal preset. Test discriminanti: `production_scenery_contains_no_tree_objects_or_geometry`,
`legacy_placeholder_trees_are_an_explicit_opt_in`.

## 17. Contatori runtime (reportati dalla run RTX3090)

Total instances, visible, culled frustum/distance, LOD0/1/2, scene draw calls,
shadow draw calls, instance-buffer capacity, bytes upload/frame — tutti reali,
da `VegetationFrameStats` e getter del renderer; nessun valore inventato.

## 18. Verifica integrazione

```text
cargo fmt --all -- --check
cargo check --workspace --all-targets
cargo clippy --workspace --all-targets -- -D warnings
cargo test --workspace --all-targets
cargo build --workspace --release
cargo test -p renderer --lib vegetation -- --ignored   # GPU su RTX3090
```