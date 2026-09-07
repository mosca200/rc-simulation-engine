# G3D — Produzione vegetazione (fondazione)

Slice base: `4eae95b`. Crate toccato: `crates/renderer`. Nessuna modifica a
physics, determinismo, interpolazione o fingerprint: placement, culling e LOD
sono presentation-only, in puro Rust (nessuna dipendenza wgpu), deterministici
e coperti da test unitari. Non esiste ancora alcun codice GPU che consumi il
mondo: **l'integrazione nel renderer (`gpu.rs`) è la slice successiva**.

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

- **Integrazione renderer** (`gpu.rs`): buffer istanze persistente con write
  per-frame dal compacted list, pipeline instanced (2 parti × 12 gruppi),
  sostituzione del foliage G2A `scenery` statico e dei relativi indicatori.
- **LOD3 billboard**: gap residuo documentato (`LOD_COUNT = 3`).
- Niente vento/animazione, texture fogliame, o dettaglio artist-final: i colori
  per-vertice e i fattori PBR sono il contenuto visivo corrente.
- L'asset set è fisso in produzione; il varianti tuning resta hard-coded
  (deterministico per contratto).

## 10. Verifica

```text
cargo test -p renderer vegetation            # 36 test vegetazione
cargo test -p renderer                       # suite renderer completa
cargo clippy -p renderer --all-targets -- -D warnings
cargo fmt --all -- --check
```