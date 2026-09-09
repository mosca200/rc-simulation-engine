# Graphics baseline — RCG-00

Stato tecnico reale del comparto grafico al commit
`535fbe1e0e662bc46443120f95f7094d224b3bd3` (`integration/current`), rilevato il
2026-09-09 sul checkout reale (worktree `.worktrees/rcg-00-baseline`, branch
`rcg-00-baseline`). Questo documento è una baseline: non aggiunge feature, non
integra branch, non modifica il renderer.

Evidenze correnti: `docs/validation/graphics/rcg-00/535fbe1e0e662bc46443120f95f7094d224b3bd3/`
(screenshot, log test, log GPU, replay, gpuinfo). Le immagini storiche in
`docs/validation/gate0/` restano historical evidence e NON valgono per questa
build.

## 1. Ambiente e toolchain (valori reali misurati)

| Voce | Valore |
|---|---|
| OS | Microsoft Windows 11 Pro, build 10.0.26200 |
| CPU | AMD Ryzen 7 5800X (8 core) |
| RAM | 31,9 GiB |
| GPU | NVIDIA GeForce RTX 3090 (24 GiB VRAM) |
| Driver GPU | 595.97 (`DriverVersion 32.0.15.9597`) |
| Adapter wgpu | `NVIDIA GeForce RTX 3090`, vendor `0x10de`, device `0x2204`, DiscreteGpu |
| Backend wgpu | **Vulkan** (selezione default di wgpu 30 su questa macchina, stessa call path del renderer: `gpu.rs:911-920`) |
| wgpu / winit / gltf | 30.0.1 / 0.30.13 / 1.4.1 (Cargo.lock) |
| rustc / cargo | 1.98.0 / 1.98.0 — coerente con `rust-toolchain.toml` (`channel = "1.98.0"`) e con la CI |
| Display | 1920×1080 @ 100 Hz (output NVIDIA); adapter virtuali secondari a 60 Hz |
| Feature GPU utili | `TIMESTAMP_QUERY` disponibile (base per harness RCG-12) |
| Blender | 4.2.16 LTS portable in `tmp/blender_portable` (usato dalla pipeline PV1, non dal runtime) |
| Acquisizione | Screenshot PNG finestra reale via `tmp/capture-win.ps1` (CopyFromScreen); sequenze immagine possibili; **nessun recorder video nel repo** |

## 2. Stato dei branch e CI (verificato con `git ls-remote` + API GitHub)

| Branch | SHA locale/remote | CI |
|---|---|---|
| `main` | `82d69e3` ✓ | — |
| `integration/current` = `feature/g3-visual-remediation-01` | `535fbe1` ✓ | run iniziale Windows **failure** (test `xfoil_aircraft_family_binding_m2_9l::fingerprint_unchanged_when_same_evidence_rebound`, JSON EOF); **re-run successiva verde** (ubuntu+windows success) |
| `feature/pv1-production-vegetation-assets` | `705119f` (avanzato rispetto al `5367920` noto all'orchestratore) | `705119f`: ubuntu+windows **success**. Il failure Ubuntu `apt-get`/Hash-Sum Chrome riguardava il commit superato `5367920` → CI_INFRASTRUCTURE_FAILURE storica, ALREADY_FIXED per avanzamento |
| `feature/pv2-production-terrain-materials` | `375e3d2` ✓ | ubuntu+windows **success** |

Delta commit: `main..integration/current` = 19 commit; `integration/current..PV1` = 4
commit (`8cbbc98`, `78dfdc8`, `003267f`, `705119f`); `integration/current..PV2` = 1
commit. Branch locali non più remoti (pruned): `integration/g3bcde-convergence`
(`8a648eb`), `feature/g3e-shadow-production`, `feature/g3c-b-aircraft-visual-closure`,
`feature/g3d-r-vegetation-visual-closure` — il loro contenuto è già dentro
`integration/current` tramite merge.

## 3. Inventario del lavoro grafico

### 3.1 Integrato in `integration/current@535fbe1` (INTEGRATED)

| Area | Contenuto | Commit chiave | Test |
|---|---|---|---|
| G1B/G1C/G1D | sky/haze/fog, world foundation, PBR metallic/roughness + GGX | pre-main | `shader_wgsl_g3a.rs`, unit test gpu.rs |
| G1E | superfici mobili articolate (control surfaces) | pre-main | `moving_surfaces_g1e.rs` (10 test PASS) |
| G2A–G2F | terreno, presentation campo, vegetation belt, ombre direzionali stabili, chase camera robusta | pre-main | suite renderer |
| G3A/G3A-R | terreno production: mip chain, trilinear+AF, stack 3 frequenze, anti-repetition, distance fade, debug channels | `9a2f88d` (in main) | `terrain_visual_g3ar.rs`, GPU test fs_terrain |
| G3B | HDR outdoor: scena `Rgba16Float`, esposizione EV, tone mapping Khronos PBR Neutral, cielo analitico | `be96aee` | `hdr_pipeline_g3b.rs` (13 CPU + 1 GPU PASS) |
| G3C-A / G3C-B | asset foundation aereo + closure visiva Acro Electric (GLB 252 KB, generatore `tools/generate_acro_electric_01_glb.ps1`, binding articolazioni) | `a18feb4`, `e614c5a` | `aircraft_asset_g3c.rs` (renderer+model) |
| G3D / G3D-R | vegetazione production instanced (culling, LOD0-2, hysteresis, shadow caster) + alberi organici e clustering | `baa306a`, `dec359a`, `23ae238`, `411db94` | `vegetation_tests` gpu.rs (5 GPU PASS), unit test |
| G3E | ombre production a 3 cascade, PCF, bias/range stabili | `00d883c` | `three_cascade_gpu_smoke…` (GPU PASS) |
| G3-VR1 / VR1.1 | remediation visiva: fedeltà LOD, fog 0.0012, haze 0.68, PCF 5×5, pali pista attenuati, leggibilità aereo distante | `34bbdba`, `7f18dac` | regression strutturali shader; A/B screenshot documentati |
| OA1 | preservazione evidence TX16S | `eef9bb5`, `3f49ef8` | test model/controller |

Totale test: workspace `cargo test --workspace --all-targets` = **0 failure** su
tutte le suite; renderer lib = 329 passed + 12 ignored (GPU); GPU ignored =
**12/12 PASS** + 1/1 PASS (`hdr_pipeline_g3b`).

### 3.2 Solo su feature branch (FEATURE_ONLY, candidati all'integrazione)

| Branch | Contenuto | Stato |
|---|---|---|
| PV1 `705119f` | Vegetazione production da modelli **Poly Haven CC0** (4 asset × 3 LOD, bark+foliage con texture e alpha shadow), `load_glb_bytes`, bin `generate_vegetation_glbs`, PROVENANCE.md, pipeline Blender/python in `tools/vegetation_processing` | IMPLEMENTED, CI verde, smoke runtime documentato; **non validato visivamente in RCG-00** (non integrato) |
| PV2 `375e3d2` | Materiali terreno production 1024² (albedo/normal/roughness), macro decorrelato, roughness indipendente | IMPLEMENTED, CI verde; **non validato visivamente in RCG-00** |

Nota di divergenza documentale: `docs/architecture/renderer_pv1_vegetation_assets.md`
al HEAD PV1 descrive ancora il bake procedurale "project-original" a 18 GLB
(stato `8cbbc98`), mentre R2/R3 (`003267f`, `705119f`) hanno sostituito il set
con 12 GLB Poly Haven CC0 texturizzati. Il doc va aggiornato in sede di
integrazione PV1 (RCG-10).

### 3.3 Mancante (MISSING) rispetto ai pacchetti RCG

- Trainer (Sig Kadet LT-40): esiste `models/sig_kadet_lt40_egv/model.json` ma
  **nessun GLB** → nessuna presentazione visiva del Trainer (RCG-05 interamente da eseguire).
- LOD3 billboard/impostor vegetazione; wind animation; foliage texture sul set
  procedurale (PV1 le introduce solo sul set Poly Haven).
- Instrumentation frame-time/profiling: assente (limitazione documentata da G3E).
- Automazione acquisizione evidenze nel repo (screenshot/sequenze/video/metadata): assente.
- Target matrix visiva vs benchmark esterni: assente.
- Blend band tra cascade ombre; sky/atmosfera oltre cielo analitico+haze G3B.
- Propeller presentation/animazione elica: intenzionalmente non implementata
  (non è un finding).

## 4. Verifica dei quattro finding del piano (codice corrente)

| Finding | Classificazione | Percorso codice | Evidenza |
|---|---|---|---|
| A — terrain/vegetation pipeline attiva durante aircraft draw | **FIXED_IN_INTEGRATION** | `gpu.rs:2200` restore incondizionato `triangle_pipeline` dopo i draw vegetation (fix `23ae238`); draw order sky→terrain→scenery→vegetation→restore→overlays→aircraft (`gpu.rs:2118-2230`) | regression GPU `scene_pass_aircraft_not_displaced_by_vegetation_pipeline` + `shadow_pass_aircraft_silhouette_not_displaced_by_vegetation` (PASS) |
| B — fallback materiali non texturizzati perde metallic/roughness | **STILL_PRESENT** | `gpu.rs:1140-1152`: primitive senza `base_color_texture` → `fallback_material_index` condiviso (white fallback); `metallic_factor`/`roughness_factor` della primitive scartati | lettura codice; nessun test copre il caso; non corretto in RCG-00 (scope RCG-01) |
| C — GLB loader appiattisce primitive, ignora scene/node/hierarchy/istanze/transform | **STILL_PRESENT** (limitazione documentata, subset G1A–G1D) | `glb.rs: load_glb_asset` itera `document.meshes().flat_map(primitives)`; nessun uso di `document.scenes()/nodes()`; PV1 aggiunge solo `load_glb_bytes` con lo stesso appiattimento | doc header glb.rs; diff PV1 |
| D — opzioni camera order-dependent | **STILL_PRESENT** | `render_app.rs:283-330`: `--camera` ricostruisce `CameraSelection` con default hardcoded; `--camera-fov`/`--chase-*`/`--pilot-position` mutano la selezione corrente via `apply_*` (`render_app.rs:1437-1500`) → `--camera-fov 40 --camera chase` perde il FOV | lettura codice + test esistenti che usano solo ordini "fortunati" (`render_app.rs:1727`) |

Nessun finding è stato confuso con feature intenzionalmente assenti (animazione
elica, modalità alpha avanzate).

## 5. Baseline CPU (eseguita sul worktree RCG-00)

| Comando | Esito |
|---|---|
| `cargo test -p renderer` | PASS (329 passed, 12 ignored GPU) |
| `cargo test -p renderer --test moving_surfaces_g1e` | PASS (10/10) |
| `cargo test -p model --test fingerprint` | PASS (14/14) |
| `cargo test -p replay --test aircraft_replay` | PASS (15/15) |
| `cargo test -p rcsim-app` | PASS (40/40) |
| `cargo fmt --all -- --check` | PASS |
| `cargo check --workspace --all-targets` | PASS |
| `cargo clippy --workspace --all-targets -- -D warnings` | PASS |
| `cargo test --workspace --all-targets` | PASS (tutte le suite, 0 failure) |
| `cargo build --workspace --release` | PASS (2 m 18 s) |

Nessun comando obsoleto: tutti i nomi di test richiesti esistono ancora.

## 6. Baseline GPU / visiva

- `cargo test -p renderer --lib -- --ignored`: **12/12 PASS** (terrain upload/readback,
  fs_terrain lit+debug ×4, vegetation instances/LOD/shadow ×3, pipeline isolation ×2,
  three-cascade smoke).
- `cargo test -p renderer --test hdr_pipeline_g3b -- --ignored`: **1/1 PASS**.
- Runtime production (release) con 6 catture finestra reali (vedi evidence pack):
  chase terra, pilot terra, chase 100 m in volo, pilot in volo, chase 25 m, EV +1.5.
- Osservazioni visive qualitative sulla build corrente: aereo leggibile da vicino
  e a 100 m (punto piccolo ma presente, coerente con VR1.1); ombre di contatto e
  ombre alberi presenti e stabili; terreno con tiling 4 m ancora percepibile e
  macro variazione moderata; cielo analitico piatto (gradiente unico, nessun sole
  visibile né nuvole); vegetazione G3D-R stilizzata (silhouette organiche ma
  low-poly a distanza); risposta esposizione HDR corretta senza clipping.
- VERIFIED_GPU: sì per i percorsi coperti dai test headless. VERIFIED_VISUALLY:
  parziale — le catture confermano funzionamento e composizione, non la qualità
  rispetto a benchmark esterni (compito RCG-03).

## 7. Replay, fingerprint, determinismo

- `replay verify --model models/acro_electric_01/model.json --input tests/datasets/aircraft_replay_v1/acro_electric_01_2000.json`:
  **PASS**, 2000 step, eseguito due volte con esito identico (determinismo confermato).
- Fingerprint fisico rigenerato dal runtime: `07c48378ad0f8de786f0927c1bba206681c4153deb50174bf0c518d6eae5ba73`
  (coerente tra `replay verify`, `validate first-slice` e stdout del viewer).
- `validate first-slice`: technical gate **PASS**, manual gate PARTIAL, real-world
  gate PARTIAL, overall **PARTIAL** (atteso: i gate manuali/reali richiedono
  revisione umana e confronti esterni, non sostituibili in RCG-00).
- Hash asset: `model.json` SHA-256 `F0CC5CE4…A0EEE`; `aircraft.glb` SHA-256 `55932887…C470D9`.

## 8. Prestazioni

**NOT_MEASURED.** Il renderer non ha alcun percorso di timestamp/profiling
(limitazione documentata da G3E e ribadita da G3-VR1). Le soglie del piano
(1920×1080 60 Hz, p95 ≤ 18 ms, p99 ≤ 25 ms, nessuno stallo > 100 ms) NON sono
validate né dichiarate valide. L'hardware (RTX 3090 + Ryzen 7 5800X) è ampiamente
sopra il minimo ragionevole per tali soglie a 1080p60, ma la verifica richiede
l'harness di RCG-02/RCG-12; `TIMESTAMP_QUERY` è disponibile sull'adapter.

## 9. Problemi residui registrati (non corretti in RCG-00)

1. Finding B, C, D ancora presenti (scope RCG-01/RCG-04/RCG-13).
2. Doc architettura PV1 non allineato a R2/R3 (set Poly Haven, 12 GLB).
3. Worktree PV1 contiene file untracked locali (`tools/vegetation_processing/processing_log.json`,
   `source_models/`) appartenenti al flusso di lavoro asset: non toccati.
4. Nessun recorder video; sequenze solo via cattura ripetuta.
5. `integration/current` ha avuto un failure CI Windows storico (JSON EOF nel test
   fingerprint M2-9L) non riprodotto localmente (14/14 PASS) e superato dalla re-run verde.
