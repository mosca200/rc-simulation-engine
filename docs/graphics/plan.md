# Graphics plan — matrice RCG-01…RCG-14 (RCG-00)

Baseline di riferimento: `integration/current@535fbe1` (branch `rcg-00-baseline`).
Classificazioni contro lo STATO REALE verificato il 2026-09-09 (vedi
`docs/graphics/baseline.md`). Legenda raccomandazione: EXECUTE, REDUCE, REBASE,
REVIEW_EXISTING, INTEGRATE_FIRST, DEFER, POSSIBLY_SATISFIED.

## Matrice

| Pkg | Stato reale | Già trovato | Dove | Test presenti / passati | Prove visive | Manca | Rischio duplicazione | Dipendenze | Raccomandazione |
|---|---|---|---|---|---|---|---|---|---|
| RCG-01 Correctness fixes | 3 finding aperti su 4 | A già corretto in integration (`23ae238`, restore pipeline + regression GPU); B aperto (`gpu.rs:1140` fallback perde metallic/roughness); C aperto (glb.rs ignora scene/node/istanze, documentato); D aperto (`render_app.rs` opzioni camera order-dependent) | integration/current | regression GPU A: PASS; B/C/D: nessun test | n/a | fix B, C, D + test | basso (fix puntuali) | nessuno | **EXECUTE** (ridotto: solo B, C, D; A è ALREADY_FIXED) |
| RCG-02 Instrumentation & evidence capture | parzialmente dimostrato fuori repo | `tmp/capture-win.ps1` (screenshot finestra), evidence pack RCG-00 con metadata JSON; niente nel repo | tmp/ (untracked) | n/a | 6 PNG correnti | pipeline ripetibile in-repo: sequenze, video, profiling CSV/log, metadata automatici | medio se si riscrive lo script esistente | RCG-12 per il profiling | **EXECUTE** (partire da capture-win.ps1 + evidence.json RCG-00) |
| RCG-03 Visual target matrix | assente | nessun target misurabile; solo A/B qualitativi nei doc G3-VR1 | — | n/a | reference esterne RealFlight/aerofly da NON committare | matrice silhouette/materiali/luce/terreno/ombre/vegetazione/cielo/camera/FX/leggibilità | basso | dopo RCG-02 (serve acquisizione ripetibile) | **EXECUTE** |
| RCG-04 Asset pipeline foundation | larga parte già presente | loader GLB con materiali/texture/sampler (G1C/G1D), generatore GLB Acro, bin generator vegetazione/terreno, test determinismo byte-per-byte; PROVENANCE PV1 (CC0) | integration + PV1 | CPU PASS; GPU n/a | n/a | scene graph/node transform/istanze (finding C), import validation formale, naming/versioning asset, criteri LOD documentati | **alto** se si riscrive il loader invece di estenderlo | RCG-01 (C) | **REVIEW_EXISTING** + EXECUTE solo per gerarchia/istanze e validation |
| RCG-05 Hero Trainer | assente | Sig Kadet LT-40: solo fisica/evidenze (`model.json`), **nessun GLB** | models/sig_kadet_lt40_egv | n/a | nessuna | asset completo: silhouette, materiali PBR, superfici mobili, elica, luce/ombra | basso | RCG-04 (pipeline asset) | **EXECUTE** |
| RCG-06 Acro production asset | in gran parte soddisfatto | G3C-A foundation + G3C-B closure: GLB 252 KB con articolazioni, materiali PBR, binding; test dedicati | integration/current | `aircraft_asset_g3c.rs` PASS (renderer+model) | sì (catture RCG-00: aereo leggibile vicino e a 100 m) | verifica qualità materiali reali (finding B tocca primitive non texturizzate), propeller presentation | **alto** se si rigenera l'asset | RCG-01 (B) | **POSSIBLY_SATISFIED** → REVIEW_EXISTING, chiudere solo i gap verificati |
| RCG-07 Lighting & tone mapping | integrato | G3B: scena Rgba16Float, EV exposure, Khronos PBR Neutral, cielo analitico; risposta EV verificata in cattura s6 | integration/current | `hdr_pipeline_g3b.rs` 13 CPU + 1 GPU PASS | sì (s1/s6) | sun/sky response avanzata, color management completo, leggibilità vs cielo da tarare | medio | RCG-03, RCG-11 | **REVIEW_EXISTING** (ridurre a tarature mirate) |
| RCG-08 Shadowing | integrato | G3E 3 cascade + PCF 5×5 (VR1), bias/range stabili, ombre vegetazione; regression GPU | integration/current | 3 GPU test PASS | sì (ombre contatto/alberi in s1/s2/s5) | blend band cascade, penumbra tuning fine, cost measurement | medio | RCG-12 | **REVIEW_EXISTING** |
| RCG-09 Terrain & runway | integrato + candidato PV2 | G3A/G3A-R (mips, AF, 3-freq stack, fade, debug) in main/integration; PV2 sostituisce le mappe con set 1024² production | integration + PV2 `375e3d2` | `terrain_visual_g3ar.rs` + 5 GPU test PASS (set corrente); PV2: suite propria PASS, CI verde | sì per set corrente (tiling ancora percepibile); PV2 non catturato | integrazione PV2 + catture comparative | **alto** se si rifanno mappe invece di integrare PV2 | integrazione PV2 | **INTEGRATE_FIRST** (PV2) poi REDUCE |
| RCG-10 Vegetation & scenery | integrato + candidato PV1 | G3D/G3D-R instanced (culling, LOD0-2, shadow) + VR1 fidelity; PV1: 4 asset Poly Haven CC0 texturizzati con alpha shadow, 12 GLB committed | integration + PV1 `705119f` | 5 GPU test PASS (set procedurale); PV1: strict test + CI verde | sì per set procedurale; PV1 non catturato | LOD3 billboard, wind, integrazione PV2/PV1 congiunta, aggiornamento doc PV1 (obsoleto su R2/R3) | **alto** se si ricrea il set asset | integrazione PV1; RCG-08/07 per luce/ombre | **INTEGRATE_FIRST** (PV1) poi REVIEW_EXISTING |
| RCG-11 Sky & atmosphere | parziale | cielo analitico + haze/fog G3B/G3-VR1 (fog 0.0012, haze 0.68) | integration/current | regression strutturali shader | sì (horizon blend in s2/s3) | sole visibile, nuvole/gradienti credibili, condizioni coerenti, profondità aerea | medio | RCG-07 | **EXECUTE** (scope ridotto: sky response, non nuovo sottosistema) |
| RCG-12 Performance & LOD | non misurabile oggi | nessun profiling; budget LOD vegetazione già pinned da test; draw call per batch | integration/current | test strutturali draw-call PASS | n/a | harness frame-time (TIMESTAMP_QUERY disponibile), p95/p99, stalli, simulated-time discarded | medio | RCG-02 | **EXECUTE** (harness nuovo; misurare prima di ottimizzare) |
| RCG-13 Camera, motion & FX | parziale | G2F chase robusta; camera pilot/chase con FOV/distanza/altezza; finding D order-dependence | integration/current | test parse opzioni PASS (ordini limitati) | sì (s1-s5) | fix order-dependence (RCG-01 D), smoothing/framing, transizioni, propeller presentation, motion cues | medio | RCG-01 (D) | **REDUCE** (fix D in RCG-01; qui solo presentation) |
| RCG-14 Packaging & final validation | prematuro | gate tecnici verdi, evidence pack RCG-00, validate first-slice PARTIAL | integration/current | gate CPU/GPU PASS | parziale | convergence finale post PV1/PV2 + fix + harness + target matrix | n/a | tutti | **DEFER** (dopo RCG-01…13 e integrazione PV1/PV2) |

## Sequenza di integrazione raccomandata

1. **Integrare PV1** (`705119f`) — tocca gpu.rs/vegetation_assets.rs/shader.wgsl.
   PV1 e PV2 sono indipendenti (nessun file in comune oltre gpu.rs/shader.wgsl
   in punti diversi): l'ordine è indifferente sul piano dei conflitti; si
   raccomanda PV1 prima perché è il delta maggiore e perché i fix RCG-01 su
   gpu.rs (finding B) dovranno ribasarsi sulla base convergente. Aggiornare il
   doc architettura PV1 obsoleto (descrive il bake procedurale pre-R2).
2. **Integrare PV2** (`375e3d2`) su `integration/current` — conflitto atteso basso
   (terrain_textures/shader/gpu.rs), CI già verde.
3. **RCG-01** (B, C, D) subito dopo o in parallelo all'integrazione, su base
   già convergente, per non dover ribasare i fix.
4. RCG-02 → RCG-12 (harness e misure) in parallelo a RCG-03 (target matrix).
5. RCG-04 (gerarchia GLB/istanze) prima di RCG-05 (Trainer) e come consolidamento
   di RCG-06.
6. RCG-07/08/11 come tarature mirate su base integrata; RCG-13 dopo il fix D.
7. RCG-09/RCG-10 = review post-integrazione PV2/PV1 (non riscrivere).
8. RCG-14 ultimo, con evidence pack completo della build convergente.

## Regole anti-duplicazione

- Non ricreare asset vegetazione: il set production è PV1 (Poly Haven CC0); il
  set procedurale G3D-R resta solo come fallback/test.
- Non ricreare mappe terreno: PV2 è il set production; G3A-R resta historical.
- Non riscrivere il loader GLB: estendere `load_glb_document` con scene/node.
- Non introdurre un secondo sistema di esposizione/tone mapping: G3B è l'unico.
