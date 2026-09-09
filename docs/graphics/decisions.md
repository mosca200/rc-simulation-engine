# Graphics decisions — RCG-00

Decisioni prese durante la ricognizione RCG-00 (2026-09-09), con motivazione.
Nessuna di queste decisioni modifica fisica, renderer stack, branch remoti o CI.

## D-01 — Base di integrazione: `integration/current@535fbe1`

`main@82d69e3` non rappresenta lo stato corrente: 19 commit di lavoro grafico
(G3B→G3E, G3-VR1/VR1.1, OA1) vivono solo su `integration/current`. Ogni futura
slice grafica e la matrice RCG usano `535fbe1` come base; il worktree
`.worktrees/rcg-00-baseline` (branch `rcg-00-baseline`) è il punto di partenza
documentale.

## D-02 — PV1/PV2 sono candidati, non integrati

RCG-00 non integra. PV1 (`705119f`) e PV2 (`375e3d2`) sono classificati
FEATURE_ONLY con CI verde e vengono raccomandati per integrazione immediata
(`INTEGRATE_FIRST` in RCG-09/RCG-10) perché duplicarli costerebbe più che
integrarli. Il remote PV1 è avanzato rispetto allo SHA noto all'orchestratore
(`5367920` → `705119f`, PV1-R3): la baseline usa lo SHA nuovo e documenta la
differenza.

## D-03 — Classificazione dei finding senza ricerca di stringhe

Ogni finding è stato verificato sul percorso codice reale:
- A = FIXED_IN_INTEGRATION (restore incondizionato pipeline + 2 regression GPU);
- B = STILL_PRESENT (fallback materiale condiviso scarta metallic/roughness);
- C = STILL_PRESENT ma limitazione documentata del subset G1A–G1D, non bug occulto;
- D = STILL_PRESENT (mutazione della selezione camera corrente).
RCG-01 va quindi ridimensionato a B/C/D: dichiararlo "completo" sarebbe falso,
dichiarare A aperto sarebbe duplicazione.

## D-04 — Prestazioni: NOT_MEASURED, harness rimandato

Il renderer non espone timestamp/profiling (documentato da G3E e G3-VR1).
RCG-00 non introduce un profiler: registra l'assenza, la disponibilità di
`TIMESTAMP_QUERY` sull'adapter e rimanda l'harness a RCG-02/RCG-12. Le soglie
del piano restano NON validate.

## D-05 — Evidenze visive: solo build corrente

Le immagini Gate 0 (`docs/validation/gate0/`) sono historical evidence e non
valgono per `535fbe1`. Il pack RCG-00
(`docs/validation/graphics/rcg-00/535fbe1e0e662bc46443120f95f7094d224b3bd3/`)
contiene 6 catture del renderer di produzione release con metadata completi
(commit, hash asset, GPU, backend Vulkan, driver 595.97, risoluzione finestra,
camera, scenario, EV, data). Nessun video: lo strumento non esiste; RCG-02 lo
deve fornire. Le reference RealFlight/aerofly restano esterne e non committate.

## D-06 — Strumenti diagnostici minimi, fuori dal repo

L'adapter/backend wgpu non è loggato dal prodotto. Per non aggiungere codice al
repository, la query adapter è stata eseguita con un crate usa-e-getta in
`tmp/rcg00/gpuinfo` (untracked, gitignored): esito Vulkan / RTX 3090 / 595.97,
riportato in `gpuinfo_out.txt` dentro l'evidence pack.

## D-07 — Non toccare worktree e file di altri flussi

I worktree esistenti (`.worktrees/*`, inclusi PV1/PV2 con file untracked della
pipeline asset) non sono stati modificati né puliti. `build.log` e `.worktrees/`
restano untracked e fuori da ogni commit RCG-00.

## D-08 — Doc PV1 obsoleto: segnalato, non riscritto

`renderer_pv1_vegetation_assets.md` al HEAD PV1 descrive il bake procedurale a
18 GLB, superato da R2/R3 (12 GLB Poly Haven CC0). RCG-00 non modifica branch
altrui: l'aggiornamento è vincolato all'integrazione PV1 (RCG-10).

## D-09 — Vincoli assoluti riconfermati

Passo 0,002 s / 500 Hz, RK4, f64, NED/FRD, SI, quaternione Hamilton, scheduler,
masse/inerzie/CG, aerodinamica, propulsione, servi, mixer, contatti, replay,
fingerprint e determinismo non sono stati toccati: il fingerprint runtime
(`07c48378…ba73`) e il replay verify (2000 step, due esecuzioni identiche) lo
confermano sulla build corrente.
