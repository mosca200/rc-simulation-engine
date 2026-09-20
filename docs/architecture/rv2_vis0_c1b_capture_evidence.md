# RV2-VIS0-C1B — Tooling & Capture Evidence Contract

**Workstream:** LINEA 2
**Branch:** `work/line-2/rv2-vis0-c1b-tooling-contract`
**Base:** `origin/integration/render-v2` @ `b64eb79b3d50f12433375651c90c1cf83f6b0d4a`

## 1. Obiettivo

Chiudere due lacune **indipendenti** del benchmark VIS0, restando interamente nel
tooling e nella documentazione:

1. rendere completamente esplicito nel manifest lo stato iniziale aircraft che il
   runtime sa già configurare;
2. definire un contratto machine-readable stabile per le **future** capture
   evidence prodotte dal runtime.

Questa tranche **non produce immagini**, **non implementa framebuffer capture** e
**non modifica renderer/app runtime**. Nessun file sotto `crates/**` è toccato.

Relazioni:

- `docs/architecture/rv2_vis0_visual_benchmark.md` — il contratto VIS0-A (manifest).
- `docs/architecture/rv2_vis0_b_benchmark_runner.md` — il runner VIS0-B.
- Questo documento — il contratto evidence VIS0-C1B e l'handshake verso LINEA 1.

## 2. Lacuna 1 — stato iniziale aircraft esplicito

### Problema

VIS0-B aveva documentato un contract gap concreto: `rcsim-app render` espone già
`--altitude-m` e `--airspeed-mps`, ma il manifest v1 non poteva esprimerli. Il
runner dichiarava esplicitamente di non emetterli mai. Conseguenza: parte dello
stato iniziale aircraft dipendeva dai default impliciti del runtime
(`DEFAULT_ALTITUDE_M = 30.0`, `DEFAULT_AIRSPEED_MPS = 18.0`) e il manifest **non
determinava completamente la scena** — esattamente la proprietà che un benchmark
riproducibile deve garantire.

### Soluzione

Due nuovi campi nel blocco `aircraft`:

| Campo | Tipo | Range | Flag runtime |
| --- | --- | --- | --- |
| `aircraft.altitude_m` | number | `(0, 10000]` | `--altitude-m` |
| `aircraft.airspeed_mps` | number | `(0, 200]` | `--airspeed-mps` |

Nessun flag nuovo è stato inventato: il mapping punta ai flag reali già
implementati. I default del runtime non sono stati modificati.

Questa modifica evolve il **GoldenSceneManifest** da `1.0.0` a `1.1.0`:
`1.0.0` resta il contratto VIS0-A storico, mentre `1.1.0` richiede lo stato
iniziale airborne esplicito. Il validator del manifest e il suo JSON Schema
accettano soltanto `1.1.0`; non è introdotta alcuna migration o compatibilità
multi-versione. Questa revisione non modifica la versione indipendente del
contratto `VisualCaptureEvidence`.

### Bounds verificati sul codice, non assunti

Da `crates/app/src/render_app.rs`:

```rust
const DEFAULT_ALTITUDE_M: f64 = 30.0;
const DEFAULT_AIRSPEED_MPS: f64 = 18.0;
const MAXIMUM_ALTITUDE_M: f64 = 10_000.0;
const MAXIMUM_AIRSPEED_MPS: f64 = 200.0;
```

e, in `RenderOptions::parse_with_defaults`:

```rust
if !options.altitude_m.is_finite()
    || options.altitude_m <= 0.0
    || options.altitude_m > MAXIMUM_ALTITUDE_M
{
    return Err(RenderAppError::InvalidAltitude(value));
}
```

Da cui:

- **estremo inferiore esclusivo**: `<= 0.0` è scartato, quindi `0` **non** è un
  valore accettato;
- **estremo superiore inclusivo**: solo `> MAXIMUM_*` è scartato, quindi `10000`
  e `200` sono accettati;
- i valori devono essere finiti (niente NaN/Infinity).

Il validator riusa `_validate_finite_number(..., exclusive_min=True)`, lo stesso
meccanismo già usato per `camera.chase_distance_behind_m`, quindi non introduce
una seconda convenzione di range.

### Asimmetria con `start_on_ground`

| Condizione | `altitude_m` / `airspeed_mps` |
| --- | --- |
| `start_on_ground` assente o `false` (airborne) | **obbligatori entrambi** |
| `start_on_ground: true` (ground start) | **vietati entrambi** |

Il motivo è nel runtime. In `RenderApplication::new`:

```rust
let (initial_state, ground_below_render_origin_m, terrain_mode, initial_ground) =
    if ground_start {
        let initialized = supported_ground_start(&model)?;
        (initialized.state, initialized.ground_below_render_origin_m,
         RenderTerrainMode::Flat, initialized.ground_evaluation)
    } else {
        (render_initial_state(altitude_m, airspeed_mps), altitude_m as f32,
         RenderTerrainMode::Rolling, GroundEvaluation::zero())
    };
```

Sul ramo `ground_start` né `altitude_m` né `airspeed_mps` vengono letti: lo stato
iniziale deriva dal modello. Un manifest che li dichiarasse insieme a
`start_on_ground: true` **affermerebbe di determinare uno stato che il runtime
ignora**, cioè mentirebbe sulla propria autorità. Per questo la combinazione è
rifiutata — lo stesso principio per cui il blocco `camera` rifiuta
`pilot_position_render_m` quando `mode='chase'`, pur accettando il runtime
entrambi i flag.

Rendere i campi obbligatori sul ramo airborne è ciò che chiude davvero la lacuna:
l'alternativa (campi opzionali) avrebbe lasciato intatta la dipendenza dai
default. Il manifest è per progettazione un livello **più severo** di
`RenderOptions`, che invece ha default per ogni campo: non è una regola
incompatibile, è la stessa relazione già valida per `camera.vertical_fov_deg` e
per i campi camera mode-specific.

### Effetti sul tooling

`run_benchmark.py`:

- `CANONICAL_FIELD_ORDER` e `FIELD_POLICY` estesi (il runner fallisce chiuso se
  un campo del manifest non ha policy, quindi i due elenchi vanno aggiornati
  insieme — fatto);
- `EMITTABLE_FLAGS` esteso con `--altitude-m` e `--airspeed-mps`, e il commento
  che dichiarava il contrario è stato corretto;
- `_resolve_cli_value` formatta entrambi con `format_number`, quindi un intero
  resta integrale (`120`, non `120.0`) e un float usa la repr più corta che Rust
  riparserà esattamente.

Esempio di comando generato dalla scena di riferimento airborne:

```
rcsim-app render --renderer v2 --terrain-debug final --vegetation-debug final \
  --scenery flying-field --camera chase --camera-fov 55 --chase-distance-m 3.5 \
  --chase-height-m 1.25 --exposure-ev 0 \
  --model models/acro_electric_01/model.json --throttle 0.55 \
  --altitude-m 100.0 --airspeed-mps 18.0
```

### Scena di riferimento

`vis0_reference_scene.json` (ground start) **resta invariata**: è già coerente con
la nuova regola. È stata aggiunta una scena compagna,
`docs/validation/visual_benchmark/vis0_reference_scene_airborne.json`
(`aircraft_acro_airborne_cruise`), che esercita il ramo airborne con valori
espliciti. I parametri camera sono le costanti reali del runtime
(`EXPLICIT_CHASE_DISTANCE_M = 3.5`, `EXPLICIT_CHASE_HEIGHT_M = 1.25`,
`EXPLICIT_CAMERA_FOV_DEG = 55.0`). Anche questa è un esempio di contratto, **non**
una golden image approvata.

## 3. Lacuna 2 — VisualCaptureEvidence

### GoldenSceneManifest vs VisualCaptureEvidence

| | `GoldenSceneManifest` | `VisualCaptureEvidence` |
| --- | --- | --- |
| Risponde a | cosa **VOGLIAMO** renderizzare | cosa **È STATO** realmente renderizzato/catturato |
| Natura | intento riproducibile, approvato a priori | fatti osservati, registrati a posteriori |
| Valori | solo *requested* | *requested* **e** *actual*, separati |
| Schema | `golden_scene_manifest.schema.json` | `visual_capture_evidence.schema.json` |
| Validator | `validate_manifest.py` | `validate_capture_evidence.py` |
| Prodotto da | autore della scena | runner + (in futuro) runtime capture |
| Introdotto da | VIS0-A | VIS0-C1B |

Il manifest è la domanda; l'evidence è la risposta. Nessuno dei due può sostituire
l'altro: un manifest non dice cosa è successo davvero, e un evidence non autorizza
alcunché.

### requested vs actual restano distinti

I due insiemi vivono in oggetti separati e non si sovrappongono mai:

```
capture.requested.width              capture.actual.framebuffer_width
capture.requested.height             capture.actual.framebuffer_height
capture.requested.frame_index        capture.actual.presentation_frame_index
```

La divergenza è **rappresentabile e lecita**, ed è anzi il caso atteso oggi: la
finestra è creata con `LogicalSize::new(1_280.0, 720.0)` hardcoded, quindi una
richiesta 1920x1080 produce realmente 1280x720. Un contratto che costringesse i
due valori a coincidere nasconderebbe la lacuna invece di esporla; uno che
permettesse all'*actual* di sovrascrivere il *requested* renderebbe
l'intento illeggibile a posteriori.

### Policy di indisponibilità

**`null` è l'unico marcatore di "non disponibile".**

- una misura che il runtime non può riportare deve essere `null`;
- non deve mai essere `0`, `-1` o stringa vuota, perché quei valori si leggono
  come misure reali;
- nessun valore reale deve essere inventato per riempire un campo che il runtime
  non fornisce.

Il validator rende la regola strutturale, non solo documentale: dimensioni e byte
size hanno `minimum: 1`, gli indici di frame `minimum: 0` con `null` consentito,
le stringhe vuote o di soli spazi sono rifiutate. Un placeholder non passa.

### Policy di presenza

**Ogni leaf della shape v1 deve essere PRESENTE.** Gli unici due stati conformi sono:

```
PRESENTE + valore reale
PRESENTE + null
```

**Mai `MISSING`.** Le due condizioni non sono equivalenti e il contratto le
distingue strutturalmente:

| Stato | Significato |
| --- | --- |
| `"gpu_adapter_name": null` | il producer **ha seguito il contratto** e dichiara esplicitamente che il dato non era disponibile |
| chiave `gpu_adapter_name` assente | artefatto **incompleto / non conforme**, oppure scritto da un producer che non implementa questa versione del contratto |

Collassarle indebolirebbe la provenance: un artefatto incompleto risulterebbe
indistinguibile da uno onesto. Omettere un campo non è un modo per dire
"non disponibile", e omettere `verdict.visual_pass` non è un modo per lasciare il
verdetto aperto.

L'implementazione usa un sentinel `ABSENT` distinto da `None`: `dict.get(key)`
appiattisce le due condizioni in `None`, quindi i call site passano
`block.get(field, ABSENT)` e ciascun helper `_require_*` rifiuta il sentinel con
un messaggio dedicato. Una sola guardia per helper, non decine di controlli
manuali duplicati.

Lo **schema JSON applica la stessa regola** elencando ogni property nel
`required` dell'oggetto che la contiene, a tutti i livelli. Non è quindi
possibile che lo schema accetti ciò che il validator rifiuta, né il contrario:
`test_schema_requires_every_property_of_every_object` e
`test_validator_rejects_exactly_the_leaves_the_schema_requires` verificano
l'accordo su tutti i 39 leaf.

### Struttura del contratto (v1.0.0)

```
schema_version            "1.0.0" (major 1 obbligatorio)
scene_id                  stessa grammatica del manifest
manifest { path, path_display, sha256 }            sha256 obbligatorio, 64 hex
source   { commit_sha, commit_sha_short, branch, detached_head, dirty,
           dirty_entry_count, runner_name, runner_version }
renderer { version, exposure_ev, camera_mode, scenery_preset }
capture  { requested { width, height, frame_index },
           actual    { framebuffer_width, framebuffer_height,
                       presentation_frame_index },
           format,
           image     { path, sha256, byte_size } }
execution { capture_success, process_exit_code, failure_reason }
hardware  { operating_system, os_release, architecture,
            gpu_adapter_name, graphics_backend, driver_version, notes }
verdict   { visual_pass, visual_pass_reason }
```

39 leaf, tutti `additionalProperties: false` e **tutti obbligatoriamente presenti**
(ogni oggetto elenca nel proprio `required` l'intera lista di `properties`).

### Regole di validazione

Strutturali:

1. `schema_version` semver e major supportato (`1`);
2. `scene_id` con la grammatica del manifest, 3–128 caratteri;
3. **ogni leaf e ogni contenitore della shape v1 deve essere presente**: un campo
   omesso è errore, distinto dal campo presente con `null` (vedi § "Policy di
   presenza");
4. `manifest.sha256` obbligatorio e non nullo, 64 hex minuscoli;
5. `source.commit_sha` obbligatorio, 40 hex (repo SHA-1) o 64 hex (repo SHA-256);
6. `source.runner_version` semver;
7. enum chiuse per `renderer.version`, `camera_mode`, `scenery_preset`,
   `capture.format`;
8. campi sconosciuti **rifiutati** a ogni livello (vedi § "Unknown fields");
9. root non-oggetto, JSON malformato o file assente → exit code distinti.

Semantiche (cross-field):

10. `capture_success == true` ⇒ `image.path`, `image.sha256`, `image.byte_size`
    **e** i tre `actual.*` tutti non nulli: un capture dichiarato deve essere
    verificabile;
11. `capture_success == true` ⇒ `process_exit_code` nullo o `0`, e
    `failure_reason` nullo;
12. `capture_success == false` ⇒ tutti i leaf `image.*` **presenti e nulli**: un
    capture fallito non deve pubblicizzare un'immagine mai scritta, ma deve
    comunque dichiarare esplicitamente di non averla;
13. `capture_success == false` ⇒ `failure_reason` presente e non nullo: dire
    perché, non lasciarlo implicito;
14. `verdict.visual_pass` deve essere **presente** e deve essere `null`.
    **Qualsiasi** altro valore — `true`, `false`, una stringa — è errore, e anche
    l'omissione è errore.

Cross-check opzionale con `--manifest <path>`:

15. `manifest.sha256` deve coincidere con lo SHA-256 reale dei byte del manifest;
16. `scene_id` deve coincidere con quello del manifest;
17. ogni valore *requested* (width, height, frame_index, format) e ogni impostazione
    renderer (version, camera_mode, scenery_preset, exposure_ev) deve coincidere
    con il manifest fornito.

### Unknown fields: policy coerente e documentata

I campi sconosciuti sono **rifiutati**, non ignorati — la stessa scelta
fail-closed del manifest VIS0-A e del runner (`_reject_unmapped_fields`). La
ragione è la stessa: un artefatto prodotto da una versione più recente del
contratto verrebbe altrimenti letto in modo silenziosamente sbagliato, e un
benchmark visivo che fraintende il proprio input è peggio di uno che si ferma.
Il messaggio di errore elenca i campi ammessi e spiega la policy; il test
`test_unknown_field_error_documents_the_policy` la verifica, e
`test_schema_forbids_additional_properties_everywhere` impone
`additionalProperties: false` a **ogni** livello dello schema.

## 4. Evidence ≠ verdict

Il contratto separa strutturalmente i fatti dal giudizio:

- `execution.capture_success` — **fatto procedurale**: un'immagine è stata
  davvero prodotta;
- `verdict.visual_pass` — **giudizio visivo**: resta `null`.

`verdict.visual_pass` è dichiarato `"type": "null"` nello schema, è elencato nel
`required` del blocco `verdict`, e il validator lo rifiuta sia se valorizzato sia
se **omesso**. Omettere il verdetto non è un modo di lasciarlo aperto: sarebbe
indistinguibile da un artefatto incompleto. Il campo esiste comunque, riservato e
versionato, perché un futuro reviewer umano o un motore di metriche **approvato**
abbia già una casa contrattuale invece di doverne inventare una.

Nessuna soglia SSIM/PSNR/LPIPS, nessuno scoring percettivo, nessuna approvazione
automatica di baseline. Il test `test_no_metric_threshold_constants_exist`
verifica che i token `ssim`, `psnr`, `lpips` e `threshold` non compaiano affatto
nel validator, e `test_validation_never_fills_visual_pass` verifica che la
validazione non scriva un verdict.

La qualità visiva resta dominio futuro di human review o di metriche percettive
approvate. Un evidence con `capture_success: true` **non** è un'approvazione.

## 5. Handshake richiesto alla futura LINEA 1

La split dei produttori è espressa come dato nel codice
(`RUNTIME_SUPPLIED_FIELDS` / `TOOLING_SUPPLIED_FIELDS` in
`validate_capture_evidence.py`) e riportata nel `plan.json`/`run.json` alla voce
`capture_evidence_contract.handshake`. Nessun campo è di entrambi: il test
`test_schema_leaves_match_the_handshake_exactly` confronta la split con le leaf
dello schema, quindi un campo nuovo non può restare senza proprietario.

### Dati che il runtime capture (LINEA 1) dovrà consegnare — 10 leaf

| Leaf | Significato |
| --- | --- |
| `capture.actual.framebuffer_width` | extent reale del framebuffer presentato |
| `capture.actual.framebuffer_height` | extent reale del framebuffer presentato |
| `capture.actual.presentation_frame_index` | indice reale del presentation frame catturato |
| `capture.image.path` | percorso dell'immagine scritta |
| `capture.image.sha256` | SHA-256 dei byte dell'immagine |
| `capture.image.byte_size` | dimensione in byte |
| `execution.capture_success` | l'immagine è stata prodotta davvero |
| `hardware.gpu_adapter_name` | adapter wgpu, **solo se** realmente riportato |
| `hardware.graphics_backend` | backend (vulkan/dx12/metal/gl), **solo se** reale |
| `hardware.driver_version` | versione driver, **solo se** reale |

I tre leaf hardware restano `null` finché il runtime non li espone davvero:
nessun valore va derivato da `platform` o inventato.

### Dati che questo tooling fornisce già — 29 leaf

Provenance del manifest (`path`, `path_display`, `sha256`), provenance git
(`commit_sha`, `commit_sha_short`, `branch`, `detached_head`, `dirty`,
`dirty_entry_count`), identità del runner, impostazioni renderer lette dal
manifest, tutti i valori *requested*, `capture.format`,
`execution.process_exit_code` (osservato direttamente dal subprocess),
`execution.failure_reason`, OS/architecture e il blocco `verdict`.

### Requisiti che oggi il runtime NON può soddisfare

Da verificare sul codice, non ipotizzati. Perché l'handshake sia completabile,
LINEA 1 (o una tranche successiva) dovrà fornire:

1. **un backend di capture** — oggi `rcsim-app render` non scrive alcuna
   immagine; la crate `image` è usata solo per decodifica texture e per il
   generatore offline di texture terreno;
2. **l'extent reale del framebuffer** — oggi la finestra è
   `LogicalSize::new(1_280.0, 720.0)` hardcoded in `RenderApplication::resumed`
   e non esiste alcun flag di risoluzione;
3. **l'indice reale del presentation frame** — oggi il render loop non espone un
   frame counter e non esiste selezione di frame;
4. **auto-exit deterministico** — oggi il loop winit è `ControlFlow::Poll` ed
   esce solo su Escape o chiusura finestra, quindi un run non presidiato termina
   in timeout;
5. **metadata adapter/backend/driver** — oggi non esposti dal CLI.

Il tooling **non** è stato modificato per ottenere questi dati e il runtime
**non** è stato toccato: sono requisiti documentati, e nel frattempo ogni leaf
corrispondente resta `null`.

Nessuna dipendenza da funzioni, struct o nomi del branch LINEA 1: l'handshake è
puramente concettuale e espresso come nomi di campo JSON.

## 6. Integrazione nel runner (deliberatamente limitata)

`run_benchmark.py` è stato esteso **solo** per:

- **A.** mappare `aircraft.altitude_m` / `aircraft.airspeed_mps` sui flag reali;
- **B.** conoscere il contratto evidence: `plan.capture_evidence_contract`
  descrive schema, validator, stato `not-produced` e handshake;
  `run.json.capture_evidence` contiene uno skeleton **conforme** con ogni leaf
  runtime a `null`, e `run.json.capture_evidence_validation` riporta l'esito
  dell'auto-verifica.

Il runner **non** dichiara che il capture sia disponibile. Le capability
continuano a riportare, invariate:

- `capture_backend` = `unavailable`, `produces_image: false`;
- `resolution_enforcement` = `unsupported`, `enforced: false`;
- `warmup_frames` = `unsupported`, `enforced: false`;
- `process_auto_exit` = `unsupported`.

`artifacts.capture` resta `null` con la propria `capture_reason`, e
`verdict.visual_pass` resta `null`. Lo skeleton non è un artefatto scritto su
disco: vive dentro `run.json`, così non può essere scambiato per il referto di un
capture avvenuto.

`RUNNER_VERSION` e `PLAN_VERSION` passano a `1.1.0` perché la forma del plan è
cresciuta (nuova chiave `capture_evidence_contract`) e il runner ha nuova
capacità di emissione.

## 7. Scope rispettato

Modifiche confinate a:

```
docs/architecture/rv2_vis0_visual_benchmark.md      (aggiornato)
docs/architecture/rv2_vis0_b_benchmark_runner.md    (aggiornato)
docs/architecture/rv2_vis0_c1b_capture_evidence.md  (questo documento)
docs/validation/visual_benchmark/vis0_reference_scene_airborne.json  (nuovo)
tools/visual_benchmark/golden_scene_manifest.schema.json             (aggiornato)
tools/visual_benchmark/validate_manifest.py                          (aggiornato)
tools/visual_benchmark/test_validate_manifest.py                     (aggiornato)
tools/visual_benchmark/run_benchmark.py                              (aggiornato)
tools/visual_benchmark/test_run_benchmark.py                         (aggiornato)
tools/visual_benchmark/visual_capture_evidence.schema.json           (nuovo)
tools/visual_benchmark/validate_capture_evidence.py                  (nuovo)
tools/visual_benchmark/test_validate_capture_evidence.py             (nuovo)
```

**Nessun file sotto `crates/**` è stato modificato.** `render_app.rs` è stato
letto esclusivamente per verificare flag, costanti e bounds. Physics, integratore
(fixed step 500 Hz, RK4, f64), NED/FRD, aero, propulsion, controls, contacts,
scheduler, replay, fingerprints, determinismo, shader e render graph: invariati.

Solo Python standard library (`argparse`, `hashlib`, `json`, `math`, `re`, `sys`,
`pathlib`, `typing`). Nessuna nuova dipendenza: il test
`test_runner_depends_only_on_the_standard_library` è stato esteso con
`test_capture_evidence_module_is_also_standard_library_only`, così allowlistare
un modulo sibling non apre la porta a una dipendenza pip.

## 8. Test

| Suite | Test | Note |
| --- | --- | --- |
| `test_validate_manifest.py` | 84 | era 60; +24 su stato iniziale aircraft |
| `test_run_benchmark.py` | 133 | era 101; +30 su mapping e contratto evidence, +2 sulla completezza strutturale dello skeleton |
| `test_validate_capture_evidence.py` | 100 | nuova; +22 sulla policy di presenza (MISSING ≠ null) |

Copertura richiesta e dove è verificata:

| Requisito | Test |
| --- | --- |
| evidence valido | `test_honest_failure_evidence_is_valid`, `test_successful_capture_evidence_is_valid` |
| `schema_version` invalida | `TestSchemaVersion` (5 test) |
| scene_id mismatch | `test_mismatch_against_manifest_rejected` |
| manifest hash invalido | `test_malformed_sha256_rejected`, `test_digest_mismatch_against_real_manifest_rejected` |
| commit SHA invalido | `TestSourceProvenance` (6 test, di cui 4 sul commit SHA) |
| framebuffer dimensions negative/zero | `TestPlaceholderRejection` (10 test) |
| requested vs actual resolution distinti | `test_divergent_resolution_is_representable` |
| requested vs actual frame distinti | `test_divergent_frame_index_is_representable` |
| image hash mancante con capture riuscito | `test_success_without_image_sha256_rejected` |
| capture failure senza falsa immagine | `test_failure_with_false_image_rejected` (+ path, byte_size) |
| `visual_pass` non auto-impostato | `TestEvidenceIsNotAVerdict` (6 test) |
| hardware metadata unavailable accettabile | `test_all_null_hardware_is_acceptable` |
| unknown fields policy coerente e documentata | `TestUnknownFieldPolicy` (5 test) |
| **MISSING ≠ null: ogni leaf obbligatorio** | `TestFieldPresencePolicy` (21 test): parametrizzato su tutti i 39 leaf e i 10 contenitori |
| **schema e validator d'accordo sulla presenza** | `test_schema_requires_every_property_of_every_object`, `test_validator_rejects_exactly_the_leaves_the_schema_requires` |
| **coppie esplicite missing→FAIL / null→PASS** | `hardware.gpu_adapter_name`, `hardware.driver_version`, `capture.actual.presentation_frame_index`, `capture.actual.framebuffer_*`, `capture.image.sha256`, `capture.image.path`/`byte_size`, `verdict.visual_pass`, `source.branch` (detached HEAD), `manifest.path`, `capture.requested.frame_index`, `execution.process_exit_code`, `renderer.*` |
| **campi non-nullable rifiutano anche null** | `test_mandatory_leaves_reject_null_too` |
| **skeleton del runner strutturalmente completo** | `test_skeleton_omits_no_contract_leaf`, `test_skeleton_nulls_are_explicit_keys_not_absences` |
| bounds allineati al runtime | `test_aircraft_bounds_match_runtime_source` |
| handshake completo e disgiunto | `TestLineOneHandshake` (4 test) |
| capability invariate | `test_capability_gaps_are_still_reported_unchanged` |

Nota d'ambiente: su questa macchina lo stdout Python può essere cp1252 quando
ridiretto, dove il `✓` del validator VIS0-A solleva `UnicodeEncodeError`. Le
suite vanno eseguite con `python -X utf8`. Il nuovo validator emette solo marker
ASCII (`[OK]` / `[INVALID]`) e riconfigura gli stream con `errors="replace"`,
quindi non ha il problema.

## 9. Cosa questa tranche NON ha fatto

- nessuna image capture, encoding PNG/JPEG/EXR, GPU readback o screenshot Win32;
- nessun headless renderer;
- nessun TAA, SSIM, PSNR, LPIPS o scoring percettivo;
- nessuna approvazione automatica di baseline o di verdict visivo;
- nessun Full 3D Field, PhotoField o geometria aircraft di produzione;
- nessuna modifica a `crates/**`, al runtime CLI, ai default del runtime;
- nessuna dipendenza dal branch LINEA 1;
- nessun merge verso `integration/render-v2`.
