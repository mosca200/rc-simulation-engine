# RV2-VIS0-C2B — Runtime capture → benchmark tooling/evidence integration

**Workstream:** LINEA 2
**Branch:** `work/line-2/rv2-vis0-c2b-capture-evidence`
**Base:** `origin/integration/render-v2` @ `418d57c21c59fe3af0cd6b3815afed992e3e3356`

## 1. Obiettivo

Collegare il runtime capture VIS0-C2A, già integrato, al tooling VIS0. Prima di
questa tranche il runner costruiva un comando deterministico e registrava
provenance, ma dichiarava onestamente di non poter produrre alcuna immagine: i
leaf runtime del `VisualCaptureEvidence` restavano `null` perché non esisteva
nessuna fonte. Quella fonte ora esiste.

Pipeline realizzata:

```
GoldenSceneManifest 1.1.0
  → deterministic runner plan (PLAN_VERSION 1.2.0)
  → rcsim-app render --capture-frame/--capture-out/--capture-format
                      --capture-receipt-out/--exit-after-frame
  → PNG reale (RGBA8 lossless, display-referred)
  → RuntimeCaptureReceipt 1.0.0 reale
  → verifica indipendente del tooling (receipt + byte PNG + header PNG)
  → VisualCaptureEvidence 1.0.0
  → capture_evidence.json (autorevole) + run.json (copia di comodità)
  → STOP
```

Questa tranche **non implementa rendering**, **non modifica `crates/**`**, **non
introduce metriche visuali** e **non assegna visual PASS/FAIL**.

Documenti correlati:

- [`rv2_vis0_visual_benchmark.md`](rv2_vis0_visual_benchmark.md) — contratto VIS0-A (manifest).
- [`rv2_vis0_b_benchmark_runner.md`](rv2_vis0_b_benchmark_runner.md) — il runner.
- [`rv2_vis0_c1b_capture_evidence.md`](rv2_vis0_c1b_capture_evidence.md) — contratto evidence.
- [`rv2_vis0_c2a_frame_capture.md`](rv2_vis0_c2a_frame_capture.md) — contratto runtime della capture.
- Questo documento — l'integrazione fra i due.

## 2. Tre contratti, tre versioni, tre proprietari

| Contratto | Versione | Proprietario | Ruolo |
| --- | --- | --- | --- |
| `GoldenSceneManifest` | **1.1.0** (invariata) | tooling (VIS0-A/C1B) | intento: cosa vogliamo renderizzare |
| `RuntimeCaptureReceipt` | **1.0.0** (contratto runtime separato) | `rcsim-app` (VIS0-C2A) | dichiarazione runtime stretta: cosa è stato scritto |
| `VisualCaptureEvidence` | **1.0.0** (invariata) | runner (VIS0-C1B, producer C2B) | fatti: provenance + requested + actual verificato + esecuzione + hardware + verdetto nullo |

Nessuna delle tre versioni è stata bumpata. Il `RuntimeCaptureReceipt` **non** è
un `VisualCaptureEvidence`: è una dichiarazione di 8 campi che il runtime può
conoscere, e basta.

### `RuntimeCaptureReceipt` ≠ `VisualCaptureEvidence`

Il receipt dichiara soltanto:

```json
{
  "schema_version": "1.0.0",
  "presentation_frame_index": 10,
  "framebuffer_width": 320,
  "framebuffer_height": 240,
  "format": "png",
  "image_path": "<percorso assoluto del PNG>",
  "image_sha256": "<64 hex minuscoli>",
  "image_byte_size": 12345
}
```

Il `VisualCaptureEvidence` aggiunge ciò che il runtime non sa e non deve sapere:

- **provenance del manifest** — `manifest.path`, `path_display`, `sha256`;
- **provenance git** — `source.commit_sha`, `commit_sha_short`, `branch`,
  `detached_head`, `dirty`, `dirty_entry_count`, identità e versione del runner;
- **stato requested** — `capture.requested.width/height/frame_index`, cioè
  l'intento, tenuto separato dall'esito;
- **fatti runtime verificati** — `capture.actual.*` e `capture.image.*`, riempiti
  **solo** da un receipt trusted e da un PNG riverificato;
- **fatti di esecuzione** — `execution.capture_success`,
  `execution.process_exit_code`, `execution.failure_reason`;
- **fatti hardware visibili al tooling** — `hardware.operating_system`,
  `os_release`, `architecture`, `notes`;
- **verdetto visuale nullo** — `verdict.visual_pass = null` + motivazione.

## 3. Mapping manifest → capture CLI

### Flag guidati dal manifest

| Campo manifest | Flag | Valore emesso |
| --- | --- | --- |
| `capture.filename` | `--capture-out` | `<output_dir>/<scene_id>/<filename>`, **assoluto** |
| `capture.format` | `--capture-format` | `png` |
| `capture.frame` | `--capture-frame` | `N` (presentation frame zero-based) |
| `resolution.width` | `--render-width` | già presente da VIS0-C1 |
| `resolution.height` | `--render-height` | già presente da VIS0-C1 |

### Flag DERIVATI dal runner

Due flag emessi non corrispondono ad alcun campo manifest, e il plan lo dichiara
esplicitamente (`capture_plan.derived_arguments`, `provenance: "runner-derived"`):

| Flag | Valore | Derivato da |
| --- | --- | --- |
| `--capture-receipt-out` | `<scene_dir>/runtime_capture_receipt.json` | decisione del tooling: è il runner a scegliere dove sta il receipt, perché deve poterlo leggere, validare e confrontare. **Nessun campo manifest nomina un percorso di receipt** e attribuirglielo falsificherebbe la provenance. |
| `--exit-after-frame` | uguale a `capture.frame` | controllo di lifecycle del processo. `rcsim-app` rifiuta `--exit-after-frame < --capture-frame` (`ExitBeforeCaptureFrame`); nel caso canonico a frame uguali il runtime cattura, presenta, scrive PNG e receipt, e solo allora esce. |

I flag manifest-driven vengono emessi per primi, i derivati per ultimi: la
distinzione è visibile nell'argv stesso, non solo nel JSON.

### argv canonico di riferimento

Per `docs/validation/visual_benchmark/vis0_reference_scene.json`
(`aircraft_acro_static_front`, 1920x1080, `warmup = capture.frame = 10`):

```
<app> render
  --renderer v2
  --terrain-debug final
  --vegetation-debug final
  --scenery flying-field
  --camera pilot
  --camera-fov 55
  --pilot-position 0.0,1.8,20.0
  --exposure-ev 0
  --model models/acro_electric_01/model.json
  --throttle 0.0
  --start-on-ground
  --render-width 1920
  --render-height 1080
  --capture-out <scene_dir>/aircraft_acro_static_front_1920x1080.png
  --capture-format png
  --capture-frame 10
  --capture-receipt-out <scene_dir>/runtime_capture_receipt.json
  --exit-after-frame 10
```

Ordine deterministico (derivato da `CANONICAL_FIELD_ORDER`, poi i derivati),
`shell=False`, argv come lista: nessun passaggio da shell, nessun quoting
ricalcolato, i percorsi con spazi restano un singolo elemento.

## 4. Policy capture frame / warmup

Il frame di capture è un **presentation frame zero-based** contato da
`RenderRunControl`: con `--capture-frame N` vengono presentati i frame `0..N-1`
prima della cattura di `N`. Ne segue che il riferimento

```
warmup = 10
capture.frame = 10
```

significa esattamente: 10 presentazioni precedenti, cattura del frame 10.

La policy C2B è volutamente stretta. L'esecuzione reale è supportata **solo** se:

- `capture.frame` è esplicito, **e**
- `capture.frame == warmup`.

Non esiste un flag `--warmup` e non ne è stato inventato uno: il warmup è
**derivato** dalla relazione sopra (`field_mapping.warmup.status = "derived"`,
`runtime_capabilities.warmup_frames.status = "derived"`, `runtime_flag = null`).
Il warmup **non** è supportato per combinazioni arbitrarie, e non viene
dichiarato tale.

`GoldenSceneManifest` `1.1.0` **non** è stato ristretto: schema e validator
continuano ad accettare `capture.frame` assente. Se un manifest è formalmente
valido ma non eseguibile, il runner **fallisce chiuso prima di avviare l'app**
con:

```
C2B requires explicit capture.frame matching warmup.
```

## 5. Policy formati: manifest valido ≠ runtime eseguibile

Il backend VIS0-C2A supporta soltanto `png`. Quindi:

| `capture.format` | Manifest valido | Eseguibile in C2B |
| --- | --- | --- |
| `png` | sì | **sì** |
| `jpg` | sì | no |
| `exr` | sì | no |

Il contratto può continuare a esprimere formati previsti storicamente o in
futuro; lo schema non è stato cambiato per questo. Il runner distingue i due
stati in `capture_plan.executable` + `capture_plan.blocking_reasons`, e su
`--execute` fallisce chiuso **prima** del processo (exit `1`). Una scena bloccata
non emette alcun flag capture: stampare un comando che il runtime rifiuterebbe
renderebbe il plan una menzogna.

`capture.quality` resta `unsupported` e **non** viene ignorato silenziosamente:
il contratto VIS0-A lo ammette solo con `format: "jpg"`, e il PNG lossless non ha
parametro di qualità, quindi la sua presenza blocca l'esecuzione e viene
riportata sia nel plan sia su stderr.

## 6. Output path

```
<output_dir>/<scene_id>/
    <capture.filename>              # PNG reale scritto dal runtime
    runtime_capture_receipt.json    # RuntimeCaptureReceipt 1.0.0 scritto dal runtime
    capture_evidence.json           # VisualCaptureEvidence 1.0.0 — artefatto AUTOREVOLE
    run.json                        # provenance completa + plan + copia dell'evidence
    stdout.txt
    stderr.txt
```

`--capture-out` e `--capture-receipt-out` ricevono percorsi **assoluti**, così il
`image_path` che il runtime riversa nel receipt è interpretabile senza conoscere
il cwd del subprocess. La metadata human-readable può comunque usare path
display/repo-relative dove già previsto.

## 7. Stale artifact safety

Prima di una vera esecuzione il runner rimuove, **fallendo chiuso**:

- l'immagine di capture preesistente;
- `runtime_capture_receipt.json`;
- `capture_evidence.json`;
- i sibling temporanei pertinenti (`<nome>.tmp`, la convenzione del runtime) e
  qualunque `*.tmp` residuo dentro `scene_output_dir`.

Il runtime VIS0-C2A ha già una propria stale-output policy, ma non può coprire i
casi in cui **il processo non parte** o **muore prima della propria cleanup**:
senza questo passo un'immagine della run precedente resterebbe sul percorso
atteso e sarebbe leggibile come risultato della run corrente. Una rimozione
rifiutata (file bloccato, permesso negato) ferma la run con exit `4` invece di
procedere su una directory sporca.

Nulla fuori da `<output_dir>/<scene_id>/` viene toccato: le baseline approvate non
sono cancellabili da questo percorso, e un test verifica che un file estraneo
nella scene dir e uno fuori sopravvivano.

## 8. Lettura e validazione del `RuntimeCaptureReceipt`

`tools/visual_benchmark/runtime_capture_receipt.py` è un parser/validator piccolo
e fail-closed, stdlib-only. Non è un framework generale.

Campi richiesti **esatti** (8, né uno in più né uno in meno):
`schema_version`, `presentation_frame_index`, `framebuffer_width`,
`framebuffer_height`, `format`, `image_path`, `image_sha256`, `image_byte_size`.

Rifiutati:

- campo mancante;
- campo sconosciuto;
- `schema_version != "1.0.0"`;
- frame negativo o non intero (`10.0`, `"10"`, `true` compresi);
- `framebuffer_width <= 0` o `framebuffer_height <= 0`;
- `format != "png"`;
- `image_path` vuoto o non stringa;
- SHA non esattamente 64 hex minuscoli;
- `image_byte_size <= 0`;
- tipo errato su qualunque leaf;
- radice non oggetto;
- JSON malformato, file assente, percorso non regolare.

Gli errori si accumulano invece di fermarsi al primo, così una sola run spiega
tutte le divergenze. Nessun valore viene corretto, coercito o completato con un
default.

## 9. Receipt expectation check

Un receipt è **trusted** solo se coincide con il request plan su almeno:

- `presentation_frame_index`;
- `format`;
- `image_path` — confrontato in forma canonica (`normcase` + `normpath`, con i
  percorsi relativi risolti contro il cwd del subprocess), così differenze di
  separatori o di maiuscole non producono un falso mismatch.

`framebuffer_width` e `framebuffer_height` **non** vengono dedotti dal manifest e
**non** vengono confrontati con la resolution richiesta: restano i valori
dichiarati dal receipt e verificati contro il PNG. Se il receipt non coincide con
la richiesta, la capture end-to-end è **FAILED**: il receipt non viene corretto e
i valori richiesti non vengono sostituiti.

## 10. Verifica indipendente dell'immagine

Il runner non si fida del receipt. Dopo process exit `0` e receipt valido,
riverifica direttamente il file con la sola stdlib (nessuna Pillow, nessun
decode dei pixel, nessuna metrica, nessun confronto con baseline):

1. il file esiste;
2. è un file regolare;
3. byte size reale == `receipt.image_byte_size`;
4. SHA-256 reale == `receipt.image_sha256`;
5. signature PNG valida (`89 50 4E 47 0D 0A 1A 0A`);
6. IHDR presente e leggibile (primo chunk, lunghezza dati 13);
7. width PNG == `receipt.framebuffer_width`;
8. height PNG == `receipt.framebuffer_height`;
9. bit depth == 8;
10. color type == 6 (truecolor con alpha).

Bastano 33 byte a prefisso fisso: signature (8) + length (4) + type (4) + dati
IHDR (13) + CRC (4). Per questo leggere un header PNG non richiede una libreria
di immagini e non costituisce analisi dei pixel.

## 11. requested ≠ actual

I due insiemi restano rigorosamente separati:

```
capture.requested.width          ← manifest/plan
capture.requested.height         ← manifest/plan
capture.requested.frame_index    ← manifest/plan

capture.actual.framebuffer_width          ← SOLO receipt runtime verificato
capture.actual.framebuffer_height         ← SOLO receipt runtime verificato
capture.actual.presentation_frame_index   ← SOLO receipt runtime verificato
```

`requested != actual` è perfettamente rappresentabile e viene preservato come due
fatti: il runner non sovrascrive `requested`, non copia `requested` dentro
`actual` e il validator evidence non forza `actual == requested`. Un test
end-to-end lo dimostra eseguendo una capture in cui il runtime presenta davvero
un'estensione diversa da quella richiesta.

Se non esiste un receipt completamente trusted, i leaf runtime restano `null`
(fail-closed): un valore parzialmente verificato non viene comunque pubblicato.

## 12. Criteri di successo e di fallimento

### Successo

`execution.capture_success = true` solo con capture end-to-end verificata:

```
process exit 0
AND RuntimeCaptureReceipt trusted (parse + expectation check)
AND PNG verificato in modo indipendente (size, SHA, signature, IHDR, extent, RGBA8)
AND VisualCaptureEvidence accettato dal proprio validator
```

In quel caso l'evidence contiene:

| Campo | Valore |
| --- | --- |
| `execution.capture_success` | `true` |
| `execution.process_exit_code` | `0` |
| `execution.failure_reason` | `null` |
| `capture.actual.framebuffer_width` | width del receipt |
| `capture.actual.framebuffer_height` | height del receipt |
| `capture.actual.presentation_frame_index` | frame del receipt |
| `capture.image.path` | immagine verificata |
| `capture.image.sha256` | hash reale verificato |
| `capture.image.byte_size` | size reale verificata |
| `verdict.visual_pass` | `null` — nessun giudizio automatico |

### Fallimento

Con processo non partito, timeout, exit code != 0, receipt mancante, receipt
invalido, mismatch receipt/richiesta, immagine mancante, hash mismatch, size
mismatch, header PNG invalido, dimensioni PNG != receipt, o PNG non RGBA8:

```
capture_success = false
failure_reason  = spiegazione concreta (non un capability gap generico)
visual_pass     = null
capture.image.* = null, null, null
capture.actual.* = null, null, null
```

Per il contratto v1 esistente i campi image di un fallimento restano `null`:
un'immagine non viene pubblicizzata come valida, e il contratto non viene
indebolito per salvare una run fallita.

### `process exit 0` non basta più

`verdict.runner_success` (e l'exit code del runner) richiede tutte e quattro le
condizioni. Se il processo esce `0` ma receipt/immagine/evidence non superano la
verifica, il runner restituisce **`EXIT_EXECUTION_FAILED` = 4**, non `0`.

## 13. Evidence standalone

Su `--execute` il runner produce `capture_evidence.json` sia per una capture
riuscita sia, quando possibile, per un tentativo fallito onesto — un fallimento
con `capture_success: false`, `failure_reason` concreto e leaf runtime `null` è
un artefatto conforme e utile.

Il documento viene validato con `CaptureEvidenceValidator` **prima** di
considerare completata la run. Se il producer genera evidence non conforme:

- la run è **FAILED** (exit `4`);
- `capture_evidence.json` **non** viene scritto, perché pubblicare un artefatto
  autorevole che il suo stesso validator rifiuta sarebbe peggio che non
  pubblicarlo;
- il documento rifiutato e i suoi errori restano in `run.json`
  (`capture_evidence`, `capture_evidence_validation`,
  `capture_evidence_artifact.written = false`) per auditabilità.

`run.json` continua a incorporare la stessa evidence per comodità; quando è
valida, i due documenti sono identici byte per byte e un test lo verifica.
**La standalone evidence è l'artefatto autorevole.**

## 14. Dry run

Il default **resta** dry-run. Un dry run:

- non avvia `rcsim-app`;
- non produce PNG, né receipt, né evidence, né `run.json`, né la scene directory;
- non finge un receipt e non finge valori `actual`;
- mostra il comando **completo** pianificato, i percorsi di capture/receipt/
  evidence, gli argomenti derivati con la loro provenance e i motivi di blocco;
- riporta `visual_pass = null`.

È corretto — e voluto — che un dry run dichiari `capture backend = supported`
(capacità del runtime) e insieme `capture produced = false` (nessuna run
eseguita). Sono due affermazioni su oggetti diversi.

## 15. Runtime capabilities aggiornate

`build_runtime_capabilities()` riporta ora:

- `capture_backend.status = "supported"`, `produces_image = true`,
  `final_display_referred_capture = true`, con sottoblocchi che documentano
  frame selection, runtime receipt, process auto-exit ed explicit resolution
  enforcement, ciascuno con `mechanism` e `reason`;
- `capture_backend.capture_produced = null` nel plan: una capacità descrive il
  runtime, la produzione è un fatto per-run;
- `resolution_enforcement.status = "supported"`, `enforced = true`;
- `warmup_frames.status = "derived"`, `runtime_flag = null`, `enforced` vero solo
  se `warmup == capture.frame`;
- `process_auto_exit.status = "supported"`.

Il wording stale è stato rimosso e un test ne impedisce il ritorno:
`"CAPTURE BACKEND NOT YET AVAILABLE"`, `"no framebuffer readback"`,
`"no image output"`, `"no PNG writer"`, `"capture backend unavailable"`.

`expected_capture_basename` è stato sostituito da
`build_expected_capture_metadata`, che riporta `planned_path`,
`expected_filename`, `format`, `executable`, `blocking_reason` e un campo
`verified` che resta `null` finché un execute reale non conferma l'artefatto. Il
vecchio `produced: false` motivato da "backend unavailable" era diventato
semanticamente falso dopo C2A.

## 16. Hardware ancora non disponibile

`RuntimeCaptureReceipt` `1.0.0` non contiene `gpu_adapter_name`,
`graphics_backend` né `driver_version`. Di conseguenza quelle leaf di
`VisualCaptureEvidence` **restano `null`** finché non esisterà un handshake
machine-readable. Il logging testuale non viene parsato come autorità e la GPU
non viene indovinata dal sistema: un adapter inventato attribuirebbe la capture
all'hardware sbagliato.

`hardware.notes` spiega che `operating_system`, `os_release` e `architecture`
sono visibili al tooling (`platform.*`), mentre GPU/backend/driver non sono
ancora nel receipt runtime.

## 17. Verdetto visuale

`verdict.visual_pass` resta **sempre** `null`, nell'evidence, nel `run.json` e
nel `plan.json`, insieme al `visual_pass` top-level/legacy del runner. Non sono
implementati: SSIM, PSNR, LPIPS, pixel diff, perceptual hash, threshold,
PASS automatico, FAIL automatico, comparison score.

`capture_success != visual quality success`. Una capture verificata dice che
questi byte esistono e corrispondono al proprio receipt; non dice che
l'immagine sia giusta.

## 18. Field policy aggiornata

| Campo | Status | Runtime flag |
| --- | --- | --- |
| `capture.filename` | `cli` | `--capture-out` |
| `capture.format` | `cli` | `--capture-format` |
| `capture.frame` | `cli` | `--capture-frame` |
| `warmup` | `derived` | nessuno — enforced dalla relazione col capture frame |
| `capture.quality` | `unsupported` | nessuno — e non ignorato silenziosamente |
| `resolution.width` / `height` | `cli` | `--render-width` / `--render-height` |
| `schema_version`, `scene_id`, `description`, `tags`, `reference_hardware` | `metadata-only` | nessuno |

È stato aggiunto lo status esplicito `derived` per ciò che controlla davvero il
runtime senza avere un flag proprio. Nessun campo che ora comanda il runtime è
più rappresentato come metadata-only. `--exit-after-frame` (lifecycle) e
`--capture-receipt-out` (evidence handshake) non compaiono nella field policy
come campi manifest: vivono in `capture_plan.derived_arguments`.

## 19. Versioni

| Elemento | Prima | Dopo |
| --- | --- | --- |
| `RUNNER_VERSION` | `1.1.0` | **`1.2.0`** |
| `PLAN_VERSION` | `1.1.0` | **`1.2.0`** |
| `GoldenSceneManifest` | `1.1.0` | **`1.1.0`** (invariato) |
| `VisualCaptureEvidence` | `1.0.0` | **`1.0.0`** (invariato) |
| `RuntimeCaptureReceipt` | `1.0.0` | **`1.0.0`** (contratto runtime separato) |

Nessuna incompatibilità concreta è stata introdotta nel contratto evidence, quindi
nessun bump era dovuto.

## 20. Scope rispettato

Modifiche confinate a:

```
tools/visual_benchmark/run_benchmark.py                 (runner + verifica + evidence)
tools/visual_benchmark/runtime_capture_receipt.py       (NUOVO: receipt reader + PNG verifier)
tools/visual_benchmark/test_run_benchmark.py            (test)
tools/visual_benchmark/validate_capture_evidence.py     (solo docstring: wording stale)
docs/architecture/rv2_vis0_c2b_capture_evidence_integration.md  (NUOVO, questo file)
docs/architecture/rv2_vis0_b_benchmark_runner.md        (aggiornato)
docs/architecture/rv2_vis0_c1b_capture_evidence.md      (aggiornato)
docs/architecture/rv2_vis0_visual_benchmark.md          (aggiornato)
```

**Zero** modifiche a `crates/**`, `Cargo.toml`, `Cargo.lock`, physics, replay,
renderer, app e all'implementazione della capture. Nessuna modifica a
`golden_scene_manifest.schema.json`, `validate_manifest.py`,
`test_validate_manifest.py`, `visual_capture_evidence.schema.json`,
`test_validate_capture_evidence.py` e ai due manifest di riferimento committed.

Non implementato, deliberatamente: C2C, metriche visuali, golden comparison,
baseline promotion, human-review UI, TAA, PhotoField, Full 3D Field, aircraft
asset work.

Nota: `visual_capture_evidence.schema.json` contiene ancora, nella sola
`description` testuale, l'affermazione storica "integration/render-v2 has no
capture backend yet". È stata lasciata intatta di proposito: lo schema è un file
di contratto versionato e questa tranche non lo modifica. L'affermazione è
superata dai fatti e questo documento lo registra.

## 21. Test

```
python -X utf8 -m unittest tools/visual_benchmark/test_validate_manifest.py -v
python -X utf8 -m unittest tools/visual_benchmark/test_validate_capture_evidence.py -v
python -X utf8 -m unittest tools/visual_benchmark/test_run_benchmark.py -v
```

Nessun processo grafico, nessuna finestra, nessuna GPU. La copertura
sottoprocesso reale usa l'interprete Python come eseguibile innocuo; il percorso
end-to-end di capture usa un test double che riproduce il **contratto di
artefatto** VIS0-C2A — scrive un vero PNG RGBA8 (stdlib `zlib` + `struct`,
nessuna libreria di immagini) su `--capture-out` e un `RuntimeCaptureReceipt`
byte-accurate su `--capture-receipt-out`, con manopole per receipt mancante,
digest malformato, digest errato, frame errato, `image_path` errato, signature
corrotta, extent diverso ed exit code non nullo. Non renderizza nulla.

Aree coperte: flag capture reali noti ed emettibili; mapping manifest → CLI e
provenance dei flag derivati; determinismo dell'argv e `shell=False`; policy
warmup/frame; policy PNG-only e `capture.quality`; dry-run senza side effects;
rimozione degli stale artifact; parsing strict del receipt; expectation check;
verifica indipendente del PNG; ownership requested/actual; evidence di successo e
di fallimento; evidence standalone identica a quella incorporata; validator
dell'evidence che determina l'exit code del runner.

Gate di regressione del workspace, eseguiti anche se `crates/**` non cambia, per
certificare che il tooling non abbia alterato accidentalmente nulla:

```
cargo fmt --all -- --check
cargo test --workspace --all-features
cargo clippy --workspace --all-targets --all-features -- -D warnings
```
