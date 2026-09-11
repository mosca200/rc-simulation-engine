# RV2-VIS0 — Golden Visual Benchmark Contract

## Obiettivo

VIS0 definisce il **contratto machine-readable** per benchmark visivi riproducibili del renderer RC Simulation Engine.

Questo sistema permetterà di:
- Catturare screenshot in condizioni deterministiche e documentate
- Confrontare metriche visive tra versioni del renderer
- Identificare regressioni qualitative (non solo funzionali)
- Supportare review umane con metadata completi
- Integrare futuri sistemi di GPU profiling e TAA sequence tests

**VIS0 non implementa il capture runner.** Definisce solo il contratto: cosa catturare, come documentarlo, come validarlo.

## Differenza tra tipi di test

### Technical rendering tests
Test funzionali che verificano correttezza computazionale:
- Output buffer contiene valori attesi
- Pipeline non crasha
- Shader compila
- Texture caricate correttamente

**Non valutano qualità visiva.**

### Visual regression tests
Confrontano immagini tra versioni per detectar cambiamenti:
- Perceptual metrics (SSIM, PSNR) con tolerance
- Edge detection per shimmer/temporal artifacts
- Human review per qualità fotografica

**Non richiedono bitwise equality tra GPU/driver diversi.**

### Human visual review
Review qualitativa da parte di umani:
- Qualità fotografica
- Realismo materiali
- Coerenza artistica
- Artefatti visivi sottili

**Non automatizzabile, ma supportabile con metadata completi.**

### Deterministic simulation tests
Verificano che la simulazione fisica sia riproducibile:
- Stessi input → stessi output
- Fingerprint fisici invariati
- Replay consistenti

**Indipendenti dal renderer. Richiedono bitwise determinism dove previsto.**

## Perché bitwise equality NON è un requisito per visual regression

Screenshot **non sono bitwise-identical** tra:
- Diverse GPU (NVIDIA vs AMD vs Intel)
- Diversi driver
- Diverse versioni del driver
- Diverse configurazioni OS
- Diverse impostazioni di power management

Anche la stessa GPU può produrre output leggermente diversi tra sessioni per:
- Floating point non-determinism in shader
- Texture filtering variations
- Rasterization order
- Memory layout differences

**Requisito VIS0:** riproducibilità **semantica** (stessa scena, stessi parametri, stesso risultato visivo entro tolerance), non bitwise.

**Policy:**
- **Simulation determinism:** bitwise/exact dove previsto (fingerprint fisici, replay)
- **Visual regression:** perceptual/tolerance-based + human review
- **Raw pixel delta:** solo diagnostico, non gate
- **Cross-GPU/driver bitwise:** NON requisito

## Metadata per riproducibilità

Una cattura è riproducibile se documenta tutti i parametri runtime-configurabili:

### Scena e renderer
- `scene_id`: identificatore univoco
- `renderer.version`: `v1` | `v2` (maps to `--renderer`)
- `renderer.terrain_debug`: debug mode (maps to `--terrain-debug`)
- `renderer.vegetation_debug`: debug mode (maps to `--vegetation-debug`)
- `scenery.preset`: `none` | `flying-field` (maps to `--scenery`)

### Camera
- `camera.mode`: `pilot` | `chase` (maps to `--camera`)
- `camera.vertical_fov_deg`: FOV in gradi (maps to `--vertical-fov-deg`)
- **Pilot mode:** `camera.pilot_position_render_m` [x, y, z] (maps to `--pilot-position-render-m`)
- **Chase mode:** `camera.chase_distance_behind_m`, `camera.chase_height_above_m` (maps to `--chase-distance-m`, `--chase-height-m`)

### Risoluzione e output
- `resolution.width`, `resolution.height`: in pixels
- `capture.filename`: nome file (stabile, senza timestamp)
- `capture.format`: `png` | `jpg` | `exr`

### Esposizione
- `exposure_ev`: exposure value in EV stops, range `[-8, 8]` (maps to `--exposure-ev`)

### Aircraft
- `aircraft.model`: path al model.json (maps to `--model`)
- `aircraft.throttle`: 0.0-1.0 (maps to `--throttle`)
- `aircraft.start_on_ground`: boolean (maps to `--start-on-ground`)

### Temporizzazione
- `warmup`: frame di warmup prima del capture

### Metadata aggiuntivi
- `tags`: lista di tag per categorizzazione
- `reference_hardware`: GPU/driver/OS per cui la baseline è stata validata (opzionale ma raccomandato)
- `reference_image`: path a immagine di riferimento opzionale

## Naming convention

### Scene ID
Formato: `<category>_<scenario>_<variant>`

Esempi:
- `aircraft_acro_static_front`
- `aircraft_acro_dynamic_roll`
- `terrain_flyingfield_overview`
- `vegetation_forest_dense`
- `atmosphere_sunset_horizon`

### Golden baseline filename
Formato: `<scene_id>_<width>x<height>.png`

**STABILE, senza timestamp.** Esempi:
- `aircraft_acro_static_front_1920x1080.png`
- `terrain_flyingfield_overview_2560x1440.png`

Il timestamp/run id appartiene a:
- Evidence run directory (es. `runs/2026-09-11_143022/`)
- Metadata sidecar (es. `capture_metadata.json`)

### Directory structure
```
docs/validation/visual_benchmark/
├── vis0_reference_scene.json          # Manifest esempio
├── scenes/
│   ├── aircraft/
│   │   ├── acro_static_front.json
│   │   └── acro_dynamic_roll.json
│   ├── terrain/
│   │   └── flyingfield_overview.json
│   └── atmosphere/
│       └── sunset_horizon.json
└── baselines/
    └── reference_hardware/
        ├── rtx_4090_windows/
        │   └── aircraft_acro_static_front_1920x1080.png
        └── rx_7900_linux/
            └── aircraft_acro_static_front_1920x1080.png
```

## Golden-scene lifecycle

### 1. Definition
Creare manifest JSON con tutti i parametri richiesti, allineati al runtime reale.

### 2. Validation
Eseguire `validate_manifest.py` per verificare correttezza sintattica e semantica.

### 3. Capture (futuro)
Il capture runner (non implementato in VIS0) leggerà il manifest e catturerà l'immagine.

### 4. Review
- Automated metrics (SSIM, PSNR) con tolerance
- Human review per qualità

### 5. Approval
Se l'immagine è approvata, diventa baseline per quel reference hardware.

### 6. Regression detection
Future catture vengono confrontate con la baseline usando metriche perceptual, non bitwise comparison.

## Regole per aggiornare una baseline

Una baseline deve essere aggiornata solo se:
1. **Bug fix intenzionale:** il renderer è stato corretto e il cambiamento è desiderato
2. **Feature addition:** nuova feature grafica approvata per quality
3. **Hardware upgrade:** nuova reference hardware aggiunta

**Mai** aggiornare baseline per:
- Nascondere regressioni
- Compensare mancanza di tempo per fixare bug
- Cambiamenti estetici non approvati

Ogni aggiornamento deve:
- Essere documentato nel commit message
- Includere before/after comparison
- Essere approvato da human reviewer

## PASS/FAIL policy

### PASS
- Manifest valido (sintassi + semantica)
- Cattura riuscita
- Metriche entro threshold (se definite)
- Human review approvata (se richiesta)

### FAIL
- Manifest invalido
- Cattura fallita
- Metriche fuori threshold
- Human review rifiutata

**Threshold non sono definiti in VIS0.** Saranno definiti quando il capture runner sarà implementato.

## Gestione reference hardware

Diverse GPU producono output diversi. VIS0 supporta multiple reference hardware:

### Reference hardware metadata
```json
{
  "reference_hardware": {
    "gpu": "NVIDIA RTX 4090",
    "driver_version": "546.33",
    "os": "Windows 11 23H2",
    "notes": "Baseline validated on 2026-09-11"
  }
}
```

### Cross-hardware comparison
Non richiedere bitwise equality tra hardware diversi. Invece:
- Ogni hardware ha le proprie baseline
- Metriche perceptual (SSIM) per confronto cross-hardware con tolerance
- Human review per validare similarità

## Capacità runtime attuali vs future

### Supportato nel runtime attuale (v1 manifest)
- Renderer version: `v1` | `v2`
- Scenery preset: `none` | `flying-field`
- Camera mode: `pilot` | `chase`
- Camera parameters: FOV, pilot position, chase distance/height
- Exposure EV: `[-8, 8]`
- Aircraft: model, throttle, start_on_ground
- Debug modes: terrain_debug, vegetation_debug
- Resolution: width, height
- Warmup frames

### Future capability (non nel v1 manifest)
- Weather conditions (clear, cloudy, fog)
- Time of day (dawn, noon, dusk, night)
- TAA enabled/disabled
- MSAA samples configurabile
- HDR toggle
- Lighting intensity in lux configurabile
- Sun direction/azimuth/elevation configurabile
- Free camera con look_at
- Sequence tests (TAA temporal)
- GPU profiling integration

**Queste capacità future devono essere documentate come tali, non rese required nel manifest v1.**

## Futura integrazione con GPU profiling

VIS0 è progettato per supportare futuri profiling:

### Profiling metadata (futuro)
```json
{
  "profiling": {
    "expected_frame_time_ms": 16.67,
    "expected_gpu_time_ms": 12.34,
    "draw_calls": 1234,
    "triangles": 5678901,
    "texture_memory_mb": 512
  }
}
```

### Integration points
- Capture runner può raccogliere profiling data
- Manifest può includere expected performance bounds
- Automated regression detection per performance

## Futura integrazione con TAA sequence tests

TAA (Temporal Anti-Aliasing) richiede test su sequenze temporali:

### Sequence test metadata (futuro)
```json
{
  "sequence": {
    "frames": 60,
    "camera_motion": "orbit",
    "aircraft_motion": "static",
    "capture_mode": "all_frames"
  }
}
```

### Temporal metrics
- Shimmer detection
- Ghosting detection
- Edge stability
- Temporal consistency

## Separazione Simulation / Presentation

**Critico:** VIS0 non tocca la simulazione.

### Simulation (non toccato)
- Fisica (500 Hz, RK4, f64)
- Aerodinamica
- Propulsione
- Contatti
- Replay
- Fingerprint fisici

### Presentation (benchmark target)
- Renderer
- Shader
- Texture
- Post-processing
- UI overlay

**Se un file runtime viene modificato durante VIS0: STOP e spiegare perché.**

## Design per supporto futuro

VIS0 è progettato per supportare:

### Scenari
- **Full3D:** scene 3D complete con aircraft, terrain, vegetation
- **PhotoField:** rendering fotorealistico per screenshot promozionali
- **Aircraft review:** review dettagliata di ogni aircraft model
- **Terrain:** review qualità terrain rendering
- **Vegetation:** review qualità vegetation rendering
- **Atmosphere:** review qualità atmosphere/sky rendering (v2 renderer)
- **TAA:** test qualità temporal anti-aliasing (futuro)
- **Shadows:** review qualità shadow rendering
- **Materials:** review qualità PBR materials
- **Distant aircraft visibility:** test visibilità aircraft a distanza

### Estensibilità
- Schema JSON estendibile con campi opzionali
- Validator supporta custom rules
- Capture runner può essere esteso per nuovi tipi di cattura
- Metriche possono essere aggiunte senza breaking changes

## Architettura sistema (futuro)

```
GoldenSceneManifest (JSON)
        ↓
Capture Runner (non implementato in VIS0)
        ↓
PNG / sequence
        ↓
Metadata (JSON sidecar)
        ↓
Automated metrics (SSIM, PSNR, ecc.)
        +
Human Review
        ↓
Regression Report
```

### Componenti
1. **Manifest:** definizione scena (VIS0)
2. **Capture Runner:** cattura immagini (futuro)
3. **Metadata sidecar:** parametri usati per cattura (futuro)
4. **Metrics engine:** calcolo metriche (futuro)
5. **Review UI:** interfaccia per human review (futuro)
6. **Regression detector:** confronto con baseline (futuro)

## Limitazioni VIS0

### Cosa VIS0 fa
- Definisce contratto machine-readable allineato al runtime reale
- Fornisce validator per manifest con reject di unknown properties
- Fornisce manifest esempio con soli parametri runtime-configurabili
- Documenta architettura e policy
- Distingue capacità attuali da future

### Cosa VIS0 NON fa
- Non implementa capture runner
- Non cattura immagini
- Non calcola metriche
- Non confronta immagini
- Non modifica runtime
- Non tocca simulazione
- Non inventa capacità runtime inesistenti

### Cosa VIS0 NON è
- Non è un sistema di testing completo
- Non è un benchmark suite
- Non è un regression detection system
- Non è un'interfaccia utente

## Visual metrics futuri

Documentati, non implementati in VIS0:

- **Absolute pixel delta:** solo diagnostico, non gate
- **SSIM/perceptual metric:** futura, con tolerance
- **Edge/shimmer temporal metrics:** future, per TAA
- **Human review:** per qualità fotografica

**Cross-GPU bitwise image equality NON è un requisito.**

## Conclusione

VIS0 posa le fondamenta per un sistema di benchmark visivi riproducibile e estensibile. Il contratto definito ora:
- È allineato al runtime reale (nessuna capacità inventata)
- Distingue chiaramente cosa è supportato ora vs futuro
- Supporta riproducibilità semantica, non bitwise
- Richiede metadata completi per ogni cattura
- Mantiene separazione netta simulation/presentation
- Supporta multiple reference hardware
- È estensibile per futuri scenari e metriche

**Principi chiave:**
- Solo parametri runtime-configurabili nel v1 manifest
- Riproducibilità semantica con tolerance, non bitwise
- Metadata completi per ogni cattura
- Separazione netta simulation/presentation
- Supporto per multiple reference hardware
- Estensibilità per futuri scenari e metriche
