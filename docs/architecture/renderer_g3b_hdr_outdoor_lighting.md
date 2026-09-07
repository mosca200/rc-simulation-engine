# G3B — HDR Outdoor Lighting

Slice base: `82d69e3` (Operational Alpha + OA2C + G3A-R).
Crate toccato: `crates/renderer` (+ CLI in `crates/app`). Nessuna modifica a
physics, determinismo, interpolazione o fingerprint: l'exposure è
presentation-only e il renderer resta isolato dai crate di dominio.

## 1. HDR pass topology

```
snapshot (f64→f32 interpolated, renderer read-only)
  → directional shadow depth pass (invariato)
  → scene HDR pass   → texture Rgba16Float (linear, scene-referred)
  → postprocess pass → exposure * Khronos PBR Neutral tone mapping
  → sRGB surface     → encode lineare→sRGB hardware
```

- Il target intermedio `Rgba16Float` (`HDR_FORMAT`) è creato a startup e
  ricreato **solo** su `resize()` / surface reconfigure. La view è ri-rilegata
  nel bind group del postprocess a ogni resize. **Nessuna creazione di
  texture/sampler/bind group/pipeline nel frame loop** (guard strutturali a
  `tests/hdr_pipeline_g3b.rs`).
- La surface/swapchain resta il display target finale, formato sRGB
  (`Bgra8UnormSrgb`/`Rgba8UnormSrgb` da capability).
- Depth e shadow target esistenti: invariati.

## 2. Texture formats e color-space contract

| Stage | Formato | Note |
|---|---|---|
| Albedo (GLB, terrain) | `Rgba8UnormSrgb` | decode sRGB→linear al sampling (hardware) |
| Normal / roughness | `Rgba8Unorm` / `R8Unorm` | dati lineari |
| Scene HDR target | `Rgba16Float` | risultati lighting lineari, nessun clamp LDR |
| Surface | sRGB | encode lineare→sRGB hardware (ultimo step) |

Catena colore unica: `texture sRGB → decode → lighting linear HDR → exposure
(× exp2(EV)) → tone map Khronos PBR Neutral → display linear → encode sRGB`.
**Non esiste alcun `pow()` gamma manuale nello shader**: la surface sRGB fa la
conversione; un doppio gamma sarebbe un bug. Il sole/cielo non sono più
clampati a [0,1] prima del tonemapper.

## 3. Exposure

- Parametro `--exposure-ev <f32>` (CLI, `render`/`play`), default `0.0`.
- `multiplier = exp2(exposure_ev)`.
- Validazione in `validate_exposure_ev` (renderer) e al parse (app): rifiuta
  non-finiti e fuori banda `[-8, +8]` EV. Nessuna auto-exposure temporale.
- L'exposure è applicata **nel postprocess** (scena scene-referred intatta) e
  non entra mai in physics/model fingerprint.

## 4. Tone mapper di default

`khronos_pbr_neutral` — formula esatta del **Khronos PBR Neutral** tone mapper
(reference: Khronos glTF-Sample-Renderer `tonemapping.glsl`): compressione
degli highlights con shift quadratico sulle ombre, desaturazione controllata
(`desaturation = 0.15`), risposta monotona e finita. Niente Reinhard (default),
niente AgX. Passthrough per valori sotto `start_compression` (le livree e il
medio-grigio restano fedeli). Pass dedicato fullscreen (triangolo a schermo
intero, sampler nearest deterministico 1:1).

## 5. Sky diffuse / environment specular

Sostituzione dell'ambient flat con un modello analitico deterministico:

- **Sky diffuse** (`sky_diffuse_irradiance`): emissività emisferica costruita
  dallo stesso gradiente zenith/horizon/ground del cielo procedurale, in
  funzione di `dot(normal, world_up)` (+ piccola lift sulle normali verso il
  sole). Coerente con il cielo outdoor, mai shadowata (il lato in ombra non è
  nero ma non è nemmeno lavato). Miscela [`sky_diffuse.xyz`, `.w`] nel
  `EnvironmentUniform` (128B totali, esteso da 96B).
- **Environment specular** (`environment_specular_response`): risposta
  roughness/metallic-aware passata dal path PBR — Schlick Fresnel + peso lobo
  `smoothness²` — così metalli (F0≈albedo) riflettono il colore cielo e i
  dielettrici restano sobri; sharp vs rough chiaramente distinguibili.
  Predisposto per IBL: i termini vivono nell'uniform e possono essere sostituiti
  da probe prefiltered in una slice futura.
- La directional shadow map modula **solo** il termine sun diretto
  (`direct = direct_unshadowed * shadow_visibility`); la sky contribution non
  è mai moltiplicata per l'ombra.

## 6. Lifecycle resources

| Risorsa | Startup | Resize | Frame |
|---|---|---|---|
| HDR target + view | ✔ | ✔ (recreate) | — |
| Postprocess sample | ✔ | — | — |
| Postprocess uniform buffer (UNIFORM+COPY_DST) | ✔ | — | write 16B |
| Postprocess bind group | ✔ | ✔ (rebind view) | — |
| Postprocess pipeline | ✔ | — | — |

## 7. Terrain G3A-R

Invariato semanticamente: stack a tre frequenze, mip chain 512→1, trilinear +
anisotropia (16/1 per capability), anti-repetition, fade del dettaglio, e
canali debug (`--terrain-debug`) continuano a funzionare sulla stessa
`fs_terrain` + uniform. Nessun asset terrain modificato.

## 8. Performance / resource discipline

- Frame loop: 5 `queue.write_buffer` su buffer persistenti (camera, aircraft,
  surface hinges, shadow matrix, postprocess uniform) — zero allocazioni
  sistematiche, zero creazioni GPU.
- Il postprocess è un fullscreen triangle senza depth; costo trascurabile.
- Target di progetto (Ryzen 7 5800X / RTX 3090): nessun benchmark inventato —
  va misurato sul hardware reale (comando sotto). La pipeline HDR aggiunge un
  solo sample-pass; i target 1080p120+/1440p90+ rimangono il riferimento.

## 9. Known limitations / confine futuro

- Nessuna auto-exposure (G3B esplicito), nessuna IBL prefiltered real: il
  modello analitico è il placeholder documentato per `G3F`/IBL.
- Il sun disk attraversa il tonemapper senza clamp prematuro; la gestione
  dell'aerial perspective completa resta G3F.
- `Rgba16Float` è sufficiente per questa scena; un pass compose HDR con
  più sample (AA G3I) può richiedere Rgba32Float in futuro.

## 10. Esecuzione manuale (RTX 3090)

```text
cargo run -p rcsim-app --release -- play --scenery flying-field --exposure-ev 0.0
cargo run -p rcsim-app --release -- render --model models/acro_electric_01/model.json --scenery flying-field --camera chase --exposure-ev 1.0
```