# ENV1-A — Fondamenta materiali production e open asset

**Slice:** ENV1-A · Photorealistic Flying Field
**Branch:** `work/line-1/env1-a-production-material-foundation`
**Base:** `integration/render-v2` @ `e84876a1b93b9ccf7a4d04af777bf3d4eb2a0882`

## 1. Obiettivo

Sostituire nel percorso FINAL del terreno il solo materiale *procedurale* grass
con un materiale fotografico PBR derivato da un asset CC0 reale — Poly Haven
`sparse_grass` — introducendo al contempo un'acquisizione riproducibile e una
provenance verificabile degli open asset. Nessuna altra variabile visiva della
scena di riferimento viene toccata.

## 2. Punto di partenza (audit del codice esistente)

L'audit ha corretto un'assunzione del brief: il percorso FINAL del terreno **non
era** un materiale procedurale in shader. Era già un materiale PBR testurizzato
con tre mappe impegnate, un bind group dedicato (group 4) e una mip chain reale
generata a CPU. Ciò che era procedurale era il **generatore offline** che
produceva quei PNG (`terrain_textures::generate_terrain_textures`, 1024²).

Di conseguenza ENV1-A è prevalentemente una sostituzione di *sorgente asset* più
il completamento del percorso materiale glTF, non una nuova architettura shader.

Stato rilevato alla base:

| Ambito | Stato alla base |
| --- | --- |
| Terreno: mip chain | Già reale — 11 livelli 1024→1, box 2×2 nello spazio colore corretto |
| Terreno: formati | Già corretti — `Rgba8UnormSrgb` / `Rgba8Unorm` / `R8Unorm` |
| Terreno: sampler | Già trilineare + anisotropia 16× (o 1× se non supportata) |
| Terreno: texture nello shader | 11 `textureSample`, nessuna noise procedurale in fragment |
| glTF: `baseColorTexture` | Letto e usato |
| glTF: `normalTexture`, `metallicRoughnessTexture` | **Assenti** |
| glTF: filtri mipmap | **Collassati a single-mip** (`SamplerFilter::from_gltf_min`) |
| glTF: texture GPU | `mip_level_count: 1` ovunque |
| Aircraft di produzione | GLB **senza alcuna texture** (0 immagini, 0 texture) |
| Vegetazione | 12 GLB con solo `baseColorTexture`, `minFilter = 9987` (LINEAR_MIPMAP_LINEAR) |

## 3. Asset e provenance

`ENV1-GND-01` · Poly Haven `sparse_grass` · CC0 · autore *Amal Kumar*.

Acquisizione tramite API pubblica (`/info`, `/files`) con `User-Agent`
identificativo, nessuno scraping HTML. Ogni download è verificato contro size e
MD5 pubblicati dall'API; lo SHA-256 locale è registrato. I sorgenti 4k (16-bit
PNG, ~212 MB) restano nella cache gitignored `tmp/env1_source_cache/`; nel
repository sono impegnati solo i loro digest.

**Unità fisica.** Lo schema OpenAPI dell'API pubblica di Poly Haven definisce
`dimensions` come la dimensione su ciascun asse **in millimetri**. `sparse_grass`
dichiara `[2000, 2000]`, cioè un'area scansionata di 2.0 m × 2.0 m. Il manifest
registra quindi il valore grezzo (`dimensions`), l'unità documentata dal
provider (`dimensions_unit = "mm"`) e il valore derivato esplicito
(`physical_dimensions_m = [2.0, 2.0]`); la conversione vive in un solo punto
(`env1_assets.physical_dimensions_m`) e il validator rifiuta ogni combinazione
incoerente — `[2000, 2000]` con `"cm"`, oppure `physical_dimensions_m`
`[20, 20]`. Nessuna unità ipotizzata, nessuna conversione nascosta.

**Null deliberato.** L'API non restituisce un campo `license`: CC0 è registrato
da <https://polyhaven.com/license> (stessa citazione già usata in
`tools/vegetation_processing/PROVENANCE.md`) e la base è documentata.

Autorità: `docs/assets/env1/env1_open_assets.json`. Verifica fail-closed:
`tools/env1_asset_pipeline/verify_env1_assets.py`.

## 4. Ricetta di processing (versione 1)

I sorgenti 4k sono PNG **16-bit** (`Diffuse` e `nor_gl` truecolor, `Rough`
grayscale). Pillow tronca silenziosamente il RGB 16-bit a 8-bit, quindi il
processing è implementato in **Rust** (`crates/renderer/src/env1_material.rs`),
dove il crate `image` già presente decodifica correttamente i 16 bit.

Riduzione 4096 → 2048 con un **singolo box filter 2×2 esatto** nello spazio
colore corretto, poi un'unica quantizzazione a 8 bit:

- **base color** — decodifica sRGB → lineare, media in luce lineare,
  ricodifica sRGB. La funzione di trasferimento è l'unica canonica del crate
  (`texture::srgb_to_linear_f64` / `linear_to_srgb_f64`), ora condivisa con la
  mip chain del terreno. Alpha = 255: il sorgente non ha canale alpha.
- **normal** — dati lineari, orientazione OpenGL (+Y) già conforme allo shader.
  I quattro vettori sono decodificati in [-1,1], sommati e **rinormalizzati**,
  mai mediati come byte (che appiattirebbe il rilievo).
- **roughness** — aritmetica intera `u32` esatta: somma, divisione per 4 con
  arrotondamento half-up, riscalatura esatta 65535 → 255. Nessun floating point,
  quindi bit-identico su ogni piattaforma.

Un sorgente che non sia esattamente 4096×4096 viene **rifiutato**, non
ricampionato con un filtro diverso.

**Non applicati:** nessun boost di saturazione o contrasto, nessuna ombra
cucinata, nessun AO moltiplicato nell'albedo, nessuno sharpening cosmetico,
nessuna LUT, nessuno displacement. L'elenco è registrato nel campo
`not_applied` del manifest, e il validator lo rifiuta se accorciato.

Output impegnati in `crates/renderer/assets/env1/terrain/sparse_grass/`:
`RGBA8` (color type 6), `RGBA8` (6), grayscale 8-bit (0) — lo stesso layout dei
PNG procedurali esistenti, così il percorso di decode runtime non cambia.

## 5. Integrazione renderer

Tre modifiche minime, nessuna riscrittura:

1. `env1_material::runtime_assets` incorpora i tre PNG con `include_bytes!`,
   con gli stessi nomi di costante di `terrain_textures::generated`: il
   ripuntamento in `gpu.rs` è quindi di una riga.
2. `create_terrain_material` usa `ENV1_RUNTIME_EDGE` (2048) invece di
   `TERRAIN_TEXTURE_SIZE` (1024). La mip chain passa a **12 livelli** 2048→1.
   Formati, sampler, bind group, uniform e shader restano identici.
3. `DEFAULT_TERRAIN_TEXTURE_SCALE_M`: 4.0 → **2.0 m** per tile. Non è una
   scelta di tuning libera: è la misura fisica dell'asset. `sparse_grass`
   dichiara `dimensions = [2000, 2000]` mm, cioè 2.0 m per asse, quindi un tile
   di base copre esattamente l'area scansionata (2048 texel su 2.0 m =
   1024 texel/m). Il legame è registrato in `runtime_binding` nel manifest,
   verificato dal test Rust
   `env1_runtime_material_uses_the_documented_physical_tile_span` e
   ricontrollato contro la costante compilata da `verify_env1_assets.py`;
   nessun JSON viene letto a render time. Solo la frequenza del layer base
   cambia: nello shader
   `macro_uv = uv * (base_scale / macro_scale)` con `uv = world / base_scale`,
   quindi macro (48 m) e detail (0.40 m) si risolvono in metri assoluti e sono
   **invarianti** a questa costante. Il companion anti-repetition deriva dallo
   stesso UV base e scala con esso, restando decorrelato.

Il generatore procedurale e i suoi asset 1024² **restano impegnati e invariati**,
coperti dai loro test bit-a-bit; non alimentano più il percorso FINAL.

## 6. Capacità materiale glTF aggiunte

- `SamplerMipmapFilter` preserva l'asse di selezione mipmap del `minFilter`
  glTF, che prima veniva distrutto a livello di tipo (`SamplerFilter` è un enum
  a due varianti). `SamplerConfig::mipmap_filter` è `None` esattamente quando
  l'asset chiede single-mip: `None` è un'informazione, non un valore mancante.
- `create_gpu_material` onora ora tale mappatura invece di hardcodare
  trilineare.
- `PrimitiveMaterial` legge `normalTexture` (con `scale`) e
  `metallicRoughnessTexture`, condividendo la cache di decode per indice.

**Decisione di scope (approvata):** le texture GLB restano a
`mip_level_count: 1`. Con un solo livello la mappatura del sampler è inerte,
quindi aircraft e vegetazione restano **bit-identici** e il confronto
BEFORE/AFTER isola esattamente il materiale terreno. Due fatti verificati lo
rendono sicuro: il GLB aircraft di produzione non ha alcuna texture, e nessuno
dei 12 GLB vegetazione usa `normalTexture` o `metallicRoughnessTexture` — il
nuovo parsing è quindi a costo zero sugli asset esistenti. `normalTexture` e
`metallicRoughnessTexture` sono letti e testati ma **non** consumati dallo
shading: collegarli cambierebbe l'aspetto degli asset esistenti.

**Alpha attraverso le mips:** `downsample_rgba8_srgb` media già il canale alpha
invece di forzarlo opaco; un test dedicato lo verifica, come richiesto per il
foliage futuro. Nessuna soluzione foliage è implementata.

**Occlusion:** rinviata. Il group 4 ha binding 0–4 occupati, l'uniform ha uno
slot `_padding2` libero, ma aggiungere una quarta mappa significa cambiare
layout, bind group e shader per un beneficio che ENV1-A non richiede. Documentata
per una tranche successiva.

## 7. Debug e validazione

I sei canali `TerrainDebugMode` (final, albedo, normal, roughness, macro,
detail) restano catturabili: tutti e cinque i canali terrain sono stati
ricatturati con il nuovo materiale e i loro audit riportano il selettore atteso.
Il contratto `RuntimeVisualAudit 1.0.0` **non è stato modificato**: nessun campo
del blocco `terrain` legge le texture.

Tre probe GPU headless assertavano che il terreno fosse *green-dominant*: era
una proprietà dell'asset procedurale ritirato, non un invariante del renderer.
Il materiale fotografico (ciuffi verdi su terreno umido bruno) è caldo e povero
di blu, con medie misurate R > G > B. Le asserzioni sono state ri-puntate su
margini derivati dall'output **misurato**, e i test rinominati di conseguenza.

Un probe GPU leggeva i livelli di mip con dimensioni hardcodate a 1024: difetto
reale introdotto dal cambio di edge, corretto derivando le dimensioni dalla
chain stessa.

`visual_pass` resta `null`: è bloccato a `null` dal JSON Schema
(`"type": "null"`) e la evidenzia di cattura registra fatti, mai un verdetto
visivo.

## 8. Test

- `crates/renderer/src/env1_material.rs` — 21 test: determinismo bit-a-bit,
  correttezza dello spazio colore (la media in luce lineare deve essere più
  chiara della media sRGB), rinormalizzazione dei normali distinta dalla media
  dei byte, aritmetica intera esatta della roughness, preservazione alpha
  attraverso la mip chain, rifiuto di sorgenti non 4k.
- `crates/renderer/tests/env1_a_sparse_grass_material.rs` — 22 test: asset
  impegnati (dimensioni, bit depth, color type, opacità, normali unitari e
  +Z-dominanti, roughness dielettrica), mip chain completa 2048→1 con 12
  livelli e level 0 identico ai byte impegnati, contratto di wiring e di spazio
  colore, generatore procedurale ancora riproducibile, nessuna perdita di `wgpu`
  nel nuovo modulo, parsing `normalTexture` / `metallicRoughnessTexture` /
  `scale` su GLB sintetici minimi, e verifica che nessun asset impegnato usi i
  nuovi slot.
- `crates/renderer/src/texture.rs` — mappatura totale dei sei `minFilter` glTF,
  indipendenza dei due assi di filtro, funzioni di trasferimento sRGB.
- `tools/env1_asset_pipeline/test_env1_assets.py` — 36 test: parsing del
  manifest e campi obbligatori, ogni mutazione rifiutata, source SHA mismatch
  fail-closed, digest runtime deterministici, riprocessing bit-identico.

## 9. Limiti noti

- Le mappe runtime impegnate pesano ~24 MB contro ~1,6 MB delle procedurali: è
  il costo di un materiale fotografico 2048² senza perdita.
- Il layer detail a 0.40 m per tile campiona la stessa fotografia a forte
  ingrandimento; con contenuto fotografico appare più morbido che con la noise
  procedurale. L'architettura a tre frequenze è preservata come richiesto, non
  ri-tarata.
- La tinta vertex-colour G2D continua a moltiplicare l'albedo. È parte
  dell'architettura esistente ed è lasciata invariata.
- Le texture GLB restano single-mip per decisione di scope; la mappatura del
  sampler è ora fedele, quindi abilitare le mip chain GLB è una modifica locale.
- VRAM, tempo GPU di frame intero, timing specifico del terreno e draw call
  totali **non esistono** nel repository e non sono stati inventati: sono
  registrati come `null` con la ragione di indisponibilità.
