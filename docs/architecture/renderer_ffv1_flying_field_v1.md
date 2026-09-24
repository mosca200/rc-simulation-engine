# FFV1 — Flying Field: vertical slice visuale v1

## 1. Obiettivo

Prima vertical slice VISUALE del Flying Field: aprendo il simulatore il campo
deve apparire come un ambiente RC moderno e credibile (prato falciato con
pista in erba, usura ai bordi, margine vegetale, luce diurna calibrata,
profondità atmosferica), non come una technical demo. Il lavoro è organizzato
come UN milestone unico: architettura, provenance, CI e performance restano
vincolanti ma sono al servizio del risultato visivo, non milestone separati.

Base autoritativa: `integration/render-v2` == `integration/current` ==
`224bad6588465b6828ed1f7236d479c04788a6f4` (precondition verificata prima di
iniziare; `work/product-release` non è stato usato).

## 2. Look development (luce e atmosfera)

Misura di riferimento: HDRI Poly Haven `meadow` (CC0, mezzogiorno, parzialmente
nuvoloso, basso contrasto) analizzata offline
(`tmp/hdr_analysis.json`: sole a ~69° di elevazione, radiance media cielo 1.26,
zenith 2.0-2.5, suolo fotografato 0.19). Con il preset clear-air originale il
rapporto cielo/suolo era INVERTITO (~0.7) perché la radianza del cielo fisico
era ~10× troppo bassa: ombre nere e campo fangoso.

Intervento (nessun nuovo sottosistema HDRI, nessun hack di post):
- nuovo preset `AtmosphereParameters::continental_summer_haze()` (aerosol
  ottico ~0.08, Rayleigh ×1.5, ground albedo neutro-basso): alza la radianza
  diffusa del cielo senza toccare il rapporto fisico sole/sky;
- `V2_FIELD_SUN_INTENSITY = 2.6` (irradianza scene-referred `PI * intensity`),
  esposizione resta `0.0` (il look è corretto a monte, non col post);
- il tone mapper Khronos PBR Neutral esistente comincia a comprimere le alte
  luci del cielo invece di restare nel tratto identico.

La prospettiva aerea RV2-6 resta quella esistente (tuned, non riscritta): con
l'aerosol nuovo il margine vegetale si stacca dall'orizzonte senza muro di
nebbia.

## 3. Terreno: regioni di campo e scala macro/meso/micro

Il terreno era UN tile fotografico uniforme (sparse_grass 2.0 m) e la "pista"
era una lastra grigia con strisce dipinte in `scenery.rs` (aspetto asfalto).

- Lastra e segnaletica dipinta RIMOSSA da `scenery.rs` (restano pali margine,
  recinzione, marker piloti, manica a vento); la pista diventa una REGIONE del
  materiale terreno.
- Maschera regioni analitica e deterministica in `shader.wgsl`
  (`field_region_weights`, specchiata in Rust in `terrain.rs` con test):
  pista falciata (round-box 6×60 m), collar usurato (±5 m), campo mantenuto
  (round-box 34×118 m), margine secco verso la vegetazione (150-205 m); i
  bordi sono perturbati da value-noise a due ottave (nessuna banda geometrica,
  nessun seam).
- Tre materiali CC0 Poly Haven miscelati con height-blend pesato sulla
  luminanza: `sparse_grass` (mantenuto, ENV1-GND-01, invariato a 2.0 m),
  `grass_path_3` (usurato, ENV1-GND-02, 1.0 m), `forest_ground_04` (secco,
  ENV1-GND-03, 3.15 m). Le scale sono i physical span misurati dall'API
  (mm→m) e sono vincolate da `verify_env1_assets.py` sul costante compilato.
- Scala MACRO: patch sane/secche a ~45 m (`health_tint`) + gain macro
  esistente; MESO: percorsi usurati gate-ati dal noise + collar; MICRO: stack
  detail 0.40 m esistente con fade per distanza.
- White-balance documentato del prato falciato (tint per regione) perché lo
  scan sparse_grass è turf secco e il riferimento (foto Wikimedia
  "The flying field.jpg", CC0) mostra un falciato verde medio.
- Falciature (mown laps) a 2.4 m solo sulle regioni mantenute.

Vincoli rispettati: `DEFAULT_TERRAIN_TEXTURE_SCALE_M = 2.0` invariato e
verificato in CI; nessun bind group nuovo (binding 5..9 del group 4); nessun
sistema di virtual texturing; terreno ancora fuori dal pass ombre.

## 4. Vegetazione

- Re-bake FFV1 di `field_broadleaf_a` da Poly Haven `tree_small_02` (CC0):
  atlas foglie 1k in luce lineare con filtro area 2×2, coverage gate-ata sulla
  whiteness del diffuse (una regione della mask sorgente è invertita), RGB di
  fondo riempito col colore medio foglia (i mip non sbiancano le card),
  vertex colour dello scan rimossi (tingevano il fogliame di marrone), copie
  orientate della corona a scala di cluster fogliare.
- Due asset ground-cover CC0 nuovi: `grass_medium_01`/`grass_medium_02`
  (`field_grass_a/b`), con cintura ground-cover a densità per distanza
  (piena <25 m, ridotta 25-60 m, nulla oltre) e hedgerow alla base della
  tree line; la pista falciata resta libera da ciuffi.
- Foliage: mip chain reali + anisotropia (seam ENV1-A), normali two-sided
  (`front_facing`), cutoff alpha compensato per distanza e mip bias negativo:
  le card non evaporano più al primo mip.
- Crown-preserving LOD: la corona mantiene le card LOD0 a ogni distanza
  (le card decimate dissolvono la silhouette); la bark continua a decadere.
- Ground cover e hedgerow non proiettano ombre (texel 4 cm non risolve un
  filo d'erba; solo aliasing).

Gap residuo documentato: le corone ad alpha-card erodono ancora a 150-230 m;
il tier raccomandato è un impostor/billboard LOD3 (fuori scope per il brief).

## 5. Performance

Misurata con il runner VIS0 esistente (nessun framework nuovo): hero scene a
1920×1080, 2560×1440 e 3840×2160 (sanity 4K), BEFORE (binary base 224bad6) vs
AFTER. I numeri disponibili sono i `gpu_duration_ns` per pass e il
`cpu_frame_duration_ns` del RuntimeVisualAudit 1.0.0 sul frame di capture
deterministico; FPS/percentili/1% low/VRAM/draw-call totali NON esistono nel
repository e sono registrati `null` con motivazione in
`docs/validation/ffv1_flying_field/ffv1_evidence.json`.

### Chiusura del campionamento GPU

Le capture originali al frame 10 non erano un campione stabile: in run ripetuti
il scene pass FFV1 oscillava da circa 1.8 a 17 ms e anche i tre shadow pass
oscillavano insieme, pur mantenendo identici i conteggi di vegetazione. Il
BEFORE al frame 10 misurava circa 1.23 ms, ma al frame 30 scendeva a circa
0.78 ms. Il comportamento è compatibile con un effetto di avvio del percorso di capture/GPU;
attribuire il rapporto 9-14x alle sole aggiunte FFV1 era prematuro.

Gli stessi manifest hero ora attendono il frame 30. Quattro processi indipendenti
per lato e risoluzione, sullo stesso adapter RTX 3090 e da checkout puliti,
danno scene-pass mediani BEFORE/AFTER di 0.775/2.009 ms a 1080p,
1.209/2.870 ms a 1440p e 2.254/4.845 ms a 4K. L'evidence conserva tutti i
campioni e min/max. La GPU timestamp readback può indicare il frame corrente o
quello precedente: ogni audit registra indice sorgente e age. Nei run AFTER
con timestamp del frame precedente il scene pass è circa 0.5 ms più alto a
1080p e 0.8 ms più alto a 1440p; perciò la mediana aggregata non è una misura
di un solo frame source. La capture finale verifica entrambi, senza dedurre FPS
dal solo scene pass. Non è stata cambiata la geometria o la shader architecture:
la regressione estrema non si ripete dopo il warm-up. Il costo residuo non è
attribuito a un singolo componente senza un'ablation separata; i tempi per
pass sono riportati nell'evidence.

## 6. Validazione

Gate progetto: `cargo fmt --all -- --check`, `cargo check --workspace
--all-targets`, `cargo clippy --workspace --all-targets -- -D warnings`,
`cargo test --workspace --all-targets`, `cargo build --workspace --release`,
`python -X utf8 -m unittest tools/env1_asset_pipeline/test_env1_assets.py`,
`python -X utf8 tools/env1_asset_pipeline/verify_env1_assets.py`, suite
`tools/visual_benchmark`. Test nuovi: partizione/determinismo della maschera
regioni, scale tile FFV1, tier di placement (ground-cover/hedgerow), contratti
GLB aggiornati (19 GLB = 1 aircraft + 6 asset × 3 LOD).

## 7. Evidence e verdict

`docs/validation/ffv1_flying_field/`: manifest hero 1080p/1440p/2160p e
`ffv1_evidence.json` (BEFORE/AFTER per risoluzione, receipt, audit, confronto
pixel a bande, metriche non disponibili `null`). Le PNG candidate NON sono
committate (regola di progetto): `committed_png: null` + path `tmp/` + SHA-256.
`visual_pass = null`: il verdict visivo è riservato alla review umana.
