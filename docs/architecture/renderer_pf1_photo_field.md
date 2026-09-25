# PF1 — Photo Field: prima path fotografica reale

## 1. Obiettivo

PF1 introduce la prima path di presentazione FOTOGRAFICA del simulatore: con il
pilota fisso, l'ambiente lontano non è più ricostruito con geometria 3D a basso
dettaglio ma è una fotografia panoramica reale a 360°, mentre l'aereo resta
completamente 3D, simulato fisicamente e illuminato dinamicamente.

Contratto di prodotto: **la posizione dell'occhio del pilota non trasla mai**.
La camera può ruotare/inclinare per seguire l'aereo e può cambiare FOV (il
panorama è sferico), ma il suo occhio in world-space è quello rilevato dal
manifest. Con l'aereo rimosso, quasi tutta la scena visibile è la fotografia.

Base autoritativa: `integration/current` == `integration/render-v2` ==
`86552e1ba4f87477290fd18291dd355c5ef9abac` (precondition verificata: entrambi i
ref remoti uguali allo SHA richiesto, set rami remoti = main + le due
integration, worktree pulito a parte gli untracked noti). Branch di lavoro:
`work/visual/photo-field-v1`, creato esattamente da quello SHA.

## 2. Sorgente fotografica e provenance

Sorgente: HDRI Poly Haven `meadow` (CC0, autore Sergej Majboroda, mezzogiorno,
parzialmente nuvoloso, basso contrasto), la stessa misura di riferimento usata
dal look development FFV1. La cache sorgente originale NON era più disponibile
nel repository (nessun `.hdr`/`.exr` commitato, solo citazioni nei commenti),
quindi è stata riacquisita **solo** dall'API ufficiale Poly Haven
(`api.polyhaven.com/info|files/meadow`) e verificata contro size+MD5 pubblicati:

- `meadow_8k.hdr` 108 457 266 byte, md5 `c1e25ad9fb1aba9ebc8babb952292727`,
  sha256 `9d947c59de8464a04fd22ecef8ed548750e2bbb07072f44ef61b0fbb2d738c9d`,
  Radiance `32-bit_rle_rgbe`, 8192x4096 (quindi NESSUN resampling: la derivata
  runtime ha esattamente la risoluzione richiesta dal brief);
- `meadow_tonemapped.jpg` 57 575 185 byte, md5 `9508d4679483dfc17466ff4c9ada44d9`
  — usata SOLO come riferimento fotografico per calibrare l'esposizione, mai come
  sorgente runtime;
- le backplate NEF/JPG NON sono state usate: ricucirle sarebbe fotogrammetria,
  esplicitamente fuori scope.

La sorgente HDR resta OFFLINE nella cache gitignorata `tmp/`; in repository entra
solo la derivata runtime display-referred (JPEG sRGB 8192x4096) più il GLB dei
proxy: il brief vieta di commitare la sorgente 16K/EXR/HDR "per comodità" e non
introduce KTX2/virtual texturing/streaming.

## 3. Pipeline immagine: la fotografia NON passa dal tone mapper

Vincolo del brief: non dare in pasto una foto display-referred alla chain HDR +
Khronos PBR Neutral accettando una foto visibilmente alterata, e al tempo stesso
non toccare la chain HDR dell'aereo.

La soluzione è che la derivata runtime viene prodotta OFFLINE applicando alla
radianza HDR **la stessa curva che il renderer usa per l'aereo**
(`khronos_pbr_neutral` in `shader.wgsl`, portata in Python verbatim) a esposizione
0.0 EV, poi codificata sRGB:

    display_linear = khronos_pbr_neutral(radianza_hdr * 1.0)
    byte_srgb     = srgb_encode(display_linear)

Così fotografia e aereo condividono UNA curva e UNA relazione di esposizione per
costruzione. Il fit contro la rendition fotografica ufficiale del provider dà
MAE 0.078 sRGB e correlazione 0.972 a k=1.0 (ottimo raffinato k=1.179, +0.238 EV,
MAE 0.072): si è scelto k=1.0 perché coincide con l'esposizione pinned del
renderer e il guadagno del fit è dentro il rumore di uno scalare. Residui per
banda: cielo 0.578 vs 0.559, ground 0.248 vs 0.306, zenith clip 0.999 vs 0.926
(il knee Khronos comprime lo zenit più della curva del provider: noto e accettato,
coerente con quanto FFV1 aveva già osservato sul cielo).

A runtime il panorama è una texture **sRGB**: il sampling la riporta lineare e la
surface sRGB la ricodifica, quindi la foto round-tripa inalterata. Il composite
finale (nuovo entry point `fs_postprocess_photo`) è esattamente il diagramma del
brief:

    panorama fotografico * maschera ombra  +  aereo tone-mappato  ->  sRGB finale

con `coverage = scene_depth < 1.0 && scene_depth < proxy_depth`. La copertura NON
usa alpha (nel renderer l'alpha HDR è un flag costante 1.0 e non esiste MRT né
dual-source blending): deriva da un confronto di profondità ben definito, come
il brief richiede ("small explicit presentation-only mask/resource rather than
relying on undefined alpha semantics").

## 4. Campionamento equirectangolare

Fullscreen, dal raggio della camera via inverse view-projection (riusa
`view_direction_from_clip`, già usata da `fs_sky`/`fs_sky_v2`):

    u = fract(azimuth / 2pi),  azimuth  = atan2(dir.z, dir.x)  (spazio panorama)
    v = clamp(0.5 - elevation / pi),  elevation = asin(dir.y),  v=0 = prima riga = zenite

La calibrazione `panorama_yaw_deg`/`panorama_pitch_deg` è una rotazione rigida
(yaw attorno a +Y, poi pitch attorno alla +X ruotata) della direzione world nello
spazio panorama: non è un offset di UV, quindi resta corretta per ogni azimuth.
Il wrap orizzontale è seamless (u mod 1 + sampler Repeat su U, mip chain con wrap
orizzontale); il verticale clampa ai poli. La convenzione è specchiata in Rust
(`photo_field::equirect_uv_from_direction`) con test su direzioni cardinali,
seam, poli e sul texel del sole misurato.

Nota: `fs_sky_v2` usa `v = elevation/pi + 0.5` perché campiona la Sky-View LUT
generata dal codice atmosfera, non un'immagine equirectangolare: le due
convenzioni sono volutamente distinte e documentate.

## 5. Proxy di profondità invisibili

GLB 2.0 commitato (`photo_field_depth.glb`), geometria volutamente grezza
(piano terra, anello di volumi per la tree line, box per gli edifici fotografati,
tronchi vicini): niente foglie, cortecce, materiali. Pass depth-only con
`color_attachments: &[]` e **nessun fragment stage**: i proxy non possono
scrivere colore in nessun target. Usano la view-projection della CAMERA (depth
camera-space), NON le cascade shadow (semantica light-space diversa, vietato dal
brief). Upload una volta all'inizializzazione, zero allocazioni per frame.

L'occlusione nasce nel composite: il depth di scena contiene solo geometria 3D
reale (l'aereo), il depth proxy solo i sostituti invisibili; l'aereo è visibile
esattamente dove è più vicino di ciò che la fotografia rappresenta in quel pixel.
Da qui il test di accettazione obbligatorio: aereo più vicino del proxy → visibile
davanti all'oggetto fotografato; più lontano → nascosto dietro.

## 6. Ombra dinamica dell'aereo sulla fotografia

La foto contiene già le sue ombre ambientali statiche: non ne vengono generate di
nuove per alberi/edifici. Solo l'aereo richiede integrazione. Un pass
presentation-only disegna il solo proxy di terra in un target maschera
(`R16Float`) scrivendo `1 - shadow_strength * (1 - visibility)` con i dati shadow
cascaded ESISTENTI dell'aereo (`directional_shadow_visibility`), depth-testato
contro il depth proxy senza scrivere depth: un proxy più vicino (un albero
fotografato) sopprime l'ombra a terra dietro di sé. Nessuna geometria di terra
visibile, nessun piano PBR al posto dell'erba, nessun blob nero: è una
modulazione moltiplicativa della fotografia.

## 7. Calibrazione luce: derivato fisicamente vs calibrato a mano

Derivato fisicamente dalla sorgente HDR (decode Radiance RGBE scritto ad hoc,
Pillow 12.3 non legge `.hdr`):
- direzione del sole: disco solare isolato per connected components a 0.8*max →
  longitudine 153.027°, elevazione +68.936° (cross-check indipendente: l'analisi
  offline FFV1 registrava "~69°");
- radianze: cielo 1.2509 (doc FFV1: 1.26), zenit 3.15, suolo 0.164 (doc: 0.19),
  sky/ground 7.6;
- cromatica del disco [1.0, 0.672, 0.533] (registrata ma NON usata come colore
  della luce diretta: il disco è clip-pato, `evs_cap` 15).

Calibrato a mano / ereditato:
- `sun_rgb = [1.0, 0.95, 0.85]` e `sun_intensity = 2.6`: ereditati dalla
  calibrazione FFV1, che era già misurata contro questo stesso HDRI
  (`continental_summer_haze` + `V2_FIELD_SUN_INTENSITY`);
- distanze dei proxy: il box del garage in mattoni è derivato semi-fisicamente
  (box angolare azimut [131.75,169.94]°, elevazione [-3.65,+7.51]°; con altezza
  camera fotografica assunta 1.6 m la base a -3.65° implica 25.1 m e la sommità
  +7.51° implica 4.91 m di altezza, 16.7 m di larghezza); gli altri edifici e i
  tronchi sono stime visuali documentate;
- posizione dell'occhio `[15.321, 1.6, -12.856]`: scelta perché l'aereo (che nasce
  all'origine render) stia a 20 m dall'occhio lungo l'azimut 140°, cioè davanti al
  proxy del garage a 25.1 m e dentro l'anello di tree line a 30 m: rende
  esprimibile il test di occlusione con l'aereo vero (parcheggiato = caso A,
  volato oltre l'anello = caso B). L'azimut 140° tiene la linea di vista
  dell'aereo a 12° dal tronco fotografato a 152°, il cui proxy a 5 m altrimenti
  nasconderebbe completamente l'aereo parcheggiato, e resta dentro lo span
  misurato del garage (131.75-169.94°).

Nessun secondo motore di illuminazione: si riusa `SunState`/ambiente fisico
esistente; nessun IBL runtime nuovo dal panorama (fuori scope per il brief).

## 8. Contratto camera fissa e rejection

`PhotoField` opera solo con `CameraConfig::Pilot`: `fixed_pilot_eye` rifiuta
`Chase` (`ChaseRejected`) e qualsiasi occhio diverso da quello del manifest
(`PilotPositionMismatch`), senza fallback silenziosi. Il guarantee "l'occhio non
trasla" è già in `PilotCamera` (l'argomento pose è inutilizzato) ed è coperto da
test su molte posizioni/atteggiamenti dell'aereo. L'app defaulta `--pilot-position`
al valore del manifest quando non esplicitato con `--scenery photo-field`.

## 9. Invarianza Full3D e performance

PF1 è additivo: `SceneryPreset::PhotoField` non genera terreno materiale, scenery
o vegetazione (i gate sono esplicitati, non lasciati a confronti `==` che
salterebbero silenziosi). Nessun nuovo `PassId`: i due pass PF1 (depth proxy e
maschera ombra) vivono dentro il blocco profilato `Scene` e sono documentati come
attribuiti a quel pass; il grafo, i conteggi `[3,4,1]` e l'audit restano
byte-identici per Full3D. Nel pass scena PF1 non vengono disegnati cielo né
terreno: resta solo l'aereo, quindi il costo per frame è strutturalmente inferiore
al campo Full3D denso. Tutte le risorse PF1 (texture panorama + mip, depth proxy,
maschera, pipeline, bind group) sono create all'inizializzazione o al resize, mai
per frame.

## 10. Validazione

`docs/validation/pf1_photo_field/`: manifest hero 1080p/1440p/2160p (runner VIS0
esistente), evidence JSON con receipt/audit/hash, misure per risoluzione dopo
warm-up (frame 30), evidenza del test di occlusione A/B. Le PNG candidate NON sono
committate (regola di progetto): `committed_png: null` + path `tmp/` + SHA-256.
`visual_pass = null`: il verdetto visivo è riservato alla review umana. Metriche
non disponibili nel repository (VRAM, FPS/percentili, draw-call totali) registrate
`null` con motivazione, mai stimate; i `gpu_duration_ns` restano single-sample e
va dichiarato.

### PF1 occlusion evidence closure

For each capture with a runtime receipt, the app also writes
`runtime_capture_pose.json` beside it. This separate sidecar records the aircraft
render position submitted for the captured frame, the frame number, framebuffer
extent, and PNG SHA256. The strict RuntimeCaptureReceipt 1.0.0 contract is unchanged.
`tools/measure_pf1_photo_field.py` verifies those values against the receipt and
PNG before recording the NEAR and FAR aircraft positions and pilot distances in
`pf1_evidence.json`. Both cases use `pf1_tree_ring`, with a representative 30 m
radius and a conservative 5.5 m radial tolerance (3 m jitter plus 2.5 m half
depth). The numerical check establishes depth order; visual review remains
separate and `visual_pass` remains `null`.
