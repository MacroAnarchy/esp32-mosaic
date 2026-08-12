# Mosaic Research — External Knowledge Feed

Continuity file for the Mosaic research agent. Append-only.
Every run: pick open questions → web research → verify claims + licenses → append entry.

---

## 2026-08-12 — BLE MAC rotation association / entity resolution (the stream-bind problem)

- QUESTION: The dev's collapse_chains/stream-bind/lockstep machinery binds rotating BLE
  MACs by RSSI-level continuity + time windows (|Δavg| ≤ 4-8dB, ~10 min gap, never bind
  coexisting). What does the literature/industry use for the SAME problem (associating
  randomized MACs of one device)? Is the approach validated? What's the next level?

- FINDINGS:
  - **Jouans et al. 2021, "Associating the Randomized Bluetooth MAC Addresses of a
    Device" (Inria, CCNC)** — https://inria.hal.science/hal-03045555 (open access PDF)
    — The closest published formulation of what Mosaic does. Core insight: MAC swap
    events are RARE (BLE spec: ≥15 min per address), so at any moment only a handful
    of devices swap → a WEAK identifier suffices. Method: an appearing MAC M3 pairs
    with a disappeared MAC M1 only if M3 appears within [M1_end, M1_end + d_swap]
    (measured d_swap ≈ 4 s for iPhone 7S), then solves a linear assignment problem
    using inter-packet timing (T_int) as the characterizing identifier. Results:
    <5% pair-wise mismatches in controlled setup. Also documents: only 4.5-8.4% of
    devices still use non-changing MACs; Apple "carry-over" key-identifier leaks
    affect only 0.8-2.8% of Apple devices (too narrow for general association).
    NO code released — method is fully described, implementable.
  - **McMatcher (Boussad et al. 2024, ICCE)** — https://eprints.whiterose.ac.uk/id/eprint/213511/
    (accepted-version PDF, free to read) — THE upgrade path. Symbolic Aggregate
    approXimation (SAX) of the RSSI time series per MAC → characterizing vector
    (embeds BOTH temporal sampling pattern AND signal strength) → cosine similarity
    matching. 100% accuracy matching 92 MACs from 16 smartphones inside a 332-MAC
    dataset, NO training, 230 ms for 18 MACs — realtime-capable. Works on pure RSSI
    (exactly Mosaic's data). Its related-work review is gold: Akiyama et al. 2021
    (time-diff threshold + avg-RSSI-diff threshold + nearest candidate = almost
    verbatim the dev's current greedy level-coherent stream bind!), Gagnon et al.
    2023 (RSSI-histogram ML fingerprinting — needs training). Verdict on the dev's
    approach: it is the Akiyama/Jouans family — VALIDATED, with McMatcher as the
    strictly-better generalization once streams get long enough for SAX vectors.
    Caveat: SAX sampling-pattern feature needs dense scans; a ~20s node cadence may
    be too sparse for the T_int feature (Jouans notes T_int 20ms-10.24s needs a fast
    sniffer) — the RSSI-level part works at any cadence.
  - **"Breaking BLE MAC Address Randomization with Allowlist" (ACM 2025)** —
    https://dl.acm.org/doi/10.1145/3744559 — attack/behavioral research: the filter
    accept list creates a tracking side channel (device responds only to allowlisted
    peer MACs); re-randomization happens at fixed intervals T_r → timing side channel.
    Why it matters for Mosaic: documents REAL re-randomization intervals (~15 min for
    RPA) and the RPA/IRK mechanics behind what nodes see. M3-relevant (adversarial
    tracking of visitors' phones is possible via these side channels — and so is
    misreading their absence/presence without knowing them).
  - **MAC de-randomization for WiFi PROBES: two-stage clustering (Baccichet et al.
    2024, PoliMi)** — https://arxiv.org/abs/2408.01578 (CC BY 4.0, dataset public at
    github.com/GiovanniBaccichet/ProbingPatternsDataset) — for the probes channel
    (currently near-silent, LOW issue): IEs alone are identical across same-model
    devices → use multi-channel time-frequency emission pattern + IE fingerprinting
    in two-stage clustering. Only the DATASET is public, no code. Relevant when the
    probe picture gets richer; also confirms why Mosaic's probes lens is a real
    identity channel (probe emission timing is device-unique).
  - **Juniper Mist, "BLE and MAC Randomization"** — https://www.mist.com/documentation/ble-mac-randomization/
    — industry practice: WiFi (connected + unconnected) + BLE mixture is THE way
    vendors de-anonymize rotating BLE; random-address advertising is the norm. Direct
    industry validation of Mosaic's WiFi+BLE cross-stream architecture principle.

- VERDICT: REAL — multiple independent published methods (Jouans 2021, Akiyama 2021,
  McMatcher 2024) solve exactly the stream-bind problem; the RSSI-gate + time-window
  approach is a recognized, published formulation (Akiyama family).

- IMPLEMENTABLE:
  1. McMatcher-style scoring as the NEXT tier of collapse_chains: after the greedy
     level-coherent bind, re-score candidate pairs with SAX/cosine over the RSSI
     series (public-domain algorithm, small Python; no external deps) — replaces the
     single |Δavg| gate with a whole-signature match. Paper is enough to build from.
  2. Jouans' swap-window + assignment-problem refinement: when multiple candidates
     compete for one ended stream, solve a linear assignment (Hungarian) instead of
     greedy first-match — ~30 lines with scipy or hand-rolled for small N. This
     directly addresses the ambiguous-pairing failure (67/77 shared-predecessor
     misbind).
  3. Confirm the swap-gap expectation: BLE spec ≥15 min per address, measured
     d_swap ≈ 4 s (iPhone 7S) — a 10-min MAX_STREAM_GAP is on the right side
     but should never exceed the re-randomization floor; a >15-min same-level
     "rotation" is probably a different device (or a dual-identity phone — two
     parallel streams each rotate independently).

- CONFIDENCE: HIGH on method validity (3 independent papers + industry doc agree);
  MEDIUM on McMatcher's exact accuracy numbers transferring to a sparse node scan
  cadence (their data was dense, realtime sniffing).

---

## 2026-08-12 — CSI sensing: what's real vs paper-dream (TIER 1-4 ladder)

- QUESTION: Queue item 3 (CSI bridge) + HIGH issue (body channel dead). Beyond
  RuView + esp_wifi_sensing (already known), what ESP32 CSI implementations exist,
  are they maintained, what do real users report, and which tiers are proven?

- FINDINGS:
  - **nickbild/csi_hr — "Measuring Heart Rate Using Wi-Fi"** —
    https://github.com/nickbild/csi_hr — an independent reproduction of the Pulse-Fi
    paper (heart rate via CSI, LSTM, 100-packet sliding window, 192 features = 64
    subcarriers × 3 antennas): full pipeline in ONE python script
    (read_and_process_csi.py) + train.py + arduino_hr (MAX30102 ground truth).
    Active (commits Sep 2025), 136★. ⚠️ NO LICENSE FILE → legally all-rights-reserved;
    read for METHOD, don't port code into MIT Mosaic. Hardware note: needs a TX/RX
    ESP32 PAIR with the person BETWEEN them (csi_send/csi_recv from esp-csi) — a
    different geometry than Mosaic's single-node + router. IMPORTANT for TIER 2
    expectations: the proven vitals setups are bistatic (two nodes), not monostatic.
  - **ESPectre (francescopace)** — https://github.com/francescopace/espectre +
    https://community.home-assistant.io/t/espectre-wi-fi-motion-detection-for-home-assistant/961251
    — ESPHome external component for CSI motion detection: binary sensor
    (motion_detected) + movement_score 0-100 + live threshold entity; automatic
    subcarrier calibration (NBVI), zero config, works through walls; tested on
    S3/C6/C3; thread active Dec 2025→May 2026, 8.3k views, very positive user
    reception. ⚠️ LICENSE VERIFIED: GPL-3.0 → READ-ONLY reference for Mosaic (MIT).
    Still valuable: proves ESP32-class CSI motion is a solved, productized problem
    (the NBVI auto-calibration idea is directly portable as a concept for Mosaic's
    TIER 1 baseline learning). Stationary presence is on its roadmap (Q1 2026 per HN
    coverage) — i.e. even the best CSI-motion projects don't claim stationary-presence
    yet; that matches Mosaic's honest TIER 1 = "motion only".
  - **RuView / wifi-densepose family (euaziel fork, MIT, verified)** —
    https://github.com/euaziel/WiFi-CSI-Human-Pose-Detection — the fork's README
    confirms the vital-signs pipeline details (breathing bandpass 0.1-0.5 Hz → FFT,
    HR 0.8-2.0 Hz → FFT, 6-30 BPM / 40-120 BPM ranges) and that ESP32-S3 streaming is
    the primary hardware path. Note: fps/marketing numbers are pipeline benchmarks,
    NOT accuracy claims — accuracy claims remain unverified until prove.sh runs on
    real S3 hardware (as the project doc already says).
  - **Survey anchor: "A survey on vital signs monitoring based on Wi-Fi CSI data"
    (PMC9375645)** — https://pmc.ncbi.nlm.nih.gov/articles/PMC9375645/ — the
    literature summary for "is it proven": lab studies reach ~96.6% breathing /
    ~94.7% HR accuracy — but under controlled, static, single-subject conditions
    with research NICs. Real-world ESP32 reports are more mixed: Hackster's
    ESP-CSI review (https://www.hackster.io/limengdu0117/esp-csi-diy-wifi-human-presence-detection-f80508)
    verdict: "an incredible hack, but too finicky and power-hungry to replace radar".
  - **Ecosystem status**: ESPHome STILL has no native CSI sensor (feature request
    open: https://github.com/esphome/feature-requests/issues/1822) — ESPectre fills
    it via external component. Espressif's esp_wifi_sensing (already known to the
    team) remains the canonical motion primitive; ESP-CSI tooling unchanged.

- VERDICT: REAL (TIER 1 motion: multiple independent maintained implementations,
  positive user reports) · PLAUSIBLE (TIER 2 vitals: proven in lab/bistatic setups,
  reproduced by csi_hr, but finicky monostatic and not accuracy-verified on S3 yet)
  · CLAIMED (TIER 3/4: activity classification and dense pose — demo-level).

- IMPLEMENTABLE:
  1. The CSI bridge spec should target ESPectre-style ESPHome component or RuView's
     esp32-csi-node UDP; for Mosaic's MIT repo, port the CONCEPTS (NBVI
     auto-calibration, movement_score 0-100 smoothing) not the code.
  2. TIER 2 (vitals) hardware plan: prefer a TWO-NODE bistatic geometry (TX pair +
     RX pair across the room) — that is the only geometry with published,
     reproducible results (Pulse-Fi, csi_hr). A single S3 + home router may do
     breathing (coarse) but heart-rate claims need the bistatic setup.
  3. TIER 1 fallback that works TODAY on existing hardware: esp_wifi_sensing motion
     events — the bridge task is unchanged.

- CONFIDENCE: HIGH on the ecosystem map (everything verified by direct extraction
  today: licenses, activity dates, user threads); MEDIUM on TIER-2 transferability —
  "lab-proven, apartment-finicky" is the honest summary.

---

## 2026-08-12 — BLE classification + FindMy detection improvements

- QUESTION: The labeler parses company_id + service_uuids + broadcast names and has a
  findmy device class. What's missing? Where do we get deeper device identification,
  and is there a STRONG identity signal for Apple's rotating MACs (the dual-stream
  stationary-phone case)?

- FINDINGS:
  - **reelyactive advlib-ble-manufacturers (MIT, VERIFIED)** —
    https://github.com/reelyactive/advlib-ble-manufacturers — zero-dependency Node.js
    decoders for vendor manufacturer-data: Apple (incl. iBeacon), Nordic, Ruuvi,
    Minew, EnOcean, Wiliot, MOKO, ELA... part of Pareto Anywhere (open-source IoT
    middleware, MIT family). Its apple.js decodes Apple's manufacturer data
    structures — the reference for parsing the Apple AD sub-types Mosaic sees from
    a stationary iPhone. The full advlib-ble (https://github.com/reelyactive/advlib-ble)
    is the complete AD parser. MIT → safe to port/translate the decoding tables.
    Practical value: turns opaque company_id+hex blobs into structured deviceIds
    (e.g. iBeacon UUID/major/minor, Ruuvi sensor payloads — ThermoBeacon-class
    devices could get real decoders instead of name-string matching).
  - **Apple "carry-over" identifiers — the STRONG binder for rotating Apple MACs** —
    Celosia & Cunche, "Saving private addresses" (MobiQuitous 2019) + Martin et al.,
    "Handoff all your privacy" (PoPETs 2019, open access:
    https://petsymposium.org/popets/2019/popets-2019-0045.php) — documented finding:
    Apple devices carry CONSTANT key identifiers inside the advertising DATA FIELD
    (nearby-info/continuity keys) that sometimes persist across a MAC rotation
    (mis-synchronization between MAC change and key change) — hence "carry-over".
    For Mosaic this is the missing strong identity: if the AD parser keeps the Apple
    key bytes (28-byte key in Apple adv type 0x0019/continuity types), two rotating
    MACs sharing a key = SAME device with near-certainty — far stronger than RSSI
    continuity. Jouans 2021 measured only 0.8-2.8% of Apple devices leak via
    carry-over, so it's a HIGH-PRECISION LOW-RECALL binder: use it to CONFIRM
    bindings (PROBABLE→CONFIRMED path!), not to find them.
  - **FindMy adversarial side (M3 threat model)**: Bräunlein's "Find You" tag
    (https://www.hackster.io/news/fabian-braunlein-s-esp32-powered-find-you-tag-bypasses-apple-s-airtag-anti-stalking-protections-0f2c9ee7da74)
    — ESP32 OpenHaystack firmware that bypasses AirTag anti-stalking alerts; and
    USENIX Sec 2025 "Tracking You from a Thousand Miles Away" (FindMy network
    abuse, https://www.usenix.org/conference/usenixsecurity25/presentation/chen-junming).
    Meaning: FindMy-network devices in the apartment are not just Apple accessories —
    a $5 ESP32 flashed with FindYou is an untagged, alert-free tracker. Mosaic's
    findmy class already catches its ADV format; worth noting in the M3 threat
    review, and OpenHaystack (MIT) firmware is a legitimate way to BUILD such tags
    for testing Mosaic's detection.
  - **SimpleBLE BLE Advertisement Decoder** (https://simpleble.org/tools/ble-advertisement-decoder)
    — handy dev tool for hex AD payloads, not a library.

- VERDICT: REAL — advlib decoders are MIT and maintained; the Apple key-identifier
  carry-over is a documented, peer-reviewed phenomenon with direct application to
  Mosaic's identity problem.

- IMPLEMENTABLE:
  1. Capture the Apple manufacturer-data KEY BYTES (not just company_id/service_uuids)
     in the gateway's AD parse; use key equality as a CONFIRMING binder for Apple
     MAC rotations — the natural trigger for the "PROBABLE → CONFIRMED" path.
     This is the WiFi-join alternative the pipeline already has the machinery for.
  2. Translate advlib's apple.js + ruuvi.js decoding tables (MIT) into the Python
     labeler for structured device identification of the known Apple/Ruuvi payloads.
  3. Add FindYou/OpenHaystack-style tags to the M3 test plan: an untagged tracker
     MUST appear as an unknown findmy-class device at -30..-40 dBm — the exact M3
     anomaly signature.

- CONFIDENCE: HIGH on advlib (license verified, active project); HIGH on carry-over
  existence (two peer-reviewed works); MEDIUM on its utility per-device (low recall —
  it's a confirmer, not a detector).

---

## 2026-08-12 — Presence detection: proximity, zone/room estimation from RSSI without beacons

- QUESTION: Stage 2 (two nodes) wants zone/room-level presence from RSSI. What's the
  proven open approach for multi-node BLE presence, and what's the industry view on
  RSSI-based proximity limits?

- FINDINGS:
  - **ESPresence (ESPresense/ESPresense)** — https://github.com/ESPresense/ESPresense
    — THE de-facto open-source ESP32 BLE room-presence system (1.5k★, 1120 commits,
    last commit Aug 2026 — very active; HA mqtt_room + own companion for indoor
    positioning). Approach: per-node ESP32 BLE sniffers → MQTT → server-side room
    FINGERPRINTS (calibrated per-device RSSI per room) → room assignment by best
    fingerprint match. This is exactly the "zone estimation from RSSI" playbook for
    Mosaic's Stage 2, including per-node calibration. ⚠️ LICENSE VERIFIED: AGPL-3.0
    → do NOT port code into MIT Mosaic; use as architectural reference (fingerprint
    tables, calibration flow, MQTT topology).
  - **Juniper Mist docs** (see first entry) — industry: RSSI proximity for zone
    detection is standard practice; sub-room precision requires multiple APs/
    receivers + calibration; MAC randomization is the main analytics killer —
    exactly why Mosaic's entity-binding matters for presence quality.
  - **reelyActive Pareto Anywhere** (https://github.com/reelyactive/pareto-anywhere,
    MIT family) — open-source ambient-IoT middleware with room-level positioning from
    multiple BLE receivers (RSSI-based), plus the advlib decoder stack above. Heavier
    (Node.js middleware), but the positioning math is MIT and battle-tested in
    production installs.
  - **Honest limit**: no open project claims meter-level single-node localization
    from BLE RSSI; the practical ceiling is room/zone-level with 2+ receivers and
    calibration. Mosaic's own tier design (STRONG/MID/EDGE) is consistent with this.

- VERDICT: REAL — room/zone-level presence from multi-node BLE RSSI is a solved,
  productized problem (ESPresence proves it in HA for years); single-node
  sub-room localization is not.

- IMPLEMENTABLE:
  1. Stage 2 spec: mirror ESPresence's architecture in Mosaic's own stack — per-node
     RSSI fingerprints per device, learned calibration (Mosaic's places machinery
     already does this for BSSIDs; extend to device-level zone fingerprints).
  2. Keep AGPL/GPL projects out of the repo; mine their docs/architecture for
     concepts only.
  3. The ACM allowlist paper's T_r timing observation applies here too: a device's
     fixed re-randomization interval is itself a presence-usable timing signature.

- CONFIDENCE: HIGH (license + activity verified today; multi-year production use in
  HA community).

---

## NEXT RUN IDEAS (unresearched open questions)

1. Co-occurrence graphs / composite entity binding (Level 1-2 entity stack):
   graph community-detection literature (Louvain on device co-occurrence),
   "co-presence networks" papers.
2. Radio coexistence: channel hopping/time-slicing while staying connected —
   ESP32 sniff-while-connected techniques, esp_wifi internals.
3. Neighbor-bleed per-device baseline: RSSI baseline estimation, multi-path/
   antenna-gain literature for cheap nodes.
4. ARP/router identity feed: open-source router/ARP monitoring (arpwatch lineage,
   OpenWrt arpwatch, ntopng) — the WiFi-join trigger.
5. Fall detection specifically (CSI tier 3): what's published with code?
6. The ESPHome CSI feature request thread — track ESPectre's stationary-presence
   progress as the field's state of the art.
