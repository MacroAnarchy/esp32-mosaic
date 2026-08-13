#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Start the OTA HTTP server (port 8080).
 *   POST /ota         with a firmware binary -> writes inactive slot, reboots
 *   GET  /ota_status  -> {"status":"idle|receiving|success|failed","received":N}
 *
 * This makes the orb flashable without USB (mobile operation). Coexists
 * with the gateway reporting (port 9000 — the orb POSTs to the gateway,
 * this server receives firmware pushes).
 */
esp_err_t orb_ota_start(void);

#ifdef __cplusplus
}
#endif
