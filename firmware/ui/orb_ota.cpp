/*
 * orb_ota: over-the-air firmware updates for the Orb.
 *
 * Ported from orb-csi-test/main/orb_ota.c — the proven pattern:
 * a minimal HTTP endpoint (port 8080) that receives a firmware binary
 * and writes it to the inactive OTA partition, then reboots into it.
 *
 *   curl -X POST --data-binary @firmware.bin http://<orb-ip>:8080/ota
 *   curl http://<orb-ip>:8080/ota_status   -> {"status":"idle|..."}
 *
 * Safety: only accepts a body while in IDLE state; validates the image
 * via esp_ota_end() before committing the boot partition. Does NOT
 * interfere with the gateway ingest path (port 9000 — separate server).
 */
#include <string.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_http_server.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_task_wdt.h"

#include "orb_ota.h"

static const char *TAG = "orb_ota";

#define OTA_HTTP_PORT 8080
#define OTA_RECV_BUF  4096   /* one flash sector per write: keeps the
                                interrupt-off window tiny (IWDT-safe) */

typedef enum {
    OTA_IDLE = 0,
    OTA_RECEIVING,
    OTA_SUCCESS,
    OTA_FAILED,
} ota_state_t;

static ota_state_t s_ota_state = OTA_IDLE;
static esp_ota_handle_t s_ota_handle = 0;
static const esp_partition_t *s_ota_part = NULL;
static size_t s_ota_received = 0;

static esp_err_t ota_reset(void)
{
    if (s_ota_handle) {
        esp_ota_abort(s_ota_handle);
        s_ota_handle = 0;
    }
    s_ota_part = NULL;
    s_ota_received = 0;
    s_ota_state = OTA_IDLE;
    return ESP_OK;
}

/* GET /ota_status — report current state. */
static esp_err_t ota_status_handler(httpd_req_t *req)
{
    const char *state = "idle";
    switch (s_ota_state) {
    case OTA_RECEIVING: state = "receiving"; break;
    case OTA_SUCCESS:   state = "success";   break;
    case OTA_FAILED:    state = "failed";    break;
    default:            state = "idle";      break;
    }
    char buf[128];
    int len = snprintf(buf, sizeof(buf),
                       "{\"status\":\"%s\",\"received\":%u}", state,
                       (unsigned)s_ota_received);
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, buf, len);
    return ESP_OK;
}

/* POST /ota — receive firmware binary. */
static esp_err_t ota_post_handler(httpd_req_t *req)
{
    if (s_ota_state == OTA_RECEIVING) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "ota already in progress");
        return ESP_FAIL;
    }
    if (s_ota_state == OTA_SUCCESS) {
        ota_reset(); /* allow re-flash after success */
    }

    /* Target the inactive OTA slot. */
    s_ota_part = esp_ota_get_next_update_partition(NULL);
    if (s_ota_part == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "no ota partition");
        return ESP_FAIL;
    }
    esp_err_t ret = esp_ota_begin(s_ota_part, OTA_SIZE_UNKNOWN,
                                  &s_ota_handle);
    if (ret != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "esp_ota_begin failed");
        return ESP_FAIL;
    }
    s_ota_state = OTA_RECEIVING;
    s_ota_received = 0;
    ESP_LOGI(TAG, "OTA start -> %s", s_ota_part->label);

    char *buf = (char *)malloc(OTA_RECV_BUF);
    if (!buf) {
        ota_reset();
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "oom");
        return ESP_FAIL;
    }

    int remaining = req->content_len;
    while (remaining > 0) {
        int n = httpd_req_recv(req, buf,
                               remaining < OTA_RECV_BUF ? remaining : OTA_RECV_BUF);
        if (n <= 0) {
            ESP_LOGE(TAG, "recv error %d", n);
            free(buf);
            ota_reset();
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                "recv failed");
            return ESP_FAIL;
        }
        ret = esp_ota_write(s_ota_handle, buf, n);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "ota write failed: %s", esp_err_to_name(ret));
            free(buf);
            ota_reset();
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                                "write failed");
            return ESP_FAIL;
        }
        s_ota_received += n;
        remaining -= n;
        /* Long flash writes can starve the task watchdog — feed it
         * every chunk so the 1MB+ transfer doesn't trip a WDT reset. */
        esp_err_t wdt_ret = esp_task_wdt_reset();
        if (wdt_ret != ESP_OK && wdt_ret != ESP_ERR_NOT_FOUND) {
            ESP_LOGW(TAG, "wdt feed failed: %s", esp_err_to_name(wdt_ret));
        }
    }
    free(buf);

    ret = esp_ota_end(s_ota_handle);
    s_ota_handle = 0;
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %s", esp_err_to_name(ret));
        s_ota_state = OTA_FAILED;
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "image invalid");
        return ESP_FAIL;
    }

    ret = esp_ota_set_boot_partition(s_ota_part);
    if (ret != ESP_OK) {
        s_ota_state = OTA_FAILED;
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "set boot failed");
        return ESP_FAIL;
    }

    s_ota_state = OTA_SUCCESS;
    ESP_LOGI(TAG, "OTA OK: %u bytes -> %s. Rebooting in 1s.",
             (unsigned)s_ota_received, s_ota_part->label);
    httpd_resp_sendstr(req, "{\"ok\":true,\"reboot\":true}");
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
    return ESP_OK;
}

esp_err_t orb_ota_start(void)
{
    httpd_handle_t server = NULL;
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.server_port = OTA_HTTP_PORT;
    cfg.lru_purge_enable = true;

    if (httpd_start(&server, &cfg) != ESP_OK) {
        ESP_LOGE(TAG, "httpd start failed");
        return ESP_FAIL;
    }
    httpd_uri_t status_uri = {
        .uri = "/ota_status", .method = HTTP_GET,
        .handler = ota_status_handler, .user_ctx = NULL,
    };
    httpd_uri_t post_uri = {
        .uri = "/ota", .method = HTTP_POST,
        .handler = ota_post_handler, .user_ctx = NULL,
    };
    httpd_register_uri_handler(server, &status_uri);
    httpd_register_uri_handler(server, &post_uri);
    ESP_LOGI(TAG, "OTA server on :%d (POST /ota, GET /ota_status)", OTA_HTTP_PORT);
    return ESP_OK;
}
