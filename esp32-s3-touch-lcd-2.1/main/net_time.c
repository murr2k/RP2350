#include "net_time.h"

#include <string.h>
#include <sys/time.h>
#include <time.h>

#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "rtc_pcf85063.h"

/* Kept out of the repository, which is public. Without it the firmware still
 * builds and simply reports that there is no network to ask. */
#if __has_include("wifi_secrets.h")
#include "wifi_secrets.h"
#endif

/* The absence of credentials is a runtime state, not a compile time one: the
 * code that would use them still has to compile so that a fresh clone builds.
 * These stand in for that, and are never read, because net_time_start() sees
 * the flag below and never starts the task that would. */
#if defined(WIFI_SSID)
#define HAVE_NETWORK 1
#else
#define HAVE_NETWORK 0
#define WIFI_SSID ""
#define WIFI_PASS ""
#endif

static const char *TAG = "net_time";

/* US Pacific, matching the machine this was built on. Change this one line for
 * somewhere else: it is a POSIX TZ string, so the two rules on the end are when
 * summer time starts and ends and are what keep the clock right across the
 * changeover without anyone touching it. */
#define TIME_ZONE       "PST8PDT,M3.2.0,M11.1.0"

#define SNTP_SERVER     "pool.ntp.org"
#define CONNECT_TRIES   3
#define CONNECT_WAIT_MS 15000
#define SYNC_WAIT_MS    20000
#define TASK_STACK      4096
#define TASK_PRIORITY   3

/* Core 0, alongside the render loop. Core 1 carries the panel's interrupt and
 * the sensor task, and neither wants company. */
#define TASK_CORE       0

#define BIT_GOT_IP      BIT0
#define BIT_FAILED      BIT1

static volatile net_time_state_t s_state;
static EventGroupHandle_t s_events;
static int s_attempts;
static esp_netif_t *s_netif;
static esp_event_handler_instance_t s_wifi_handler;
static esp_event_handler_instance_t s_ip_handler;
static uint32_t s_address;

/* Runs on the event loop task, whose stack is small and shared with the IDF's
 * own handlers. Anything said here is said with printf, and this build has the
 * full version of it rather than the cut down one, which was enough to run that
 * stack out and take the whole system down with a corrupt mutex inside the
 * event loop. So: record and signal, nothing else. The task does the talking. */
static void on_wifi_event(void *arg, esp_event_base_t base, int32_t id, void *data)
{
    (void)arg;

    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        if (++s_attempts < CONNECT_TRIES) {
            esp_wifi_connect();
        } else {
            xEventGroupSetBits(s_events, BIT_FAILED);
        }
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *got = (const ip_event_got_ip_t *)data;
        s_address = got->ip_info.ip.addr;
        xEventGroupSetBits(s_events, BIT_GOT_IP);
    }
}

static bool bring_up_radio(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* WiFi keeps its calibration here, so a stale partition has to go. */
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "nvs: %s", esp_err_to_name(err));
        return false;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    s_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t init = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&init));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, on_wifi_event, NULL, &s_wifi_handler));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, on_wifi_event, NULL, &s_ip_handler));

    wifi_config_t cfg = {0};
    strlcpy((char *)cfg.sta.ssid, WIFI_SSID, sizeof(cfg.sta.ssid));
    strlcpy((char *)cfg.sta.password, WIFI_PASS, sizeof(cfg.sta.password));
    cfg.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &cfg));

    /* Nothing here needs the radio between boots, and the sleep the driver
     * would otherwise do adds latency to the one exchange that matters. */
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    ESP_ERROR_CHECK(esp_wifi_start());
    return true;
}

static void take_down_radio(void)
{
    /* Handlers go first. Disconnecting raises a disconnect event, and the
     * handler's answer to that is to connect again, which would be a race
     * against the stop below. */
    esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, s_wifi_handler);
    esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, s_ip_handler);

    esp_wifi_disconnect();
    esp_wifi_stop();
    esp_wifi_deinit();

    if (s_netif != NULL) {
        esp_netif_destroy_default_wifi(s_netif);
        s_netif = NULL;
    }
    ESP_LOGI(TAG, "radio off, the panel has the memory bus back");
}

static void net_time_task(void *arg)
{
    (void)arg;

    s_state = NET_TIME_CONNECTING;
    if (!bring_up_radio()) {
        s_state = NET_TIME_FAILED;
        vTaskDelete(NULL);
        return;
    }

    const EventBits_t bits = xEventGroupWaitBits(
        s_events, BIT_GOT_IP | BIT_FAILED, pdTRUE, pdFALSE,
        pdMS_TO_TICKS(CONNECT_WAIT_MS));

    if ((bits & BIT_GOT_IP) == 0) {
        ESP_LOGE(TAG, "could not join \"%s\"", WIFI_SSID);
        take_down_radio();
        s_state = NET_TIME_FAILED;
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "joined \"%s\" as " IPSTR, WIFI_SSID,
             IP2STR((const esp_ip4_addr_t *)&s_address));
    s_state = NET_TIME_SYNCING;

    esp_sntp_config_t sntp = ESP_NETIF_SNTP_DEFAULT_CONFIG(SNTP_SERVER);
    ESP_ERROR_CHECK(esp_netif_sntp_init(&sntp));

    const bool synced = esp_netif_sntp_sync_wait(pdMS_TO_TICKS(SYNC_WAIT_MS)) == ESP_OK;
    esp_netif_sntp_deinit();
    take_down_radio();

    if (!synced) {
        ESP_LOGE(TAG, "no answer from %s", SNTP_SERVER);
        s_state = NET_TIME_FAILED;
        vTaskDelete(NULL);
        return;
    }

    /* SNTP has already put UTC into the system clock. Applying the zone here
     * means the RTC holds local time, which is what the clock on the picker
     * wants and what a part with no notion of zones can represent. */
    setenv("TZ", TIME_ZONE, 1);
    tzset();

    time_t now = 0;
    time(&now);
    struct tm local;
    localtime_r(&now, &local);

    if (pcf85063_set_tm(&local)) {
        ESP_LOGI(TAG, "clock set to %04d-%02d-%02d %02d:%02d:%02d %s",
                 local.tm_year + 1900, local.tm_mon + 1, local.tm_mday,
                 local.tm_hour, local.tm_min, local.tm_sec,
                 local.tm_isdst > 0 ? "PDT" : "PST");
        s_state = NET_TIME_SET;
    } else {
        ESP_LOGE(TAG, "the clock would not take the time");
        s_state = NET_TIME_FAILED;
    }

    vTaskDelete(NULL);
}

void net_time_start(void)
{
    if (!HAVE_NETWORK) {
        ESP_LOGW(TAG, "no wifi_secrets.h, so no network and no time");
        s_state = NET_TIME_NO_CONFIG;
        return;
    }

    s_events = xEventGroupCreate();
    if (s_events == NULL) {
        s_state = NET_TIME_FAILED;
        return;
    }
    xTaskCreatePinnedToCore(net_time_task, "net_time", TASK_STACK, NULL,
                            TASK_PRIORITY, NULL, TASK_CORE);
}

net_time_state_t net_time_state(void)
{
    return s_state;
}

const char *net_time_status(void)
{
    switch (s_state) {
    case NET_TIME_NO_CONFIG:  return "no net";
    case NET_TIME_CONNECTING: return "joining";
    case NET_TIME_SYNCING:    return "asking";
    case NET_TIME_SET:        return "ok";
    case NET_TIME_FAILED:     return "failed";
    case NET_TIME_IDLE:
    default:                  return "--";
    }
}
