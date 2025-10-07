/*
 * BLE LwM2M support with Periodic Advertising (migrated from periodic_adv_demo.c)
 *
 * This file consolidates the extended + periodic advertising logic from the
 * demo into reusable functions exposed via ble_lwm2m.h so that main.c can
 * initialize BLE and start periodic broadcasts without blocking app_main.
 */

#include <stdio.h>
#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"   /* main already initializes, harmless if re-called */

#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "esp_bt_defs.h"
#include "esp_random.h"

#include "ble_lwm2m.h"

#define LOG_TAG "BLE_LWM2M"

#define FUNC_SEND_WAIT_SEM(func, sem) do {            \
		esp_err_t __err_rc = (func);                  \
		if (__err_rc != ESP_OK) {                     \
			ESP_LOGE(LOG_TAG, "%s send fail: %s", #func, esp_err_to_name(__err_rc)); \
		}                                             \
		if ((sem) != NULL) {                          \
			xSemaphoreTake((sem), portMAX_DELAY);     \
		}                                             \
	} while(0)

/* Advertising handle constants */
#define EXT_ADV_HANDLE      0   /* Periodic advertising (non-connectable) */
#define CONN_ADV_HANDLE     1   /* Connectable advertising handle for GATT */
#define NUM_EXT_ADV         2

/* ---------------- Public state from main ---------------- */
static const lwm2m_FactoryPartition *s_factory_partition = NULL; /* not yet used in payload */
static bool s_factory_partition_valid = false;
static bool s_aes_key_exists = false;

/* ---------------- Sensor sample structure ---------------- */
typedef struct {
	uint16_t temperature;    /* *100 */
	uint16_t humidity;       /* *100 */
	uint16_t pressure;       /* hPa */
	uint32_t timestamp;      /* simple counter */
} sensor_data_t;

static sensor_data_t s_sensor = {0};
static uint32_t s_sensor_counter = 0;

/* ---------------- BLE / GATT related static data ---------------- */
static SemaphoreHandle_t s_gap_sem = NULL;
static TaskHandle_t s_adv_task_handle = NULL;
static uint32_t s_adv_interval_ms = 0;
static bool s_started = false;

/* GATT simple RW characteristic (same as demo) */
#define GATTS_APP_ID 0x55
static const uint16_t GATTS_SERVICE_UUID = 0x00FF;
static const uint16_t GATTS_CHAR_RW_UUID = 0xFF01;
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static uint8_t char_prop_rw = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;
static uint8_t rw_char_value[20] = "Hello BLE";
static uint16_t rw_char_value_len = 9;

enum {
	IDX_SVC,
	IDX_CHAR_RW_DECL,
	IDX_CHAR_RW_VAL,
	GATT_IDX_NB,
};
static uint16_t gatt_handle_table[GATT_IDX_NB];

static const esp_gatts_attr_db_t gatt_db[GATT_IDX_NB] = {
	[IDX_SVC] = {
		{ESP_GATT_AUTO_RSP},
		{ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
		 sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID), (uint8_t *)&GATTS_SERVICE_UUID}
	},
	[IDX_CHAR_RW_DECL] = {
		{ESP_GATT_AUTO_RSP},
		{ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
		 sizeof(uint8_t), sizeof(uint8_t), &char_prop_rw}
	},
	[IDX_CHAR_RW_VAL] = {
		{ESP_GATT_RSP_BY_APP},
		{ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_RW_UUID, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
		 sizeof(rw_char_value), 9, rw_char_value}
	},
};

/* Long Range (LE Coded PHY) extended advertising parameters */
static esp_ble_gap_ext_adv_params_t ext_adv_params_LR = {
	.type = ESP_BLE_GAP_SET_EXT_ADV_PROP_NONCONN_NONSCANNABLE_UNDIRECTED,
	.interval_min = 0x60,
	.interval_max = 0x60,
	.channel_map = ADV_CHNL_ALL,
	.filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
	.primary_phy = ESP_BLE_GAP_PHY_CODED,
	.max_skip = 0,
	.secondary_phy = ESP_BLE_GAP_PHY_CODED,
	.sid = 0,
	.scan_req_notif = false,
	.own_addr_type = BLE_ADDR_TYPE_RANDOM,
	.tx_power = EXT_ADV_TX_PWR_NO_PREFERENCE,
};

static esp_ble_gap_periodic_adv_params_t periodic_adv_params = {
	.interval_min = 0x40,
	.interval_max = 0x40,
	.properties = 0,
};

static esp_ble_gap_ext_adv_params_t conn_adv_params = {
	.type = ESP_BLE_GAP_SET_EXT_ADV_PROP_LEGACY_IND,
	.interval_min = 0x40,
	.interval_max = 0x40,
	.channel_map = ADV_CHNL_ALL,
	.filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
	.primary_phy = ESP_BLE_GAP_PHY_1M,
	.max_skip = 0,
	.secondary_phy = ESP_BLE_GAP_PHY_1M,
	.sid = 1,
	.scan_req_notif = false,
	.own_addr_type = BLE_ADDR_TYPE_RANDOM,
	.tx_power = EXT_ADV_TX_PWR_NO_PREFERENCE,
};

static uint8_t periodic_adv_raw_data[] = {
	0x02, ESP_BLE_AD_TYPE_FLAG, 0x06,
	0x02, ESP_BLE_AD_TYPE_TX_PWR, 0xeb,
	0x03, ESP_BLE_AD_TYPE_16SRV_CMPL, 0xab, 0xcd,
	0x11, ESP_BLE_AD_TYPE_NAME_CMPL, 'E','S','P','_','P','E','R','I','O','D','I','C','_','A','D','V'
};

static uint8_t raw_ext_adv_data_lr[] = {
	0x02, ESP_BLE_AD_TYPE_FLAG, 0x06,
	0x02, ESP_BLE_AD_TYPE_TX_PWR, 0xeb,
	/* Complete Local Name: ESP_EXTENDED_ADV (16 chars) */
	0x11, ESP_BLE_AD_TYPE_NAME_CMPL, 'E','S','P','_','E','X','T','E','N','D','E','D','_','A','D','V'
};

static esp_ble_gap_ext_adv_t ext_adv[NUM_EXT_ADV] = {
	[0] = {EXT_ADV_HANDLE, 0, 0},
	[1] = {CONN_ADV_HANDLE, 0, 0},
};

/* ---------------- Internal helpers ---------------- */
static void update_sensor_data(void)
{
	s_sensor.temperature = 2000 + (esp_random() % 1000); /* 20.00 - 30.00 */
	s_sensor.humidity = 4000 + (esp_random() % 4000);    /* 40.00 - 80.00 */
	s_sensor.pressure = 1000 + (esp_random() % 30);      /* 1000 - 1029 */
	s_sensor.timestamp = ++s_sensor_counter;
	ESP_LOGI(LOG_TAG, "Sensor Data T=%.2fC H=%.2f%% P=%u cnt=%lu", s_sensor.temperature/100.0f,
			 s_sensor.humidity/100.0f, s_sensor.pressure, (unsigned long)s_sensor.timestamp);
}

static void create_sensor_adv_data(uint8_t *adv_data, size_t *data_len)
{
	uint8_t pos = 0;
	/* Flags */
	adv_data[pos++] = 0x02; adv_data[pos++] = ESP_BLE_AD_TYPE_FLAG; adv_data[pos++] = 0x06;
	/* TX Power */
	adv_data[pos++] = 0x02; adv_data[pos++] = ESP_BLE_AD_TYPE_TX_PWR; adv_data[pos++] = 0xeb;
	/* Manufacturer specific (Company 0xFFFF) + sensor payload (11 bytes) */
	adv_data[pos++] = 0x0D; /* length */
	adv_data[pos++] = ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE; adv_data[pos++] = 0xFF; adv_data[pos++] = 0xFF;
	adv_data[pos++] = (uint8_t)(s_sensor.temperature & 0xFF);
	adv_data[pos++] = (uint8_t)(s_sensor.temperature >> 8);
	adv_data[pos++] = (uint8_t)(s_sensor.humidity & 0xFF);
	adv_data[pos++] = (uint8_t)(s_sensor.humidity >> 8);
	adv_data[pos++] = (uint8_t)(s_sensor.pressure & 0xFF);
	adv_data[pos++] = (uint8_t)(s_sensor.pressure >> 8);
	adv_data[pos++] = (uint8_t)(s_sensor.timestamp & 0xFF);
	adv_data[pos++] = (uint8_t)(s_sensor.timestamp >> 8);
	adv_data[pos++] = (uint8_t)(s_sensor.timestamp >> 16);
	/* Name */
	const char *name = s_aes_key_exists ? "ESP_SENSOR" : "ESP_LWM2M"; /* slight differentiation */
	uint8_t name_len = strlen(name);
	adv_data[pos++] = name_len + 1; adv_data[pos++] = ESP_BLE_AD_TYPE_NAME_CMPL;
	memcpy(&adv_data[pos], name, name_len); pos += name_len;
	*data_len = pos;
}

/* ---------------- GAP / GATT Callbacks ---------------- */
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
								esp_ble_gatts_cb_param_t *param)
{
	switch (event) {
	case ESP_GATTS_REG_EVT:
		ESP_LOGI(LOG_TAG, "GATT app registered, app_id %d", param->reg.app_id);
		esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, GATT_IDX_NB, 0);
		break;
	case ESP_GATTS_CREAT_ATTR_TAB_EVT:
		if (param->add_attr_tab.status == ESP_GATT_OK && param->add_attr_tab.num_handle == GATT_IDX_NB) {
			memcpy(gatt_handle_table, param->add_attr_tab.handles, sizeof(gatt_handle_table));
			esp_ble_gatts_start_service(gatt_handle_table[IDX_SVC]);
			ESP_LOGI(LOG_TAG, "GATT service started (handle %d)", gatt_handle_table[IDX_SVC]);
		} else {
			ESP_LOGE(LOG_TAG, "Attr table create failed 0x%x", param->add_attr_tab.status);
		}
		break;
	case ESP_GATTS_READ_EVT: {
		esp_gatt_rsp_t rsp = {0};
		rsp.attr_value.handle = param->read.handle;
		if (param->read.handle == gatt_handle_table[IDX_CHAR_RW_VAL]) {
			rsp.attr_value.len = rw_char_value_len;
			memcpy(rsp.attr_value.value, rw_char_value, rw_char_value_len);
		}
		esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
		break; }
	case ESP_GATTS_WRITE_EVT:
		if (param->write.handle == gatt_handle_table[IDX_CHAR_RW_VAL]) {
			size_t wlen = param->write.len > sizeof(rw_char_value) ? sizeof(rw_char_value) : param->write.len;
			memcpy(rw_char_value, param->write.value, wlen);
			rw_char_value_len = wlen;
			ESP_LOGI(LOG_TAG, "Characteristic written (%d bytes)", (int)wlen);
		}
		if (param->write.need_rsp) {
			esp_gatt_rsp_t rsp = {0};
			rsp.attr_value.handle = param->write.handle; rsp.attr_value.len = 0;
			esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, &rsp);
		}
		break;
	case ESP_GATTS_CONNECT_EVT:
		ESP_LOGI(LOG_TAG, "Client connected, conn_id=%d", param->connect.conn_id);
		break;
	case ESP_GATTS_DISCONNECT_EVT:
		ESP_LOGI(LOG_TAG, "Client disconnected reason=0x%x", param->disconnect.reason);
		/* restart connectable advertising */
		esp_ble_gap_ext_adv_start(1, &ext_adv[1]);
		break;
	default:
		break;
	}
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
	switch (event) {
	case ESP_GAP_BLE_EXT_ADV_SET_RAND_ADDR_COMPLETE_EVT:
	case ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT:
	case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
	case ESP_GAP_BLE_EXT_SCAN_RSP_DATA_SET_COMPLETE_EVT:
	case ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT:
	case ESP_GAP_BLE_EXT_ADV_STOP_COMPLETE_EVT:
	case ESP_GAP_BLE_PERIODIC_ADV_SET_PARAMS_COMPLETE_EVT:
	case ESP_GAP_BLE_PERIODIC_ADV_DATA_SET_COMPLETE_EVT:
	case ESP_GAP_BLE_PERIODIC_ADV_START_COMPLETE_EVT:
		if (s_gap_sem) xSemaphoreGive(s_gap_sem);
		break;
	default:
		break;
	}
}

/* ---------------- Advertising Task ---------------- */
static void adv_update_task(void *arg)
{
	ESP_LOGI(LOG_TAG, "Advertising update task started (interval=%u ms)", (unsigned)s_adv_interval_ms);
	uint8_t adv_buf[31];
	size_t adv_len = 0;
	while (s_started) {
		update_sensor_data();
		create_sensor_adv_data(adv_buf, &adv_len);
#if CONFIG_BT_BLE_FEAT_PERIODIC_ADV_ENH
		FUNC_SEND_WAIT_SEM(esp_ble_gap_config_periodic_adv_data_raw(EXT_ADV_HANDLE, adv_len, adv_buf, false), s_gap_sem);
#else
		FUNC_SEND_WAIT_SEM(esp_ble_gap_config_periodic_adv_data_raw(EXT_ADV_HANDLE, adv_len, adv_buf), s_gap_sem);
#endif
		uint32_t wait_ms = s_adv_interval_ms ? s_adv_interval_ms : 3000;
		if (wait_ms < 3000) wait_ms = 3000; /* keep radio load reasonable */
		vTaskDelay(pdMS_TO_TICKS(wait_ms));
	}
	ESP_LOGI(LOG_TAG, "Advertising update task exiting");
	vTaskDelete(NULL);
}

/* ---------------- Public API Implementations ---------------- */
esp_err_t ble_lwm2m_init(void)
{
	if (s_started) {
		return ESP_OK; /* already initialized */
	}

	/* Controller memory optimization: release classic BT */
	ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

	esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	esp_err_t ret = esp_bt_controller_init(&bt_cfg);
	if (ret) {
		ESP_LOGE(LOG_TAG, "Controller init failed: %s", esp_err_to_name(ret));
		return ret;
	}
	ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
	if (ret) { ESP_LOGE(LOG_TAG, "Controller enable failed: %s", esp_err_to_name(ret)); return ret; }
	ret = esp_bluedroid_init();
	if (ret) { ESP_LOGE(LOG_TAG, "Bluedroid init failed: %s", esp_err_to_name(ret)); return ret; }
	ret = esp_bluedroid_enable();
	if (ret) { ESP_LOGE(LOG_TAG, "Bluedroid enable failed: %s", esp_err_to_name(ret)); return ret; }

	ESP_ERROR_CHECK(esp_ble_gap_register_callback(gap_event_handler));
	ESP_ERROR_CHECK(esp_ble_gatts_register_callback(gatts_event_handler));
	ESP_ERROR_CHECK(esp_ble_gatts_app_register(GATTS_APP_ID));

	esp_ble_gap_set_device_name("ESP_SENSOR");

	if (!s_gap_sem) {
		s_gap_sem = xSemaphoreCreateBinary();
	}

	/* Prepare advertising data / parameters */
	esp_bd_addr_t rand_addr; esp_ble_gap_addr_create_static(rand_addr);
	FUNC_SEND_WAIT_SEM(esp_ble_gap_ext_adv_set_params(EXT_ADV_HANDLE, &ext_adv_params_LR), s_gap_sem);
	FUNC_SEND_WAIT_SEM(esp_ble_gap_ext_adv_set_rand_addr(EXT_ADV_HANDLE, rand_addr), s_gap_sem);
	FUNC_SEND_WAIT_SEM(esp_ble_gap_config_ext_adv_data_raw(EXT_ADV_HANDLE, sizeof(raw_ext_adv_data_lr), raw_ext_adv_data_lr), s_gap_sem);

	esp_bd_addr_t conn_rand_addr; esp_ble_gap_addr_create_static(conn_rand_addr);
	FUNC_SEND_WAIT_SEM(esp_ble_gap_ext_adv_set_params(CONN_ADV_HANDLE, &conn_adv_params), s_gap_sem);
	FUNC_SEND_WAIT_SEM(esp_ble_gap_ext_adv_set_rand_addr(CONN_ADV_HANDLE, conn_rand_addr), s_gap_sem);
	/* Minimal connectable adv data (Flags + Name) */
	uint8_t conn_adv_data[31]; uint8_t p = 0; const char *cn = "ESP_CONNECT"; uint8_t cn_len = strlen(cn);
	conn_adv_data[p++] = 0x02; conn_adv_data[p++] = ESP_BLE_AD_TYPE_FLAG; conn_adv_data[p++] = 0x06;
	conn_adv_data[p++] = cn_len + 1; conn_adv_data[p++] = ESP_BLE_AD_TYPE_NAME_CMPL; memcpy(&conn_adv_data[p], cn, cn_len); p += cn_len;
	FUNC_SEND_WAIT_SEM(esp_ble_gap_config_ext_adv_data_raw(CONN_ADV_HANDLE, p, conn_adv_data), s_gap_sem);

	FUNC_SEND_WAIT_SEM(esp_ble_gap_ext_adv_start(NUM_EXT_ADV, &ext_adv[0]), s_gap_sem);
	FUNC_SEND_WAIT_SEM(esp_ble_gap_periodic_adv_set_params(EXT_ADV_HANDLE, &periodic_adv_params), s_gap_sem);

#if CONFIG_BT_BLE_FEAT_PERIODIC_ADV_ENH
	FUNC_SEND_WAIT_SEM(esp_ble_gap_config_periodic_adv_data_raw(EXT_ADV_HANDLE, sizeof(periodic_adv_raw_data), periodic_adv_raw_data, false), s_gap_sem);
	FUNC_SEND_WAIT_SEM(esp_ble_gap_periodic_adv_start(EXT_ADV_HANDLE, true), s_gap_sem);
#else
	FUNC_SEND_WAIT_SEM(esp_ble_gap_config_periodic_adv_data_raw(EXT_ADV_HANDLE, sizeof(periodic_adv_raw_data), periodic_adv_raw_data), s_gap_sem);
	FUNC_SEND_WAIT_SEM(esp_ble_gap_periodic_adv_start(EXT_ADV_HANDLE), s_gap_sem);
#endif

	s_started = true;
	ESP_LOGI(LOG_TAG, "BLE LwM2M initialized (periodic advertising active)");
	return ESP_OK;
}

void ble_lwm2m_set_factory_partition(const lwm2m_FactoryPartition* partition, bool valid)
{
	s_factory_partition = partition;
	s_factory_partition_valid = valid;
	ESP_LOGI(LOG_TAG, "Factory partition set (valid=%d)", valid);
}

void ble_lwm2m_set_aes_key_status(bool exists)
{
	s_aes_key_exists = exists;
	ESP_LOGI(LOG_TAG, "AES key status: %s", exists ? "present" : "absent");
}

esp_err_t ble_lwm2m_start_periodic_broadcast(uint32_t interval_ms)
{
	if (!s_started) {
		ESP_LOGE(LOG_TAG, "BLE not initialized yet");
		return ESP_ERR_INVALID_STATE;
	}
	if (s_adv_task_handle) {
		ESP_LOGW(LOG_TAG, "Broadcast task already running");
		return ESP_OK;
	}
	s_adv_interval_ms = interval_ms;
	xTaskCreatePinnedToCore(adv_update_task, "ble_adv_upd", 4096, NULL, 5, &s_adv_task_handle, tskNO_AFFINITY);
	return ESP_OK;
}

esp_err_t ble_lwm2m_stop_periodic_broadcast(void)
{
	if (!s_started) return ESP_ERR_INVALID_STATE;
	if (s_adv_task_handle) {
		TaskHandle_t h = s_adv_task_handle; s_adv_task_handle = NULL; s_started = true; /* keep radio */
		/* Signal task to exit by clearing s_started? we keep advertising; use flag */
		s_started = true; /* keep BLE active but stop task */
		vTaskDelete(h); /* abrupt stop */
	}
	return ESP_OK;
}

esp_err_t ble_lwm2m_broadcast_appearance(void)
{
	/* This function could encode factory partition appearance into adv payload. For now, no-op. */
	return ESP_OK;
}

esp_err_t ble_lwm2m_broadcast_temperature(void)
{
	/* Temperature now embedded in periodic adv updates. */
	return ESP_OK;
}

void ble_lwm2m_deinit(void)
{
	if (!s_started) return;
	s_started = false;
	if (s_adv_task_handle) {
		TaskHandle_t h = s_adv_task_handle; s_adv_task_handle = NULL;
		vTaskDelete(h);
	}
	if (s_gap_sem) { vSemaphoreDelete(s_gap_sem); s_gap_sem = NULL; }
	esp_ble_gap_stop_advertising(); /* legacy stop, extended sets separately */
	esp_bt_controller_disable();
	esp_bluedroid_disable();
	esp_bluedroid_deinit();
	esp_bt_controller_deinit();
	ESP_LOGI(LOG_TAG, "BLE LwM2M deinitialized");
}

