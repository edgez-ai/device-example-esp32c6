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
#include "lwm2m.pb.h"
#include "pb_encode.h"
#include "pb_decode.h"

/* Conditional minimal crypto support: provide stubs if mbedTLS components not enabled. */
#ifdef CONFIG_MBEDTLS_AES_C
#include "mbedtls/aes.h"
#else
typedef struct { int dummy; } mbedtls_aes_context; 
static void mbedtls_aes_init(mbedtls_aes_context *c){(void)c;} 
static void mbedtls_aes_free(mbedtls_aes_context *c){(void)c;} 
static int mbedtls_aes_setkey_enc(mbedtls_aes_context *c,const unsigned char *k,unsigned int kb){(void)c;(void)k;(void)kb;return 0;} 
static int mbedtls_aes_crypt_ecb(mbedtls_aes_context *c,int m,const unsigned char in[16],unsigned char out[16]){(void)c;(void)m;memcpy(out,in,16);return 0;} 
#define MBEDTLS_AES_ENCRYPT 1
#endif
#ifdef CONFIG_MBEDTLS_SHA512_C
#include "mbedtls/sha512.h"
#endif
#ifdef CONFIG_MBEDTLS_ECDH_C
#include "mbedtls/ecdh.h"
#include "mbedtls/ecp.h"
#endif
#if defined(CONFIG_MBEDTLS_CHACHA20_C) && defined(CONFIG_MBEDTLS_POLY1305_C)
#include "mbedtls/chacha20.h"
#include "mbedtls/poly1305.h"
#define HAS_CHACHA20_POLY1305 1
#else
#define HAS_CHACHA20_POLY1305 0
#endif
/* Removed direct include of mbedtls/aes.h; guarded stub/conditional version provided later */

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

/* Forward declaration for challenge processing (implemented later in file) */
static void ble_lwm2m_process_challenge_write(uint16_t conn_id, const uint8_t *data, uint16_t len);

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
static TaskHandle_t s_adv_task_handle = NULL;    /* Handle for the periodic update task */
static uint32_t s_adv_interval_ms = 0;           /* Desired sensor update interval (ms) */
static bool s_initialized = false;               /* BLE stack + advertising initialized */
static bool s_adv_task_running = false;          /* Advert update task run flag */

/* GATT simple RW characteristic (same as demo) */
#define GATTS_APP_ID 0x55
static const uint16_t GATTS_SERVICE_UUID = 0x00FF;
static const uint16_t GATTS_CHAR_RW_UUID = 0xFF01;
static const uint16_t primary_service_uuid = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid = ESP_GATT_UUID_CHAR_DECLARE;
static uint8_t char_prop_rw = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;
/* Default characteristic value shown to clients that haven't written yet */
/* Increase buffers to allow returning encrypted challenge answer (sig + tag) */
static uint8_t rw_char_value[128] = "Hello BLE";  /* enlarge to hold greeting + name */
static uint16_t rw_char_value_len = 9;

/* Maintain per-connection values so only the writer sees their updated response */
#define MAX_CONN 4
typedef struct {
	bool in_use;
	uint16_t conn_id;
	uint8_t value[128];
	uint16_t len;
} conn_ctx_t;
static conn_ctx_t s_conns[MAX_CONN] = {0};

static conn_ctx_t* find_conn_ctx(uint16_t conn_id)
{
	for (int i = 0; i < MAX_CONN; ++i) {
		if (s_conns[i].in_use && s_conns[i].conn_id == conn_id) return &s_conns[i];
	}
	return NULL;
}

static conn_ctx_t* alloc_conn_ctx(uint16_t conn_id)
{
	conn_ctx_t *ctx = find_conn_ctx(conn_id);
	if (ctx) return ctx;
	for (int i = 0; i < MAX_CONN; ++i) {
		if (!s_conns[i].in_use) {
			s_conns[i].in_use = true;
			s_conns[i].conn_id = conn_id;
			/* initialize with default value */
			memcpy(s_conns[i].value, rw_char_value, rw_char_value_len);
			s_conns[i].len = rw_char_value_len;
			return &s_conns[i];
		}
	}
	return NULL; /* no slot */
}

static void free_conn_ctx(uint16_t conn_id)
{
	for (int i = 0; i < MAX_CONN; ++i) {
		if (s_conns[i].in_use && s_conns[i].conn_id == conn_id) {
			s_conns[i].in_use = false;
			s_conns[i].conn_id = 0;
			memset(s_conns[i].value, 0, sizeof(s_conns[i].value));
			s_conns[i].len = 0;
			break;
		}
	}
}

/* ---------------- Simple AES-256 ECB PKCS7 encryption helper ----------------
 * NOTE: ECB is used here for simplicity because the requirement only asked to
 * "respond with AES encryption". For production use, prefer an authenticated
 * mode (e.g. AES-GCM) or at least CBC with a random IV transmitted alongside.
 * Output buffer must have capacity for padded length ( (in_len+pad) ).
 */
static bool encrypt_aes256_ecb_pkcs7(const uint8_t *key32,
									 const uint8_t *in, size_t in_len,
									 uint8_t *out, size_t *out_len,
									 size_t out_cap)
{
	if (!key32 || !in || !out || !out_len) return false;
	const size_t block = 16;
	if (out_cap < block) return false;
	/* Make sure padded size fits */
	size_t pad_len = block - (in_len % block);
	size_t padded = in_len + pad_len;
	if (padded > out_cap) {
		/* Truncate plaintext so that after padding it fits */
		size_t max_plain = (out_cap / block) * block; /* full blocks capacity */
		if (max_plain == 0) return false;
		/* leave room for at least one padding block */
		if (in_len > max_plain - block) in_len = max_plain - block;
		pad_len = block - (in_len % block);
		padded = in_len + pad_len;
	}
	uint8_t buf[64]; /* temporary block buffer (max characteristic size) */
	if (padded > sizeof(buf)) return false; /* safety */
	memcpy(buf, in, in_len);
	/* PKCS7 padding */
	memset(buf + in_len, (uint8_t)pad_len, pad_len);

	mbedtls_aes_context ctx;
	mbedtls_aes_init(&ctx);
	if (mbedtls_aes_setkey_enc(&ctx, key32, 256) != 0) {
		mbedtls_aes_free(&ctx);
		return false;
	}
	for (size_t off = 0; off < padded; off += block) {
		if (mbedtls_aes_crypt_ecb(&ctx, MBEDTLS_AES_ENCRYPT, buf + off, out + off) != 0) {
			mbedtls_aes_free(&ctx);
			return false;
		}
	}
	mbedtls_aes_free(&ctx);
	*out_len = padded;
	return true;
}

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
	/* 5 s advertising interval: 5000 ms / 0.625 ms = 8000 (0x1F40) */
	.interval_min = 0x1F40,
	.interval_max = 0x1F40,
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
	/* 1 s periodic adv interval: 1000 ms / 1.25 ms = 800 (0x320) */
	/* NOTE: Periodic advertising interval units are 1.25 ms, so 1s = 800 (0x320).
	 * We match extended adv update (set to 0x640 in 0.625 ms units) roughly.
	 */
	.interval_min = 0x320,
	.interval_max = 0x320,
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
	
	if (!s_aes_key_exists) {
		/* When AES key doesn't exist, broadcast protobuf LwM2MMessage with Appearance */
		
		/* Flags */
		adv_data[pos++] = 0x02; adv_data[pos++] = ESP_BLE_AD_TYPE_FLAG; adv_data[pos++] = 0x06;
		/* TX Power */
		adv_data[pos++] = 0x02; adv_data[pos++] = ESP_BLE_AD_TYPE_TX_PWR; adv_data[pos++] = 0xeb;
		
		/* Create protobuf LwM2MMessage with Appearance */
		lwm2m_LwM2MMessage lwm2m_msg = lwm2m_LwM2MMessage_init_zero;
		if (s_factory_partition) {
			lwm2m_msg.serial = s_factory_partition->serial_number;
			lwm2m_msg.model = s_factory_partition->model;
		}


		/* Encode protobuf to bytes */
		uint8_t pb_buffer[92]; /* lwm2m_LwM2MMessage_size is 92 bytes max */
		pb_ostream_t stream = pb_ostream_from_buffer(pb_buffer, sizeof(pb_buffer));
		
		if (pb_encode(&stream, lwm2m_LwM2MMessage_fields, &lwm2m_msg)) {
			/* Manufacturer specific data with protobuf payload */
			size_t pb_len = stream.bytes_written;
			if (pos + 4 + pb_len < 31) { /* ensure it fits in BLE advertising packet */
				adv_data[pos++] = pb_len + 3; /* length of manufacturer data */
				adv_data[pos++] = ESP_BLE_AD_MANUFACTURER_SPECIFIC_TYPE;
				adv_data[pos++] = 0xFF; adv_data[pos++] = 0xFF; /* Company ID 0xFFFF */
				memcpy(&adv_data[pos], pb_buffer, pb_len);
				pos += pb_len;
			}
		} else {
			ESP_LOGE(LOG_TAG, "Failed to encode protobuf message");
		}
		
		/* Name */
		const char *name = "ESP_LWM2M";
		uint8_t name_len = strlen(name);
		if (pos + name_len + 2 < 31) {
			adv_data[pos++] = name_len + 1; adv_data[pos++] = ESP_BLE_AD_TYPE_NAME_CMPL;
			memcpy(&adv_data[pos], name, name_len); pos += name_len;
		}
		
	} else {
		/* Original sensor data format when AES key exists */
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
		const char *name = "ESP_SENSOR";
		uint8_t name_len = strlen(name);
		adv_data[pos++] = name_len + 1; adv_data[pos++] = ESP_BLE_AD_TYPE_NAME_CMPL;
		memcpy(&adv_data[pos], name, name_len); pos += name_len;
	}
	
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
				/* Return per-connection value if present; otherwise default */
				conn_ctx_t *ctx = find_conn_ctx(param->read.conn_id);
				if (ctx && ctx->len > 0) {
					rsp.attr_value.len = ctx->len;
					memcpy(rsp.attr_value.value, ctx->value, ctx->len);
				} else {
					rsp.attr_value.len = rw_char_value_len;
					memcpy(rsp.attr_value.value, rw_char_value, rw_char_value_len);
				}
		}
		esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
		break; }
	case ESP_GATTS_WRITE_EVT:
		ESP_LOGI(LOG_TAG, "GATT write conn_id=%d handle=%d len=%d",
				 param->write.conn_id, param->write.handle, param->write.len);
		if (param->write.handle == gatt_handle_table[IDX_CHAR_RW_VAL]) {
			/* Process challenge write (LwM2MDeviceChallenge) */
			ble_lwm2m_process_challenge_write(param->write.conn_id, param->write.value, param->write.len);
		} else {
			ESP_LOGW(LOG_TAG, "Write to unknown handle %d", param->write.handle);
		}
		if (param->write.need_rsp) {
			esp_gatt_rsp_t rsp = {0};
			rsp.attr_value.handle = param->write.handle; rsp.attr_value.len = 0;
			esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, &rsp);
		}
		break;
	case ESP_GATTS_CONNECT_EVT:
		ESP_LOGI(LOG_TAG, "Client connected, conn_id=%d", param->connect.conn_id);
		/* Allocate per-connection context */
		if (!alloc_conn_ctx(param->connect.conn_id)) {
			ESP_LOGW(LOG_TAG, "No free connection slots for conn_id=%u", (unsigned)param->connect.conn_id);
		}
		break;
	case ESP_GATTS_DISCONNECT_EVT:
		ESP_LOGI(LOG_TAG, "Client disconnected reason=0x%x", param->disconnect.reason);
		/* Release per-connection context */
		free_conn_ctx(param->disconnect.conn_id);
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
	while (s_adv_task_running) {
		update_sensor_data();
		create_sensor_adv_data(adv_buf, &adv_len);
		ESP_LOGI(LOG_TAG, "Broadcasting %s data (len=%d bytes)",
				s_aes_key_exists ? "sensor" : "LwM2M protobuf appearance", (int)adv_len);
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
	/* Mark handle NULL before deleting so stop routine can return */
	TaskHandle_t self = s_adv_task_handle;
	s_adv_task_handle = NULL;
	vTaskDelete(self);
}

/* ---------------- Public API Implementations ---------------- */
esp_err_t ble_lwm2m_init(void)
{
	if (s_initialized) {
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
	/* Use SAME random address for both periodic and connectable advertising */
	esp_bd_addr_t rand_addr; esp_ble_gap_addr_create_static(rand_addr);
	FUNC_SEND_WAIT_SEM(esp_ble_gap_ext_adv_set_params(EXT_ADV_HANDLE, &ext_adv_params_LR), s_gap_sem);
	FUNC_SEND_WAIT_SEM(esp_ble_gap_ext_adv_set_rand_addr(EXT_ADV_HANDLE, rand_addr), s_gap_sem);
	FUNC_SEND_WAIT_SEM(esp_ble_gap_config_ext_adv_data_raw(EXT_ADV_HANDLE, sizeof(raw_ext_adv_data_lr), raw_ext_adv_data_lr), s_gap_sem);

	/* IMPORTANT: Use the SAME random address for connectable advertising so gateway can connect after syncing to periodic adv */
	FUNC_SEND_WAIT_SEM(esp_ble_gap_ext_adv_set_params(CONN_ADV_HANDLE, &conn_adv_params), s_gap_sem);
	FUNC_SEND_WAIT_SEM(esp_ble_gap_ext_adv_set_rand_addr(CONN_ADV_HANDLE, rand_addr), s_gap_sem);
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

	s_initialized = true;
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
	if (!s_initialized) {
		ESP_LOGE(LOG_TAG, "BLE not initialized yet");
		return ESP_ERR_INVALID_STATE;
	}
	if (s_adv_task_handle) {
		ESP_LOGW(LOG_TAG, "Broadcast task already running");
		return ESP_OK;
	}
	s_adv_interval_ms = interval_ms;
	s_adv_task_running = true;
	if (xTaskCreatePinnedToCore(adv_update_task, "ble_adv_upd", 4096, NULL, 5, &s_adv_task_handle, tskNO_AFFINITY) != pdPASS) {
		ESP_LOGE(LOG_TAG, "Failed to create advertising update task");
		s_adv_task_running = false;
		return ESP_FAIL;
	}
	return ESP_OK;
}

esp_err_t ble_lwm2m_stop_periodic_broadcast(void)
{
	if (!s_initialized) return ESP_ERR_INVALID_STATE;
	if (!s_adv_task_handle) return ESP_OK; /* already stopped */
	/* Signal task to exit gracefully */
	s_adv_task_running = false;
	/* Wait (bounded) for task to null out its handle */
	for (int i = 0; i < 50 && s_adv_task_handle; ++i) { /* ~500ms max */
		vTaskDelay(pdMS_TO_TICKS(10));
	}
	if (s_adv_task_handle) { /* Fallback: force delete */
		ESP_LOGW(LOG_TAG, "Force deleting advertising task");
		vTaskDelete(s_adv_task_handle);
		s_adv_task_handle = NULL;
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
	if (!s_initialized) return;
	/* Stop periodic broadcast task */
	ble_lwm2m_stop_periodic_broadcast();
	s_initialized = false;
	/* Stop periodic + extended advertising */
#if CONFIG_BT_BLE_FEAT_PERIODIC_ADV_ENH
	esp_ble_gap_periodic_adv_stop(EXT_ADV_HANDLE, true);
#else
	esp_ble_gap_periodic_adv_stop(EXT_ADV_HANDLE);
#endif
	/* Stop both extended advertising instances: periodic (non-connectable) and connectable */
	uint8_t adv_handles[NUM_EXT_ADV] = { EXT_ADV_HANDLE, CONN_ADV_HANDLE };
	esp_ble_gap_ext_adv_stop(NUM_EXT_ADV, adv_handles);
	if (s_gap_sem) { vSemaphoreDelete(s_gap_sem); s_gap_sem = NULL; }
	esp_bt_controller_disable();
	esp_bluedroid_disable();
	esp_bluedroid_deinit();
	esp_bt_controller_deinit();
	ESP_LOGI(LOG_TAG, "BLE LwM2M deinitialized");
}

/* ---------------- Challenge Write Processing ---------------- */
/* ---------------- ChaCha20-Poly1305 AEAD Encryption ---------------- */
/* Derive 32-byte symmetric key via ECDH(peer_pub, our_priv=factory private key) then SHA-256(shared) */
/* Encrypt factory partition signature (64 bytes) using ChaCha20-Poly1305 with nonce from challenge. */
/* Store ciphertext||tag into per-connection buffer for later read. */
static bool chacha20poly1305_encrypt(const uint8_t *key32, const uint32_t nonce32,
									const uint8_t *in, size_t in_len,
									uint8_t *out, size_t *out_len, size_t cap)
{
	if (!key32 || !in || !out || !out_len) return false;
	
	/* Need space for ciphertext + 16-byte Poly1305 tag */
	if (cap < in_len + 16) {
		ESP_LOGE(LOG_TAG, "Output buffer too small: need %u, have %u", 
				 (unsigned)(in_len + 16), (unsigned)cap);
		return false;
	}

#if HAS_CHACHA20_POLY1305
	ESP_LOGI(LOG_TAG, "Using ChaCha20-Poly1305 AEAD encryption");
	
	/* Construct 12-byte nonce: 8 bytes zeros + 4 bytes from challenge nonce */
	uint8_t nonce12[12] = {0};
	/* Place the 32-bit nonce in little-endian at offset 8 */
	nonce12[8] = (nonce32 >> 0) & 0xFF;
	nonce12[9] = (nonce32 >> 8) & 0xFF;
	nonce12[10] = (nonce32 >> 16) & 0xFF;
	nonce12[11] = (nonce32 >> 24) & 0xFF;
	
	ESP_LOG_BUFFER_HEX_LEVEL(LOG_TAG, nonce12, 12, ESP_LOG_DEBUG);
	
	/* Initialize ChaCha20 context */
	mbedtls_chacha20_context chacha_ctx;
	mbedtls_chacha20_init(&chacha_ctx);
	
	int ret = mbedtls_chacha20_setkey(&chacha_ctx, key32);
	if (ret != 0) {
		ESP_LOGE(LOG_TAG, "ChaCha20 setkey failed: -0x%04x", -ret);
		mbedtls_chacha20_free(&chacha_ctx);
		return false;
	}
	
	ret = mbedtls_chacha20_starts(&chacha_ctx, nonce12, 1); /* counter starts at 1 for AEAD */
	if (ret != 0) {
		ESP_LOGE(LOG_TAG, "ChaCha20 starts failed: -0x%04x", -ret);
		mbedtls_chacha20_free(&chacha_ctx);
		return false;
	}
	
	/* Encrypt the plaintext */
	ret = mbedtls_chacha20_update(&chacha_ctx, in_len, in, out);
	if (ret != 0) {
		ESP_LOGE(LOG_TAG, "ChaCha20 encrypt failed: -0x%04x", -ret);
		mbedtls_chacha20_free(&chacha_ctx);
		return false;
	}
	
	mbedtls_chacha20_free(&chacha_ctx);
	
	/* Generate Poly1305 authentication tag */
	mbedtls_poly1305_context poly_ctx;
	mbedtls_poly1305_init(&poly_ctx);
	
	/* Derive Poly1305 key from ChaCha20 with counter 0 */
	uint8_t poly_key[32];
	mbedtls_chacha20_init(&chacha_ctx);
	ret = mbedtls_chacha20_setkey(&chacha_ctx, key32);
	if (ret != 0) {
		ESP_LOGE(LOG_TAG, "ChaCha20 setkey for Poly1305 key failed: -0x%04x", -ret);
		mbedtls_chacha20_free(&chacha_ctx);
		mbedtls_poly1305_free(&poly_ctx);
		return false;
	}
	
	ret = mbedtls_chacha20_starts(&chacha_ctx, nonce12, 0); /* counter 0 for Poly1305 key */
	if (ret != 0) {
		ESP_LOGE(LOG_TAG, "ChaCha20 starts for Poly1305 key failed: -0x%04x", -ret);
		mbedtls_chacha20_free(&chacha_ctx);
		mbedtls_poly1305_free(&poly_ctx);
		return false;
	}
	
	memset(poly_key, 0, 32);
	ret = mbedtls_chacha20_update(&chacha_ctx, 32, poly_key, poly_key);
	if (ret != 0) {
		ESP_LOGE(LOG_TAG, "ChaCha20 Poly1305 key generation failed: -0x%04x", -ret);
		mbedtls_chacha20_free(&chacha_ctx);
		mbedtls_poly1305_free(&poly_ctx);
		return false;
	}
	
	mbedtls_chacha20_free(&chacha_ctx);
	
	/* Initialize Poly1305 with the derived key */
	ret = mbedtls_poly1305_starts(&poly_ctx, poly_key);
	if (ret != 0) {
		ESP_LOGE(LOG_TAG, "Poly1305 starts failed: -0x%04x", -ret);
		mbedtls_poly1305_free(&poly_ctx);
		return false;
	}
	
	/* No additional authenticated data (AAD) in this implementation */
	/* Authenticate the ciphertext */
	ret = mbedtls_poly1305_update(&poly_ctx, out, in_len);
	if (ret != 0) {
		ESP_LOGE(LOG_TAG, "Poly1305 update failed: -0x%04x", -ret);
		mbedtls_poly1305_free(&poly_ctx);
		return false;
	}
	
	/* Finalize and get the authentication tag */
	ret = mbedtls_poly1305_finish(&poly_ctx, &out[in_len]);
	mbedtls_poly1305_free(&poly_ctx);
	
	if (ret != 0) {
		ESP_LOGE(LOG_TAG, "Poly1305 finish failed: -0x%04x", -ret);
		return false;
	}
	
	*out_len = in_len + 16; /* ciphertext + 16-byte tag */
	ESP_LOGI(LOG_TAG, "ChaCha20-Poly1305 encryption successful: %u->%u bytes", 
			 (unsigned)in_len, (unsigned)*out_len);
	return true;
	
#else
	/* Fallback: use AES-256 ECB PKCS7 if ChaCha20-Poly1305 not available */
	ESP_LOGW(LOG_TAG, "ChaCha20-Poly1305 not available, using AES-256 ECB fallback");
	size_t enc_len = 0;
	if (encrypt_aes256_ecb_pkcs7(key32, in, in_len, out, &enc_len, cap)) {
		*out_len = enc_len;
		return true;
	} else {
		*out_len = 0;
		return false;
	}
#endif
}

static void ble_lwm2m_process_challenge_write(uint16_t conn_id, const uint8_t *data, uint16_t len)
{
	conn_ctx_t *ctx = alloc_conn_ctx(conn_id);
	if (!ctx) {
		ESP_LOGE(LOG_TAG, "No connection context for challenge write (conn %u)", (unsigned)conn_id);
		return;
	}
	ctx->len = 0; /* reset */
	if (!data || len == 0) {
		ESP_LOGW(LOG_TAG, "Empty challenge write (conn %u)", (unsigned)conn_id);
		return;
	}
	/* Decode LwM2MDeviceChallenge protobuf */
	pb_istream_t istream = pb_istream_from_buffer(data, len);
	lwm2m_LwM2MDeviceChallenge challenge = lwm2m_LwM2MDeviceChallenge_init_zero;
	if (!pb_decode(&istream, lwm2m_LwM2MDeviceChallenge_fields, &challenge)) {
		ESP_LOGE(LOG_TAG, "Challenge decode failed: %s", PB_GET_ERROR(&istream));
		return;
	}
	if (!(s_factory_partition && s_factory_partition_valid)) {
		ESP_LOGW(LOG_TAG, "Factory partition invalid; cannot answer challenge");
		return;
	}
	/* Validate lengths */
	if (challenge.public_key.size != 32) {
		ESP_LOGE(LOG_TAG, "Challenge public key size invalid (%u)", (unsigned)challenge.public_key.size);
		return;
	}
	ESP_LOGI(LOG_TAG, "Challenge: nounce=0x%08x pubkey_len=%ld",
				(unsigned)challenge.nounce, (long)challenge.public_key.size);
	/* Dump public_key in hex at DEBUG level */
	ESP_LOG_BUFFER_HEX_LEVEL(LOG_TAG, challenge.public_key.bytes,
							 challenge.public_key.size, ESP_LOG_INFO);
	const uint8_t *peer_pub = challenge.public_key.bytes;
	const uint8_t *our_priv = s_factory_partition->private_key.bytes; /* 32 bytes */

	uint8_t key32[32];
	
	/* Derive shared secret using SHA-512 based key derivation */
	/* Since we have both keys as 32-byte values, we'll use HKDF-like approach with SHA-512 */
#ifdef CONFIG_MBEDTLS_SHA512_C
	ESP_LOGI(LOG_TAG, "Using SHA-512 based key derivation");
	
	/* Combine the keys using SHA-512(our_priv || peer_pub || "ECDH_KEY") */
	mbedtls_sha512_context sha_ctx;
	mbedtls_sha512_init(&sha_ctx);
	int ret = mbedtls_sha512_starts(&sha_ctx, 0); /* 0 = SHA-512 (not SHA-384) */
	if (ret != 0) {
		ESP_LOGE(LOG_TAG, "SHA512 start failed: -0x%04x", -ret);
		mbedtls_sha512_free(&sha_ctx);
		goto fallback_key_derivation;
	}
	
	/* Hash our private key */
	ret = mbedtls_sha512_update(&sha_ctx, our_priv, 32);
	if (ret != 0) {
		ESP_LOGE(LOG_TAG, "SHA512 update private key failed: -0x%04x", -ret);
		mbedtls_sha512_free(&sha_ctx);
		goto fallback_key_derivation;
	}
	
	/* Hash peer public key */
	ret = mbedtls_sha512_update(&sha_ctx, peer_pub, 32);
	if (ret != 0) {
		ESP_LOGE(LOG_TAG, "SHA512 update public key failed: -0x%04x", -ret);
		mbedtls_sha512_free(&sha_ctx);
		goto fallback_key_derivation;
	}
	
	/* Add a salt/label for key derivation */
	const char *label = "BLE_LWM2M_ECDH_KEY_V1";
	ret = mbedtls_sha512_update(&sha_ctx, (const unsigned char*)label, strlen(label));
	if (ret != 0) {
		ESP_LOGE(LOG_TAG, "SHA512 update label failed: -0x%04x", -ret);
		mbedtls_sha512_free(&sha_ctx);
		goto fallback_key_derivation;
	}
	
	/* Finalize hash to get derived key - SHA-512 produces 64 bytes, truncate to 32 */
	uint8_t sha512_output[64];
	ret = mbedtls_sha512_finish(&sha_ctx, sha512_output);
	mbedtls_sha512_free(&sha_ctx);
	
	if (ret != 0) {
		ESP_LOGE(LOG_TAG, "SHA512 finish failed: -0x%04x", -ret);
		goto fallback_key_derivation;
	}
	
	/* Use first 32 bytes of SHA-512 output as the derived key */
	memcpy(key32, sha512_output, 32);
	
	ESP_LOGI(LOG_TAG, "SHA-512 key derivation successful");
	ESP_LOG_BUFFER_HEX_LEVEL(LOG_TAG, key32, 32, ESP_LOG_DEBUG);
	goto key_derivation_done;
	
fallback_key_derivation:
#endif
	/* Fallback: use simple XOR combination if SHA512 not available or failed */
	ESP_LOGW(LOG_TAG, "Using fallback key derivation");
	for (int i = 0; i < 32; i++) {
		key32[i] = our_priv[i] ^ peer_pub[i];
	}
	ESP_LOGI(LOG_TAG, "Fallback key derivation completed");
	
#ifdef CONFIG_MBEDTLS_SHA512_C	
key_derivation_done:
#endif

	/* Prepare plaintext = factory signature (64 bytes) */
	size_t sig_len = s_factory_partition->signature.size;
	if (sig_len == 0 || sig_len > sizeof(s_factory_partition->signature.bytes)) {
		ESP_LOGE(LOG_TAG, "Invalid factory signature len=%u", (unsigned)sig_len);
		return;
	}
	const uint8_t *sig = s_factory_partition->signature.bytes;

	/* Encrypt the signature using ChaCha20-Poly1305 */
	uint8_t encrypted_sig[128]; /* Buffer for encrypted signature */
	size_t enc_sig_len = 0;
	if (!chacha20poly1305_encrypt(key32, challenge.nounce, sig, sig_len, encrypted_sig, &enc_sig_len, sizeof(encrypted_sig))) {
		ESP_LOGE(LOG_TAG, "Challenge encryption failed for conn %u", (unsigned)conn_id);
		ctx->len = 0;
		return;
	}

	/* Create LwM2MDeviceChallengeAnswer protobuf message */
	lwm2m_LwM2MDeviceChallengeAnswer answer = lwm2m_LwM2MDeviceChallengeAnswer_init_zero;
	
	/* Set the factory partition public key */
	if (s_factory_partition->public_key.size > 0 && s_factory_partition->public_key.size <= sizeof(answer.public_key.bytes)) {
		answer.public_key.size = s_factory_partition->public_key.size;
		memcpy(answer.public_key.bytes, s_factory_partition->public_key.bytes, answer.public_key.size);
	} else {
		ESP_LOGE(LOG_TAG, "Invalid factory public key size: %u", (unsigned)s_factory_partition->public_key.size);
		ctx->len = 0;
		return;
	}
	
	/* Set the encrypted signature */
	if (enc_sig_len <= sizeof(answer.signature.bytes)) {
		answer.signature.size = enc_sig_len;
		memcpy(answer.signature.bytes, encrypted_sig, enc_sig_len);
	} else {
		ESP_LOGE(LOG_TAG, "Encrypted signature too large: %u", (unsigned)enc_sig_len);
		ctx->len = 0;
		return;
	}

	/* Encode the protobuf message into ctx->value */
	pb_ostream_t ostream = pb_ostream_from_buffer(ctx->value, sizeof(ctx->value));
	if (pb_encode(&ostream, lwm2m_LwM2MDeviceChallengeAnswer_fields, &answer)) {
		ctx->len = (uint16_t)ostream.bytes_written;
		ESP_LOGI(LOG_TAG, "Challenge answer created for conn %u: pub_key_len=%u enc_sig_len=%u total_len=%u", 
				 (unsigned)conn_id, (unsigned)answer.public_key.size, (unsigned)enc_sig_len, (unsigned)ctx->len);
	} else {
		ESP_LOGE(LOG_TAG, "Failed to encode challenge answer protobuf for conn %u", (unsigned)conn_id);
		ctx->len = 0;
	}
}

