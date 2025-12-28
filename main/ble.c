#include "gnarl.h"

#include <string.h>
#include <unistd.h>

#include <esp_mac.h>
#include <esp_nimble_hci.h>
#include <esp_pm.h>
#include <esp_timer.h>
#include <host/ble_gap.h>
#include <host/util/util.h>
#include <nimble/nimble_port.h>
#include <nimble/nimble_port_freertos.h>
#include <os/os_mbuf.h>
#include <nvs.h>
#include <nvs_flash.h>
#include <services/gap/ble_svc_gap.h>
#include <services/gatt/ble_svc_gatt.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "adc.h"
#include "commands.h"

#define MAX_DATA 150

#define DEFAULT_NAME "GNARL"

#define CUSTOM_NAME_SIZE 30
#define STORAGE_NAMESPACE "GNARL"

static uint8_t custom_name[CUSTOM_NAME_SIZE];

void ble_store_ram_init(void);

#define B0(x) ((x) & 0xFF)
#define B1(x) (((x) >> 8) & 0xFF)
#define B2(x) (((x) >> 16) & 0xFF)
#define B3(x) (((x) >> 24) & 0xFF)
#define B4(x) (((x) >> 32) & 0xFF)
#define B5(x) (((x) >> 40) & 0xFF)

#define UUID128_CONST(a32, b16, c16, d16, e48)                \
	BLE_UUID128_INIT(                                         \
		B0(e48), B1(e48), B2(e48), B3(e48), B4(e48), B5(e48), \
		B0(d16), B1(d16),                                     \
		B0(c16), B1(c16),                                     \
		B0(b16), B1(b16),                                     \
		B0(a32), B1(a32), B2(a32), B3(a32), )

static ble_uuid128_t service_uuid = UUID128_CONST(0x0235733b, 0x99c5, 0x4197, 0xb856, 0x69219c2a3845);
static ble_uuid128_t data_uuid = UUID128_CONST(0xc842e849, 0x5028, 0x42e2, 0x867c, 0x016adada9155);
static ble_uuid128_t response_count_uuid = UUID128_CONST(0x6e6c7910, 0xb89e, 0x43a5, 0xa0fe, 0x50c5e2b81f4a);
static ble_uuid128_t timer_tick_uuid = UUID128_CONST(0x6e6c7910, 0xb89e, 0x43a5, 0x78af, 0x50c5e2b86f7e);
static ble_uuid128_t custom_name_uuid = UUID128_CONST(0xd93b2af0, 0x1e28, 0x11e4, 0x8c21, 0x0800200c9a66);
static ble_uuid128_t firmware_version_uuid = UUID128_CONST(0x30d99dc9, 0x7c91, 0x4295, 0xa051, 0x0a104d238cf2);
static ble_uuid128_t led_mode_uuid = UUID128_CONST(0xc6d84241, 0xf1a7, 0x4f9c, 0xa25f, 0xfce16732f14e);

static ble_uuid16_t battery_service_uuid = BLE_UUID16_INIT(0x180F);
static ble_uuid16_t battery_level_uuid = BLE_UUID16_INIT(0x2A19);

static ble_gap_event_fn handle_gap_event;
static uint8_t addr_type;
static ble_gatt_access_fn data_access;
static ble_gatt_access_fn custom_name_access;
static ble_gatt_access_fn led_mode_access;
static ble_gatt_access_fn firmware_version_access;
static ble_gatt_access_fn no_access;

static ble_gatt_access_fn battery_level_access;

static bool connected;
static uint16_t connection_handle;

static esp_pm_lock_handle_t ble_no_light_sleep_lock;

bool ble_is_connected(void)
{
	return connected;
}

static void ble_set_no_light_sleep(bool enabled)
{
	if (ble_no_light_sleep_lock == NULL)
	{
		return;
	}

	if (enabled)
	{
		esp_err_t err = esp_pm_lock_acquire(ble_no_light_sleep_lock);
		if (err != ESP_OK)
		{
			ESP_LOGW(TAG, "esp_pm_lock_acquire(NO_LIGHT_SLEEP) failed: %s", esp_err_to_name(err));
		}
	}
	else
	{
		esp_err_t err = esp_pm_lock_release(ble_no_light_sleep_lock);
		if (err != ESP_OK)
		{
			ESP_LOGW(TAG, "esp_pm_lock_release(NO_LIGHT_SLEEP) failed: %s", esp_err_to_name(err));
		}
	}
}

static uint16_t response_count_notify_handle;
static int response_count_notify_state;
static uint8_t response_count;

static uint16_t timer_tick_notify_handle;
static int timer_tick_notify_state;
static uint8_t timer_tick;
static void timer_tick_callback(void *);

static uint16_t battery_level_notify_handle;
static int battery_level_notify_state;

static const struct ble_gatt_svc_def service_list[] = {
	{
		.type = BLE_GATT_SVC_TYPE_PRIMARY,
		.uuid = &service_uuid.u,
		.characteristics = (struct ble_gatt_chr_def[]){
			{
				.uuid = &data_uuid.u,
				.access_cb = data_access,
				.flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
			},
			{
				.uuid = &response_count_uuid.u,
				.access_cb = no_access,
				.val_handle = &response_count_notify_handle,
				.flags = BLE_GATT_CHR_F_NOTIFY,
			},
			{
				.uuid = &timer_tick_uuid.u,
				.access_cb = no_access,
				.val_handle = &timer_tick_notify_handle,
				.flags = BLE_GATT_CHR_F_NOTIFY,
			},
			{
				.uuid = &custom_name_uuid.u,
				.access_cb = custom_name_access,
				.flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
			},
			{
				.uuid = &firmware_version_uuid.u,
				.access_cb = firmware_version_access,
				.flags = BLE_GATT_CHR_F_READ,
			},
			{
				.uuid = &led_mode_uuid.u,
				.access_cb = led_mode_access,
				.flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE,
			},
			{.uuid = NULL}},
	},
	{
		.type = BLE_GATT_SVC_TYPE_PRIMARY,
		.uuid = &battery_service_uuid.u,
		.characteristics = (struct ble_gatt_chr_def[]){
			{
				.uuid = &battery_level_uuid.u,
				.access_cb = battery_level_access,
				.val_handle = &battery_level_notify_handle,
				.flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY,
			},
			{.uuid = NULL},
		},
	},
	{
		.type = BLE_GATT_SVC_TYPE_END,
	},
};

static void server_init(void)
{
	int err;
	char u[60];
	ESP_LOGI(TAG, "BLE server_init");

	ble_svc_gap_init();
	ble_svc_gatt_init();

	err = ble_gatts_count_cfg(service_list);
	assert(!err);

	err = ble_gatts_add_svcs(service_list);
	assert(!err);
	ESP_LOGI(TAG, "BLE services registered");

	ble_uuid_to_str(&service_uuid.u, u);
	ESP_LOGD(TAG, "service UUID %s", u);

	esp_timer_handle_t t;
	esp_timer_create_args_t timer_args = {
		.callback = timer_tick_callback,
	};
	ESP_ERROR_CHECK(esp_timer_create(&timer_args, &t));
	ESP_ERROR_CHECK(esp_timer_start_periodic(t, 60 * SECONDS));
}

static void advertise(void)
{
	ESP_LOGI(TAG, "BLE advertise(): starting advertisement setup");
	// While advertising, keep light sleep disabled so iAPS can reliably discover us.
	ble_set_no_light_sleep(true);
	ESP_LOGD(TAG, "BLE light sleep disabled for advertising");

	// Make advertising restart resilient: don't rely on asserts.
	struct ble_hs_adv_fields fields, fields_ext;
	char short_name[6]; // 5 plus zero byte
	memset(&fields, 0, sizeof(fields));
	memset(&fields_ext, 0, sizeof(fields_ext));

	fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

	fields.tx_pwr_lvl_is_present = 1;
	fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;

	const char *name = ble_svc_gap_device_name();
	strncpy(short_name, name, sizeof(short_name));
	short_name[sizeof(short_name) - 1] = 0;
	fields.name = (uint8_t *)short_name;
	fields.name_len = strlen(short_name);
	if (strlen(name) <= 5)
	{
		fields.name_is_complete = 1;
	}
	else
	{
		fields.name_is_complete = 0;
		ESP_LOGD(TAG, "device name shortened to %s", short_name);
	}

	fields.uuids128 = &service_uuid;
	fields.num_uuids128 = 1;
	fields.uuids128_is_complete = 1;

	ESP_LOGD(TAG, "BLE setting advertisement fields");
	int err = ble_gap_adv_set_fields(&fields);
	if (err)
	{
		ESP_LOGE(TAG, "ble_gap_adv_set_fields err %d", err);
		ESP_LOGE(TAG, "BLE advertisement setup failed, releasing light sleep lock");
		// Don't hold the lock if we aren't actually advertising.
		ble_set_no_light_sleep(false);
		return;
	}

	if (!fields.name_is_complete)
	{
		// Include the complete device name in the scan response.
		fields_ext.flags = fields.flags;
		fields_ext.name = (uint8_t *)name;
		fields_ext.name_len = strlen(name);
		fields_ext.name_is_complete = 1;
		err = ble_gap_adv_rsp_set_fields(&fields_ext);
		if (err)
		{
			ESP_LOGE(TAG, "ble_gap_adv_rsp_set_fields: name might be too long, err %d", err);
		}
	}
	ESP_LOGI(TAG, "BLE advertising: name='%s' short='%s'", name, short_name);

	// Begin advertising.
	struct ble_gap_adv_params adv;
	memset(&adv, 0, sizeof(adv));
	adv.conn_mode = BLE_GAP_CONN_MODE_UND;
	adv.disc_mode = BLE_GAP_DISC_MODE_GEN;

	err = ble_gap_adv_start(addr_type, 0, BLE_HS_FOREVER, &adv, handle_gap_event, 0);
	if (err)
	{
		// Common after quick reconnects: stack is still unwinding previous state.
		const char* err_str = "UNKNOWN";
		switch(err) {
			case 0x0d: err_str = "BLE_HS_EALREADY (already advertising)"; break;
			case 0x02: err_str = "BLE_HS_EINVAL (invalid params)"; break;
			case 0x0e: err_str = "BLE_HS_EBUSY (busy)"; break;
		}
		ESP_LOGW(TAG, "ble_gap_adv_start failed: %s (err=%d)", err_str, err);
		ESP_LOGI(TAG, "Stopping previous advertising and will retry on ADV_COMPLETE");
		int stop_err = ble_gap_adv_stop();
		if (stop_err != 0) {
			ESP_LOGW(TAG, "ble_gap_adv_stop also failed: err=%d", stop_err);
		}
		// Don't hold the lock if adv did not start.
		ble_set_no_light_sleep(false);
		return;
	}

	ESP_LOGI(TAG, "BLE advertising started successfully");
}

static int handle_gap_event(struct ble_gap_event *e, void *arg)
{
	switch (e->type)
	{
	case BLE_GAP_EVENT_CONNECT:
		ESP_LOGI(TAG, "BLE GAP_EVENT_CONNECT: status=%d, conn_handle=0x%04X", 
		         e->connect.status, e->connect.conn_handle);
		if (e->connect.status != 0)
		{
			ESP_LOGE(TAG, "BLE connect failed, status=%d", e->connect.status);
			advertise();
			return 0;
		}
		connected = true;
		connection_handle = e->connect.conn_handle;
		
		// Once connected, allow light sleep again.
		ble_set_no_light_sleep(false);
		// Force client to re-subscribe each connection.
		response_count_notify_state = 0;
		timer_tick_notify_state = 0;
		battery_level_notify_state = 0;
		int8_t rssi;
		ble_gap_conn_rssi(connection_handle, &rssi);
		set_ble_rssi(rssi);

		ESP_LOGI(TAG, "BLE connected successfully: handle=0x%04X, RSSI=%d dBm", connection_handle, (int)rssi);
		ESP_LOGD(TAG, "response count notify handle 0x%04X", response_count_notify_handle);
		ESP_LOGD(TAG, "timer tick notify handle 0x%04X", timer_tick_notify_handle);
		break;
	case BLE_GAP_EVENT_DISCONNECT:
		ESP_LOGI(TAG, "BLE GAP_EVENT_DISCONNECT: reason=0x%03x (was handle=0x%04X)", 
		         e->disconnect.reason, connection_handle);
		connected = false;
		connection_handle = 0;
		response_count_notify_state = 0;
		timer_tick_notify_state = 0;
		battery_level_notify_state = 0;
		gnarl_cancel_current_command();
		set_ble_disconnected();
		
		// Decode disconnect reason for better debugging
		const char* reason_str = "UNKNOWN";
		switch(e->disconnect.reason) {
			case 0x08: reason_str = "SUPERVISION_TIMEOUT"; break;
			case 0x13: reason_str = "REMOTE_USER_TERMINATED"; break;
			case 0x16: reason_str = "LOCAL_HOST_TERMINATED"; break;
			case 0x208: reason_str = "BLE_HS_ETIMEOUT"; break;
			case 0x20d: reason_str = "BLE_HS_EDONE"; break;
		}
		ESP_LOGI(TAG, "BLE disconnect reason: %s (0x%03x)", reason_str, e->disconnect.reason);
		ESP_LOGI(TAG, "BLE restarting advertisement after short delay...");
		
		// Small delay to allow BLE stack to clean up before restarting advertising
		// This helps prevent "already advertising" or "busy" errors on quick reconnects
		vTaskDelay(pdMS_TO_TICKS(100));
		
		advertise();
		break;
	case BLE_GAP_EVENT_ADV_COMPLETE:
		ESP_LOGI(TAG, "BLE advertising complete, reason=0x%x", e->adv_complete.reason);
		const char* adv_reason_str = "UNKNOWN";
		switch(e->adv_complete.reason) {
			case 0: adv_reason_str = "SUCCESS/CONNECTED"; break;
			case 0x0d: adv_reason_str = "TIMEOUT"; break;
			case 0x208: adv_reason_str = "STOPPED"; break;
		}
		ESP_LOGI(TAG, "BLE adv complete reason: %s - restarting", adv_reason_str);
		advertise();
		break;
	case BLE_GAP_EVENT_SUBSCRIBE:
		ESP_LOGI(TAG, "BLE subscribe: conn_handle=0x%04X, attr=0x%04X, notify=%d", 
		         e->subscribe.conn_handle, e->subscribe.attr_handle, e->subscribe.cur_notify);
		
		if (e->subscribe.attr_handle == response_count_notify_handle)
		{
			ESP_LOGI(TAG, "BLE: Client %s response count notifications", 
			         e->subscribe.cur_notify ? "enabled" : "disabled");
			response_count_notify_state = e->subscribe.cur_notify;
			break;
		}
		if (e->subscribe.attr_handle == timer_tick_notify_handle)
		{
			ESP_LOGI(TAG, "BLE: Client %s timer tick notifications", 
			         e->subscribe.cur_notify ? "enabled" : "disabled");
			timer_tick_notify_state = e->subscribe.cur_notify;
			break;
		}
		if (e->subscribe.attr_handle == battery_level_notify_handle)
		{
			ESP_LOGI(TAG, "BLE: Client %s battery level notifications", 
			         e->subscribe.cur_notify ? "enabled" : "disabled");
			battery_level_notify_state = e->subscribe.cur_notify;
			break;
		}
		ESP_LOGD(TAG, "notify %d for unknown handle %04X", e->subscribe.cur_notify, e->subscribe.attr_handle);
		break;
	default:
		ESP_LOGD(TAG, "GAP event %d", e->type);
		break;
	}
	return 0;
}

static void sync_callback(void)
{
	int err;
	ESP_LOGI(TAG, "BLE sync_callback");

	err = ble_hs_util_ensure_addr(0);
	assert(!err);

	err = ble_hs_id_infer_auto(0, &addr_type);
	assert(!err);

	uint8_t addr[6];
	ble_hs_id_copy_addr(addr_type, addr, 0);

	ESP_LOGD(TAG, "device address: %02x:%02x:%02x:%02x:%02x:%02x",
			 addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
	ESP_LOGI(TAG, "BLE addr: %02x:%02x:%02x:%02x:%02x:%02x",
			 addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);

	ESP_LOGI(TAG, "BLE starting advertising");
	advertise();
}

static uint8_t data_in[MAX_DATA];
static uint16_t data_in_len;

static uint8_t data_out[MAX_DATA];
static uint16_t data_out_len;

static void response_notify(void)
{
	response_count++;
	ESP_LOGD(TAG, "BLE response_notify(): response_count=%d, connected=%d, notify_state=%d, handle=0x%04X", 
	         response_count, connected, response_count_notify_state, connection_handle);
	if (!connected || !response_count_notify_state)
	{
		ESP_LOGD(TAG, "not notifying for response count %d (connected=%d, state=%d)", 
		         response_count, connected, response_count_notify_state);
		return;
	}
	struct os_mbuf *om = ble_hs_mbuf_from_flat(&response_count, sizeof(response_count));
	ESP_LOGD(TAG, "BLE sending notification for response count %d", response_count);
	int err = ble_gattc_notify_custom(connection_handle, response_count_notify_handle, om);
	if (err)
	{
		os_mbuf_free_chain(om);
		ESP_LOGW(TAG, "response notify failed err=%d (connected=%d)", err, connected);
		return;
	}
	ESP_LOGD(TAG, "BLE notification sent successfully: response_count=%d", response_count);
}

void send_code(const uint8_t code)
{
	ESP_LOGD(TAG, "BLE send_code(): sending code 0x%02X to client", code);
	data_out[0] = code;
	data_out_len = 1;
	ESP_LOG_BUFFER_HEX_LEVEL(TAG, data_out, data_out_len, ESP_LOG_DEBUG);
	response_notify();
}

void send_bytes(const uint8_t *buf, int count)
{
	ESP_LOGD(TAG, "BLE send_bytes(): sending %d bytes to client", count);
	data_out[0] = RESPONSE_CODE_SUCCESS;
	memcpy(data_out + 1, buf, count);
	data_out_len = count + 1;
	ESP_LOGD(TAG, "BLE total data length with response code: %d bytes", data_out_len);
	ESP_LOG_BUFFER_HEX_LEVEL(TAG, data_out, data_out_len, ESP_LOG_DEBUG);
	response_notify();
}

static void timer_tick_callback(void *arg)
{
	timer_tick++;
	ESP_LOGD(TAG, "timer tick %d (connected=%d, handle=0x%04X)", timer_tick, connected, connection_handle);

	// Periodic battery level notification (standard Battery Service 0x180F).
	if (connected && battery_level_notify_state)
	{
		uint8_t battery_level = battery_percent(get_battery_voltage());
		struct os_mbuf *om_batt = ble_hs_mbuf_from_flat(&battery_level, sizeof(battery_level));
		int err_batt = ble_gattc_notify_custom(connection_handle, battery_level_notify_handle, om_batt);
		if (err_batt)
		{
			os_mbuf_free_chain(om_batt);
			ESP_LOGW(TAG, "battery level notify failed err=%d (connected=%d, handle=0x%04X)", 
			         err_batt, connected, connection_handle);
		}
	}

	if (!connected || !timer_tick_notify_state)
	{
		if (connected)
		{
			ESP_LOGD(TAG, "not notifying for timer tick");
		}
		return;
	}
	struct os_mbuf *om = ble_hs_mbuf_from_flat(&timer_tick, sizeof(timer_tick));
	int err = ble_gattc_notify_custom(connection_handle, timer_tick_notify_handle, om);
	if (err)
	{
		os_mbuf_free_chain(om);
		ESP_LOGW(TAG, "timer tick notify failed err=%d (connected=%d)", err, connected);
		return;
	}
	ESP_LOGD(TAG, "notify for timer tick");
}

static int data_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
	int err;
	int8_t rssi;
	assert(ble_uuid_cmp(ctxt->chr->uuid, &data_uuid.u) == 0);
	switch (ctxt->op)
	{
	case BLE_GATT_ACCESS_OP_READ_CHR:
		ESP_LOGD(TAG, "BLE data_access READ: sending %d bytes from pump to phone", data_out_len);
		if (data_out_len > 0) {
			ESP_LOG_BUFFER_HEX_LEVEL(TAG, data_out, data_out_len, ESP_LOG_DEBUG);
		}
		if (os_mbuf_append(ctxt->om, data_out, data_out_len) != 0)
		{
			ESP_LOGE(TAG, "BLE data_access: insufficient resources for %d bytes", data_out_len);
			return BLE_ATT_ERR_INSUFFICIENT_RES;
		}
		ESP_LOGD(TAG, "BLE data sent successfully");
		return 0;
	case BLE_GATT_ACCESS_OP_WRITE_CHR:
		err = ble_hs_mbuf_to_flat(ctxt->om, data_in, sizeof(data_in), &data_in_len);
		assert(!err);
		ESP_LOGD(TAG, "BLE data_access WRITE: command received, %d bytes", data_in_len);
		ESP_LOG_BUFFER_HEX_LEVEL(TAG, data_in, data_in_len, ESP_LOG_DEBUG);
		ble_gap_conn_rssi(conn_handle, &rssi);
		ESP_LOGD(TAG, "BLE RSSI: %d dBm", (int)rssi);
		ESP_LOGD(TAG, "BLE calling rfspy_command with %d bytes", data_in_len);
		rfspy_command(data_in, data_in_len, (int)rssi);
		return 0;
	default:
		assert(0);
	}
	return 0;
}

static void read_custom_name(void)
{
	ESP_LOGD(TAG, "read_custom_name from nvs");
	nvs_handle my_handle;
	esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "read_custom_name: nvs_open: %s", esp_err_to_name(err));
		return;
	}
	size_t required_size = CUSTOM_NAME_SIZE;
	err = nvs_get_blob(my_handle, "custom_name", custom_name, &required_size);
	if (err == ESP_ERR_NVS_NOT_FOUND)
	{
		// Generate unique name using last 4 hex digits of MAC address
		uint8_t mac[6];
		esp_read_mac(mac, ESP_MAC_BT);
		snprintf((char *)custom_name, CUSTOM_NAME_SIZE, "%s-%02X%02X", DEFAULT_NAME, mac[4], mac[5]);
		ESP_LOGD(TAG, "set default custom name with MAC: %s", custom_name);
	}
	else if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "read_custom_name: nvs_get_blob: %s", esp_err_to_name(err));
		// Generate unique name using last 4 hex digits of MAC address
		uint8_t mac[6];
		esp_read_mac(mac, ESP_MAC_BT);
		snprintf((char *)custom_name, CUSTOM_NAME_SIZE, "%s-%02X%02X", DEFAULT_NAME, mac[4], mac[5]);
		ESP_LOGD(TAG, "fallback to default custom name with MAC: %s", custom_name);
	}
	else
	{
		ESP_LOGD(TAG, "read_custom_name success: %s", custom_name);
	}
	nvs_close(my_handle);
}

static void write_custom_name(void)
{
	ESP_LOGD(TAG, "write_custom_name to nvs");
	nvs_handle my_handle;
	esp_err_t err = nvs_open(STORAGE_NAMESPACE, NVS_READWRITE, &my_handle);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "write_custom_name: nvs_open: %s", esp_err_to_name(err));
		return;
	}
	err = nvs_set_blob(my_handle, "custom_name", custom_name, CUSTOM_NAME_SIZE);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "write_custom_name: nvs_set_blob: %s", esp_err_to_name(err));
		nvs_close(my_handle);
		return;
	}
	err = nvs_commit(my_handle);
	if (err != ESP_OK)
	{
		ESP_LOGE(TAG, "write_custom_name: nvs_commit: %s", esp_err_to_name(err));
	}
	nvs_close(my_handle);
}

static int custom_name_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
	int err;
	uint16_t custom_name_len = strlen((char *)custom_name);
	assert(custom_name_len <= sizeof(custom_name));
	assert(ble_uuid_cmp(ctxt->chr->uuid, &custom_name_uuid.u) == 0);
	switch (ctxt->op)
	{
	case BLE_GATT_ACCESS_OP_READ_CHR:
		ESP_LOGD(TAG, "custom_name_access: sending %s to phone", custom_name);
		if (os_mbuf_append(ctxt->om, custom_name, custom_name_len) != 0)
		{
			return BLE_ATT_ERR_INSUFFICIENT_RES;
		}
		return 0;
	case BLE_GATT_ACCESS_OP_WRITE_CHR:
		err = ble_hs_mbuf_to_flat(ctxt->om, custom_name, sizeof(custom_name), &custom_name_len);
		custom_name[custom_name_len] = 0;
		ESP_LOGD(TAG, "custom_name_access: received %s from phone", custom_name);
		assert(!err);
		write_custom_name();
		esp_restart();
		return 0;
	default:
		assert(0);
	}
	return 0;
}

static int firmware_version_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
	assert(ble_uuid_cmp(ctxt->chr->uuid, &firmware_version_uuid.u) == 0);
	assert(ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR);
	ESP_LOGD(TAG, "BLE firmware version = %s", BLE_RFSPY_VERSION);
	if (os_mbuf_append(ctxt->om, (const uint8_t *)BLE_RFSPY_VERSION, strlen(BLE_RFSPY_VERSION)) != 0)
	{
		return BLE_ATT_ERR_INSUFFICIENT_RES;
	}
	return 0;
}

static int battery_level_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
	assert(ble_uuid_cmp(ctxt->chr->uuid, &battery_level_uuid.u) == 0);
	assert(ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR);
	uint8_t battery_level = battery_percent(get_battery_voltage());
	ESP_LOGD(TAG, "battery_level_access: %d%%", battery_level);
	if (os_mbuf_append(ctxt->om, &battery_level, sizeof(battery_level)) != 0)
	{
		return BLE_ATT_ERR_INSUFFICIENT_RES;
	}
	return 0;
}

static uint8_t led_mode;

static int led_mode_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
	int err;
	uint16_t n;
	assert(ble_uuid_cmp(ctxt->chr->uuid, &led_mode_uuid.u) == 0);
	switch (ctxt->op)
	{
	case BLE_GATT_ACCESS_OP_READ_CHR:
		ESP_LOGD(TAG, "led_mode_access: mode = %d", led_mode);
		if (os_mbuf_append(ctxt->om, &led_mode, sizeof(led_mode)))
		{
			return BLE_ATT_ERR_INSUFFICIENT_RES;
		}
		return 0;
	case BLE_GATT_ACCESS_OP_WRITE_CHR:
		err = ble_hs_mbuf_to_flat(ctxt->om, &led_mode, sizeof(led_mode), &n);
		assert(!err);
		ESP_LOGD(TAG, "led_mode_access: set mode = %d", led_mode);
		return 0;
	default:
		assert(0);
	}
	return 0;
}

static int no_access(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
	char u[60];
	ble_uuid_to_str(ctxt->chr->uuid, u);
	ESP_LOGE(TAG, "should not happen: op %d, attr handle %04X, uuid %s", ctxt->op, attr_handle, u);
	return 0;
}

static void host_task(void *arg)
{
	nimble_port_run();
}

void gnarl_init(void)
{
	start_gnarl_task();
	ESP_LOGI(TAG, "gnarl_init: NVS + NimBLE");

	// Create a PM lock to keep advertising discoverable while still allowing
	// light sleep when connected/idle.
	esp_err_t pm_err = esp_pm_lock_create(ESP_PM_NO_LIGHT_SLEEP, 0, "ble_adv", &ble_no_light_sleep_lock);
	if (pm_err != ESP_OK)
	{
		ble_no_light_sleep_lock = NULL;
		ESP_LOGW(TAG, "esp_pm_lock_create(NO_LIGHT_SLEEP) failed: %s", esp_err_to_name(pm_err));
	}

	ESP_ERROR_CHECK(nvs_flash_init());
	nimble_port_init();

	ble_hs_cfg.sync_cb = sync_callback;

	server_init();

	read_custom_name();

	int err = ble_svc_gap_device_name_set((char *)custom_name);
	assert(!err);
	ESP_LOGI(TAG, "BLE device name set to '%s'", custom_name);

	ble_store_ram_init();
	nimble_port_freertos_init(host_task);
	ESP_LOGI(TAG, "NimBLE host task started");
}
