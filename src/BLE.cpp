/**
 * @brief BLE GATT server config.
 * 
 * @file BLE.cpp
 * @author Konstantin Klitenik
 * 
 * Copyright © 2019 Konstantin Klitenik. All rights reserved.
 */

#include <string.h>

#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatt_common_api.h"
#include "esp_gatts_api.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include "Logger.hpp"
#include "MAX31855.hpp"
#include "Utilities.hpp"
#include "BLE.hpp"

static const Logger log("BLE");

static const MAX31855 m_tc_probe_1(15);
static const MAX31855 m_tc_probe_2(4);

static constexpr uint8_t GATTS_SERVICE_UUID_COFFEE[] = { 0xFE, 0xCF };
static constexpr uint16_t GATTS_CHAR_UUID_TEMP_1    = 0xFE01;
static constexpr uint16_t GATTS_CHAR_UUID_TEMP_2    = 0xFE02;

static bool m_is_connected = false;
static TimerHandle_t m_temp_notify_timer;

#define PROFILE_NUM 			1
#define PROFILE_APP_IDX			0
#define APP_ID                  0x55
#define SVC_INST_ID             0
#define BLE_DEVICE_NAME         "CFE-SPY"

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))

static constexpr uint8_t SHORT_SERVICE_UUID[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    // uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, GATTS_SERVICE_UUID_COFFEE[0], GATTS_SERVICE_UUID_COFFEE[1], 0x00, 0x00,
};

// BLE adv and conn settings, optimized for iOS
// https://developer.apple.com/library/archive/qa/qa1931/_index.html
static constexpr int m_adv_min = 244;           // N * 0.625ms = 152.5ms
static constexpr int m_adv_max = 244;           // N * 0.625ms = 152.5ms
static constexpr int m_conn_min_interval = 36;  // N * 1.25ms = 45ms
static constexpr int m_conn_max_interval = 72;  // N * 1.25ms = 90ms
static constexpr int m_conn_slave_latency = 0;
static constexpr int m_conn_timeout = 400;      // N * 100ms = 4s

static constexpr esp_ble_adv_data_t m_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = m_conn_min_interval,
    .max_interval = m_conn_max_interval,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(SHORT_SERVICE_UUID),
    .p_service_uuid = const_cast<uint8_t *>(SHORT_SERVICE_UUID),
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static constexpr esp_ble_adv_params_t m_adv_params = {
    .adv_int_min       = m_adv_min,
    .adv_int_max       = m_adv_max,
    .adv_type          = ADV_TYPE_IND,
    .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
    .peer_addr         = { 0 },
    .peer_addr_type    = BLE_ADDR_TYPE_PUBLIC,
    .channel_map       = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

// bits for keeping track of adv/scan resp config
#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)
static uint8_t m_adv_config_done = 0;

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    // uint16_t app_id;
    // uint16_t conn_id;
    // uint16_t service_handle;
    // esp_gatt_srvc_id_t service_id;
    // uint16_t char_handle;
    // esp_bt_uuid_t char_uuid;
    // esp_gatt_perm_t perm;
    // esp_gatt_char_prop_t property;
    // uint16_t descr_handle;
    // esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static struct gatts_profile_inst m_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

// attributes state machine
enum {
    IDX_SVC,
    IDX_TEMP_1_CHAR,
    IDX_TEMP_1_VAL,
    IDX_TEMP_1_CHAR_CFG,
    IDX_TEMP_2_CHAR,
    IDX_TEMP_2_VAL,
    IDX_TEMP_2_CHAR_CFG,
    IDX_NUM_STATES,
};

static uint16_t m_handle_table[IDX_NUM_STATES];

static constexpr uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static constexpr uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static constexpr uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static constexpr uint8_t  char_prop_read               = ESP_GATT_CHAR_PROP_BIT_READ;
static constexpr uint8_t  char_prop_write              = ESP_GATT_CHAR_PROP_BIT_WRITE;
static constexpr uint8_t  char_prop_read_notify        = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static constexpr uint8_t  char_prop_read_write_notify  = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static constexpr uint8_t  temp_1_measurement_ccc[2]    = {0x00, 0x00};
static constexpr uint8_t  temp_2_measurement_ccc[2]    = {0x00, 0x00};

// characteristic values
static int32_t  temp_1_char_value = 0;
static int32_t  temp_2_char_value = 0;

/* Full Database Description - Used to add attributes into the database */
static constexpr esp_gatts_attr_db_t gatt_db[IDX_NUM_STATES] =
{
    // Service Declaration
    [IDX_SVC]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_COFFEE), (uint8_t *)&GATTS_SERVICE_UUID_COFFEE}},

    // Temp 1 Characteristic Declaration
    [IDX_TEMP_1_CHAR]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    // Temp 1 Characteristic Value
    [IDX_TEMP_1_VAL] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEMP_1, ESP_GATT_PERM_READ,
      sizeof(temp_1_char_value), sizeof(temp_1_char_value), (uint8_t *)&temp_1_char_value}},

    // Temp 1 Client Characteristic Configuration Descriptor
    [IDX_TEMP_1_CHAR_CFG]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(temp_1_measurement_ccc), (uint8_t *)temp_1_measurement_ccc}},
    
    [IDX_TEMP_2_CHAR]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    // Temp 2 Characteristic Value
    [IDX_TEMP_2_VAL] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEMP_2, ESP_GATT_PERM_READ,
      sizeof(temp_2_char_value), sizeof(temp_2_char_value), (uint8_t *)&temp_2_char_value}},

    // Temp 2 Client Characteristic Configuration Descriptor
    [IDX_TEMP_2_CHAR_CFG]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(temp_2_measurement_ccc), (uint8_t *)temp_2_measurement_ccc}},
};

typedef struct {
    esp_gatt_if_t gatts_if;
    uint16_t conn_id;
    bool need_confirm;
} notif_params_t;

static notif_params_t m_notif_params = { 0, 0, false };

static int32_t get_probe_temp(int probe_num) {
    Either<int, MAX31855::Error> result(INT32_MIN, MAX31855::Error::OK);

    switch(probe_num) {
        case 1:
            result = m_tc_probe_1.ReadTempC();
            break;
        case 2:
            result = m_tc_probe_2.ReadTempC();
            break;
        default:
            return INT32_MIN;
    }

    int32_t value = result.getError() == MAX31855::Error::OK ? result.getValue() : INT32_MIN;
    log.Verbose("TC Temp: %d C", value);
    return swap_byte_32(value);
}

static void temp_notify(TimerHandle_t timer) {
    temp_1_char_value = get_probe_temp(1);
    esp_ble_gatts_send_indicate(m_notif_params.gatts_if, m_notif_params.conn_id, m_handle_table[IDX_TEMP_1_VAL],
                                sizeof(temp_1_char_value), (uint8_t *)&temp_1_char_value, m_notif_params.need_confirm);

    temp_2_char_value = get_probe_temp(2);
    esp_ble_gatts_send_indicate(m_notif_params.gatts_if, m_notif_params.conn_id, m_handle_table[IDX_TEMP_2_VAL],
                                sizeof(temp_2_char_value), (uint8_t *)&temp_2_char_value, m_notif_params.need_confirm);
}

static void notif_start(esp_gatt_if_t gatts_if, uint16_t conn_id, bool need_confirm) {
    m_notif_params.gatts_if = gatts_if;
    m_notif_params.conn_id = conn_id;
    m_notif_params.need_confirm = need_confirm;

    // start timer, if not already active
    if (xTimerIsTimerActive(m_temp_notify_timer) == pdFALSE && xTimerStart(m_temp_notify_timer, 100) != pdPASS) {
        log.Error("Failed to start temp notify timer");
        return;
    }
};

static void notif_stop() {
    log.Verbose("Stop notifications");
    xTimerStop(m_temp_notify_timer, 100);
};

static void ble_on_read(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    // if no response is necessary (auto response by ESP), just return
    if (!param->read.need_rsp) {
        return;
    }
    
    esp_gatt_status_t gatt_status = ESP_GATT_OK;
    esp_gatt_rsp_t rsp;
    memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
    rsp.attr_value.handle = param->read.handle;

    if (param->read.handle == m_handle_table[IDX_TEMP_1_VAL]) {
        rsp.attr_value.len = sizeof(temp_1_char_value);
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 4;

        temp_1_char_value = get_probe_temp(1);
        memcpy(rsp.attr_value.value, &temp_1_char_value, sizeof(temp_1_char_value));
    } else if (param->read.handle == m_handle_table[IDX_TEMP_2_VAL]) {
        rsp.attr_value.len = sizeof(temp_2_char_value);
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 4;

        temp_2_char_value = get_probe_temp(2);
        memcpy(rsp.attr_value.value, &temp_2_char_value, sizeof(temp_2_char_value));
    } else {
        gatt_status = ESP_GATT_INVALID_HANDLE;
    }

    esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    gatt_status, &rsp);
}

static void ble_on_write(esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    esp_gatt_status_t gatt_status = ESP_GATT_OK;

    log.Debug("ON WRITE: is_prep %d  need_rsp %d", param->write.is_prep, param->write.need_rsp);

    // right now we only handle CFG writes which don't require prep or response
    if (!param->write.is_prep) {
        // the data length of gattc write  must be less than max characteristic length
        if (param->write.handle == m_handle_table[IDX_TEMP_1_CHAR_CFG] && param->write.len == 2) {
            uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
            if (descr_value == 0x0001) {
                log.Info("notify enable");
                notif_start(gatts_if, param->write.conn_id, false);
            } else if (descr_value == 0x0002) {
                log.Info("indicate enable");
                notif_start(gatts_if, param->write.conn_id, true);
            } else if (descr_value == 0x0000) {
                log.Info("notify/indicate disable ");
                notif_stop();
            } else {
                log.Error("unknown descr value");
            }
        } else {
            gatt_status = ESP_GATT_INVALID_HANDLE;
        }
        
        // send response when response is requested and not handled by ESP
        if (param->write.need_rsp) {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, gatt_status, NULL);
        }
    } else {
        // handle prep write
        // currently nothing to do
    }
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        m_adv_config_done &= (~ADV_CONFIG_FLAG);
        if (m_adv_config_done == 0) {
            esp_ble_gap_start_advertising(const_cast<esp_ble_adv_params_t *>(&m_adv_params));
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        log.Warn("Scan response set complete (shouldn't be here)");
        break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            log.Error("Advertising start failed");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            log.Error("Advertising stop failed");
        }
        else {
            log.Error("Stop adv successfully");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         log.Info("Update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT: {
        // log.Info("REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        esp_err_t status = esp_ble_gap_set_device_name(BLE_DEVICE_NAME);
        if (status != ESP_OK) {
            log.Error("set device name failed, error code = %x", status);
        }
        //config adv data
        status = esp_ble_gap_config_adv_data(const_cast<esp_ble_adv_data_t *>(&m_adv_data));
        if (status != ESP_OK) {
            log.Error("config adv data failed, error code = %x", status);
        }
        m_adv_config_done |= ADV_CONFIG_FLAG;
        
        status = esp_ble_gatts_create_attr_tab(gatt_db, gatts_if, IDX_NUM_STATES, SVC_INST_ID);
        if (status != ESP_OK) {
            log.Error("create attr table failed, error code = %x", status);
        }
        break;
    }
    case ESP_GATTS_READ_EVT: {
        log.Info("GATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
        ble_on_read(gatts_if, param);
        break;
    }
    case ESP_GATTS_WRITE_EVT:
        log.Warn("GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
        ble_on_write(gatts_if, param);
        break;
    case ESP_GATTS_EXEC_WRITE_EVT:
        log.Warn("ESP_GATTS_EXEC_WRITE_EVT");
        break;
    case ESP_GATTS_MTU_EVT:
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        // log.Info("SERVICE_START_EVT, status %d, service_handle %d\n", param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT: {
        esp_ble_conn_update_params_t conn_params;

        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));  // set BT address
        conn_params.latency = m_conn_slave_latency;
        conn_params.max_int = m_conn_max_interval;
        conn_params.min_int = m_conn_min_interval;
        conn_params.timeout = m_conn_timeout;

        log.Info("ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        
        esp_ble_gap_update_conn_params(&conn_params);   // send the updated connection parameters to the peer device.
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
        log.Info("ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
        notif_stop();
        esp_ble_gap_start_advertising(const_cast<esp_ble_adv_params_t *>(&m_adv_params));
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status != ESP_GATT_OK) {
            log.Error("create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        } else if (param->add_attr_tab.num_handle != IDX_NUM_STATES) {
            log.Error("create attribute table abnormally, num_handle (%d) doesn't equal to IDX_NUM_STATES(%d)",
                     param->add_attr_tab.num_handle, IDX_NUM_STATES);
        } else {
            // log.Info("create attribute table successfully, the number handle = %d\n", param->add_attr_tab.num_handle);
            memcpy(m_handle_table, param->add_attr_tab.handles, sizeof(m_handle_table));
            esp_ble_gatts_start_service(m_handle_table[IDX_SVC]);
        }
        break;
    case ESP_GATTS_CONF_EVT:
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    // if register event, store the gatts_if for each profile
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            m_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            log.Info("Reg app failed, app_id %04x, status %d\n", param->reg.app_id, param->reg.status);
            return;
        }
    }


    for (int idx = 0; idx < PROFILE_NUM; idx++) {
        // if ESP_GATT_IF_NONE, then gatt_if wasn't specified, need to call every profile's cb function
        if (gatts_if == ESP_GATT_IF_NONE || gatts_if == m_profile_tab[idx].gatts_if) {
            if (m_profile_tab[idx].gatts_cb) {
                m_profile_tab[idx].gatts_cb(event, gatts_if, param);
            }
        }
    }
}

void ble_init() {
    esp_err_t status;
    
    m_is_connected  = false;

    // release BT Classic memory, we don't use it
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    if (esp_bt_controller_init(&bt_cfg) != ESP_OK) {
        log.Error("BLE ctrl init failed");
        return;
    }

    status = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (status != ESP_OK) {
        log.Error("BLE ctrl enable failed: %s", esp_err_to_name(status));
        return;
    }

    log.Info("Initializing BLE");
    status = esp_bluedroid_init();
    if (status != ESP_OK) {
        log.Error("Bluedroid init failed: %s", esp_err_to_name(status));
        return;
    }

    status = esp_bluedroid_enable();
    if (status != ESP_OK) {
        log.Error("Bluedroid enable failed: %s", esp_err_to_name(status));
        return;
    }

    status = esp_ble_gatts_register_callback(gatts_event_handler);
    if (status != ESP_OK) {
        log.Error("GATTS register callback failed: %s", esp_err_to_name(status));
        return;
    }
    
    status = esp_ble_gap_register_callback(gap_event_handler);
    if (status != ESP_OK) {
        log.Error("GAP register callback failed: %s", esp_err_to_name(status));
        return;
    }
    
    status = esp_ble_gatts_app_register(APP_ID);
    if (status != ESP_OK) {
        log.Error("Reg app failed: %s", esp_err_to_name(status));
        return;
    }

    // set MTU
    status = esp_ble_gatt_set_local_mtu(200);
    if (status != ESP_OK) {
        log.Error("Failed MTU set: %s", esp_err_to_name(status));
    }

    m_temp_notify_timer = xTimerCreate("TempNotify", 1000 / portTICK_PERIOD_MS, pdTRUE, NULL, temp_notify);
    if (!m_temp_notify_timer) {
        log.Error("Failed to create temp notify timer");
        return;
    }

    return;
}

void ble_enable() {
    esp_err_t status = esp_bluedroid_enable();
    if (status != ESP_OK) {
        log.Error("BLE enable failed: %s", esp_err_to_name(status));
        return;
    }
}

void ble_disable() {
    esp_bluedroid_disable();
}

bool ble_is_connected() {
    return m_is_connected;
}