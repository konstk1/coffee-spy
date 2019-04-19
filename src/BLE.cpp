/**
 * @brief BLE GATTS server config.
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

// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/event_groups.h"

#include "Logger.h"
#include "ble.h"

static constexpr uint16_t GATTS_SERVICE_UUID_COFFEE = 0xCFFE;
static constexpr uint16_t GATTS_CHAR_UUID_TEMP_1    = 0xFE01;

static bool m_is_connected = false;

#define PROFILE_NUM 			1
#define PROFILE_APP_IDX			0
#define APP_ID                  0x55
#define SVC_INST_ID             0
#define BLE_DEVICE_NAME         "CFE-SPY"

#define PREPARE_BUF_MAX_SIZE    1024
#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))

static const Logger log("BLE");

static uint8_t short_service_uuid[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    // uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0x01, 0xEE, 0xFF, 0xC0,
};

static esp_ble_adv_data_t m_adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = true,
    .min_interval = 0x0006,    // LSB 1.25ms
    .max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,
    .p_manufacturer_data =  NULL,
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(short_service_uuid),
    .p_service_uuid = short_service_uuid,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

static esp_ble_adv_params_t m_adv_params = {
    .adv_int_min       = 160,      // N * .625ms = 100ms
    .adv_int_max       = 320,      // N * .625ms = 200ms
    .adv_type          = ADV_TYPE_IND,
    .own_addr_type     = BLE_ADDR_TYPE_PUBLIC,
    .peer_addr         = { 0 },
    .peer_addr_type    = BLE_ADDR_TYPE_PUBLIC,
    .channel_map       = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#define ADV_CONFIG_FLAG             (1 << 0)
#define SCAN_RSP_CONFIG_FLAG        (1 << 1)

static uint8_t m_adv_config_done = 0;

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

static struct gatts_profile_inst m_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_IDX] = {
        .gatts_cb = gatts_profile_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

typedef struct {
    uint8_t  *prepare_buf;
    int      prepare_len;
} prepare_type_env_t;

static prepare_type_env_t m_prepare_write_env;

void write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

/* Attributes State Machine */
enum
{
    IDX_SVC,
    IDX_TEMP_1_CHAR,
    IDX_TEMP_1_VAL,
    IDX_TEMP_1_CHAR_CFG,
    IDX_NUM_STATES,
};

static uint16_t m_handle_table[IDX_NUM_STATES];


static const uint16_t primary_service_uuid         = ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid   = ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
static const uint8_t  char_prop_read               = ESP_GATT_CHAR_PROP_BIT_READ;
static const uint8_t  char_prop_write              = ESP_GATT_CHAR_PROP_BIT_WRITE;
static const uint8_t  char_prop_read_notify        = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t  char_prop_read_write_notify  = ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t  temp_1_measurement_ccc[2]    = {0x00, 0x00};
static const uint8_t  temp_1_char_value[4] = { 0x01, 0x02, 0x03, 0x04 };

/* Full Database Description - Used to add attributes into the database */
static const esp_gatts_attr_db_t gatt_db[IDX_NUM_STATES] =
{
    // Service Declaration
    [IDX_SVC]        =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&primary_service_uuid, ESP_GATT_PERM_READ,
      sizeof(uint16_t), sizeof(GATTS_SERVICE_UUID_COFFEE), (uint8_t *)&GATTS_SERVICE_UUID_COFFEE}},

    /* Characteristic Declaration */
    [IDX_TEMP_1_CHAR]     =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_declaration_uuid, ESP_GATT_PERM_READ,
      CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t *)&char_prop_read_notify}},

    /* Characteristic Value */
    [IDX_TEMP_1_VAL] =
    {{ESP_GATT_RSP_BY_APP}, {ESP_UUID_LEN_16, (uint8_t *)&GATTS_CHAR_UUID_TEMP_1, ESP_GATT_PERM_READ,
      sizeof(temp_1_char_value), sizeof(temp_1_char_value), (uint8_t *)temp_1_char_value}},

    /* Client Characteristic Configuration Descriptor */
    [IDX_TEMP_1_CHAR_CFG]  =
    {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t *)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
      sizeof(uint16_t), sizeof(temp_1_measurement_ccc), (uint8_t *)temp_1_measurement_ccc}},
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

    log.Debug("ON READ: need rsp = %d", param->read.need_rsp);

    if (param->read.handle == m_handle_table[IDX_TEMP_1_VAL]) {
        rsp.attr_value.len = 4;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        // TODO: update this
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0xc0;
        rsp.attr_value.value[1] = 0xff;
        rsp.attr_value.value[2] = 0xee;
        rsp.attr_value.value[3] = 0x01;
    } else {
        gatt_status = ESP_GATT_INVALID_HANDLE;
    }

    esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    gatt_status, &rsp);
}

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        m_adv_config_done &= (~ADV_CONFIG_FLAG);
        if (m_adv_config_done == 0) {
            esp_ble_gap_start_advertising(&m_adv_params);
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
        log.Info("REGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        esp_err_t status = esp_ble_gap_set_device_name(BLE_DEVICE_NAME);
        if (status != ESP_OK) {
            log.Error("set device name failed, error code = %x", status);
        }
        //config adv data
        status = esp_ble_gap_config_adv_data(&m_adv_data);
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
        break;
    case ESP_GATTS_EXEC_WRITE_EVT:
        log.Warn("ESP_GATTS_EXEC_WRITE_EVT");
        break;
    case ESP_GATTS_MTU_EVT:
        log.Info("ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        log.Info("SERVICE_START_EVT, status %d, service_handle %d\n",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        log.Info("SERVICE_STOP_EVT");
        break;
    case ESP_GATTS_CONNECT_EVT: {
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
        // TODO: check params below for iOS specs
        conn_params.latency = 0;
        conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;     // timeout = 400*10ms = 4000ms
        log.Info("ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        //start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
        log.Info("ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
        esp_ble_gap_start_advertising(&m_adv_params);
        break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT:
        if (param->add_attr_tab.status != ESP_GATT_OK) {
            log.Error("create attribute table failed, error code=0x%x", param->add_attr_tab.status);
        } else if (param->add_attr_tab.num_handle != IDX_NUM_STATES) {
            log.Error("create attribute table abnormally, num_handle (%d) doesn't equal to IDX_NUM_STATES(%d)",
                     param->add_attr_tab.num_handle, IDX_NUM_STATES);
        } else {
            log.Info("create attribute table successfully, the number handle = %d\n", param->add_attr_tab.num_handle);
            memcpy(m_handle_table, param->add_attr_tab.handles, sizeof(m_handle_table));
            esp_ble_gatts_start_service(m_handle_table[IDX_SVC]);

            printf("IDX SVC       %d\n", m_handle_table[IDX_SVC]);
            printf("IDX TEMP CHAR %d\n", m_handle_table[IDX_TEMP_1_CHAR]);
            printf("IDX TEMP VAL  %d\n", m_handle_table[IDX_TEMP_1_VAL]);
            printf("IDX TEMP CFG  %d\n", m_handle_table[IDX_TEMP_1_CHAR_CFG]);
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
    // If event is register event, store the gatts_if for each profile
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            m_profile_tab[PROFILE_APP_IDX].gatts_if = gatts_if;
        } else {
            log.Info("Reg app failed, app_id %04x, status %d\n", param->reg.app_id, param->reg.status);
            return;
        }
    }


    for (int idx = 0; idx < PROFILE_NUM; idx++) {
        // ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function

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

    log.Info("Init BLE");
    status = esp_bluedroid_init();
    if (status != ESP_OK) {
        log.Error("Bluedroid init failed: %s", esp_err_to_name(status));
        return;
    }

    log.Info("Bluedroid init");
    status = esp_bluedroid_enable();
    if (status != ESP_OK) {
        log.Error("Bluedroid enable failed: %s", esp_err_to_name(status));
        return;
    }

    log.Info("Register callbacks");
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
    
    log.Info("Register app");
    status = esp_ble_gatts_app_register(APP_ID);
    if (status != ESP_OK) {
        log.Error("Reg app failed: %s", esp_err_to_name(status));
        return;
    }

    // Set MTU
    log.Info("Set MTU");
    status = esp_ble_gatt_set_local_mtu(200);
    if (status != ESP_OK) {
        log.Error("Failed MTU set: %s", esp_err_to_name(status));
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