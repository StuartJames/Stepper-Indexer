/*
*              Copyright (c) 2019-2021 HydraSystems.
*
*  This software is copyrighted by and is the sole property of
*  HydraSystems.  All rights, title, ownership, or other interests
*  in the software remain the property of HydraSystems.
*  This software may only be used in accordance with the corresponding
*  license agreement.  Any unauthorised use, duplication, transmission,
*  distribution, or disclosure of this software is expressly forbidden.
*
*  This Copyright notice may not be removed or modified without prior
*  written consent of HydraSystems.
*
*  HydraSystems, reserves the right to modify this software without
*  notice.
*
* =====================================================================
*
* This file contains code to implement the Bluetooth connection.
*
* Edit     Date     Version       Edit Description
* ====  ==========  ======= =====================================================
* SJ    2019/11/26   1.a.1   Original
* SJ    2021/09/26   1.a.4   Release
*
*/

#include <esp_gap_ble_api.h>
#include <esp_gatts_api.h>
#include <esp_bt_defs.h>
#include <esp_bt_main.h>
#include <esp_gatt_common_api.h>
#include "StepperIndexer.h"

#define BT_TAG "[GATT ]"

#define SPP_PROFILE_NUM             1
#define SPP_PROFILE_APP_IDX         0
#define ESP_SPP_APP_ID              0x56
#define DEVICE_NAME                 "FireBreakSystem"
#define SPP_SVC_INST_ID             0

/// SPP Service
static const uint16_t               spp_service_uuid = 0xABF0;

/// Characteristic UUID
#define ESP_GATT_UUID_SPP_DATA_RECEIVE      0xABF1
#define ESP_GATT_UUID_SPP_DATA_NOTIFY       0xABF2
#define ESP_GATT_UUID_SPP_COMMAND_RECEIVE   0xABF3
#define ESP_GATT_UUID_SPP_COMMAND_NOTIFY    0xABF4
#define ESP_GATT_UUID_SPP_REPORT            0xABF5

#define spp_sprintf(s,...)         sprintf((char*)(s), ##__VA_ARGS__)
#define SPP_DATA_MAX_LEN           (512)
#define SPP_CMD_MAX_LEN            (20)
#define SPP_STATUS_MAX_LEN         (20)
#define SPP_DATA_BUFF_MAX_LEN      (2 * 1024)


//Attributes State Machine
enum{
    SPP_IDX_SVC,
    SPP_IDX_DATA_RECV_CHAR,
    SPP_IDX_DATA_RECV_VAL,
    SPP_IDX_DATA_NOTIFY_CHAR,
    SPP_IDX_DATA_NTY_VAL,
    SPP_IDX_DATA_NTF_CFG,
    SPP_IDX_COMMAND_CHAR,
    SPP_IDX_COMMAND_VAL,
    SPP_IDX_STATUS_CHAR,
    SPP_IDX_STATUS_VAL,
    SPP_IDX_STATUS_CFG,
    SPP_IDX_NB,
};


static const uint8_t    AdvertisingData[23] = {0x02, 0x01, 0x06, 0x03, 0x03, 0xF0, 0xAB, 0x0F, 0x09, 0x45, 0x53, 0x50, 0x5f, 0x53, 0x50, 0x50, 0x5f, 0x53, 0x45, 0x52, 0x56, 0x45, 0x52};

static uint16_t         spp_mtu_size = 23;
static uint16_t         spp_conn_id = 0xffff;
static esp_gatt_if_t    spp_gatts_if = 0xff;
//static uint8_t          heartbeat_s[9] = {'E','s','p','r','e','s','s','i','f'};
//static bool             EnableReportNtf = false;

static bool             EnableDataNtf = false;
static bool             IsConnected = false;
static esp_bd_addr_t    spp_remote_bda = {0x0, };

static uint16_t         spp_handle_table[SPP_IDX_NB];

static esp_ble_adv_params_t ssp_advertising_params = {
  .adv_int_min =        0x20,
  .adv_int_max =        0x40,
  .adv_type =           ADV_TYPE_IND,
  .own_addr_type =      BLE_ADDR_TYPE_PUBLIC,
  .channel_map =        ADV_CHNL_ALL,
  .adv_filter_policy =  ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};


struct gatts_profile_inst{
  esp_gatts_cb_t        gatts_cb;
  uint16_t              gatts_if;
  uint16_t              app_id;
  uint16_t              conn_id;
  uint16_t              service_handle;
  esp_gatt_srvc_id_t    service_id;
  uint16_t              char_handle;
  esp_bt_uuid_t         char_uuid;
  esp_gatt_perm_t       perm;
  esp_gatt_char_prop_t  property;
  uint16_t              descr_handle;
  esp_bt_uuid_t         descr_uuid;
};


typedef struct spp_receive_data_node{
  int32_t               len;
  uint8_t               *node_buff;
  struct spp_receive_data_node *next_node;
} spp_receive_data_node_t;


typedef struct spp_receive_data_buff{
  int32_t               node_num;
  int32_t               buff_size;
  spp_receive_data_node_t *first_node;
} spp_receive_data_buff_t;


static void GattsProfileEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst spp_profile_tab[SPP_PROFILE_NUM] = {
  [SPP_PROFILE_APP_IDX] = { .gatts_cb = GattsProfileEventHandler, .gatts_if = ESP_GATT_IF_NONE, },       // defaults to ESP_GATT_IF_NONE
};


/*
 *  SPP PROFILE ATTRIBUTES
 ****************************************************************************************
 */

#define CHAR_DECLARATION_SIZE   (sizeof(uint8_t))

static const uint16_t primary_service_uuid =          ESP_GATT_UUID_PRI_SERVICE;
static const uint16_t character_declaration_uuid =    ESP_GATT_UUID_CHAR_DECLARE;
static const uint16_t character_client_config_uuid =  ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

static const uint8_t char_prop_read_notify =          ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
static const uint8_t char_prop_read_write =           ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_READ;

///SPP Service - data receive characteristic, read&write without response
static const uint16_t spp_data_receive_uuid =         ESP_GATT_UUID_SPP_DATA_RECEIVE;
static const uint8_t spp_data_receive_val[20] =       {0x00};

///SPP Service - data notify characteristic, notify&read
static const uint16_t spp_data_notify_uuid =          ESP_GATT_UUID_SPP_DATA_NOTIFY;
static const uint8_t spp_data_notify_val[20] =        {0x00};
static const uint8_t spp_data_notify_ccc[2] =         {0x00, 0x00};

///SPP Service - command characteristic, read&write without response
static const uint16_t spp_command_uuid =              ESP_GATT_UUID_SPP_COMMAND_RECEIVE;
static const uint8_t spp_command_val[10] =            {0x00};

///SPP Service - status characteristic, notify&read
static const uint16_t spp_status_uuid =               ESP_GATT_UUID_SPP_COMMAND_NOTIFY;
static const uint8_t spp_status_val[10] =             {0x00};
static const uint8_t spp_status_ccc[2] =              {0x00, 0x00};

///Full HRS Database Description - Used to add attributes into the database
static const esp_gatts_attr_db_t SPPGattDB[SPP_IDX_NB] = {
    [SPP_IDX_SVC] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&primary_service_uuid, ESP_GATT_PERM_READ, sizeof(spp_service_uuid), sizeof(spp_service_uuid), (uint8_t*)&spp_service_uuid}},                                         //SPP -  Service Declaration
    [SPP_IDX_DATA_RECV_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_read_write}},                      //SPP -  data receive characteristic Declaration
    [SPP_IDX_DATA_RECV_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&spp_data_receive_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, SPP_DATA_MAX_LEN, sizeof(spp_data_receive_val), (uint8_t*)spp_data_receive_val}},     //SPP -  data receive characteristic Value
    [SPP_IDX_DATA_NOTIFY_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_read_notify}},                   //SPP -  data notify characteristic Declaration
    [SPP_IDX_DATA_NTY_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&spp_data_notify_uuid, ESP_GATT_PERM_READ, SPP_DATA_MAX_LEN, sizeof(spp_data_notify_val), (uint8_t*)spp_data_notify_val}},                               //SPP -  data notify characteristic Value
    [SPP_IDX_DATA_NTF_CFG] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(spp_data_notify_ccc), (uint8_t*)spp_data_notify_ccc}}, //SPP -  data notify characteristic - Client Characteristic Configuration Descriptor
    [SPP_IDX_COMMAND_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_read_write}},                        //SPP -  command characteristic Declaration
    [SPP_IDX_COMMAND_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&spp_command_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, SPP_CMD_MAX_LEN, sizeof(spp_command_val), (uint8_t*)spp_command_val}},                       //SPP -  command characteristic Value
    [SPP_IDX_STATUS_CHAR] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_declaration_uuid, ESP_GATT_PERM_READ, CHAR_DECLARATION_SIZE, CHAR_DECLARATION_SIZE, (uint8_t*)&char_prop_read_notify}},                        //SPP -  status characteristic Declaration
    [SPP_IDX_STATUS_VAL] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&spp_status_uuid, ESP_GATT_PERM_READ, SPP_STATUS_MAX_LEN, sizeof(spp_status_val), (uint8_t*)spp_status_val}},                                              //SPP -  status characteristic Value
    [SPP_IDX_STATUS_CFG] = {{ESP_GATT_AUTO_RSP}, {ESP_UUID_LEN_16, (uint8_t*)&character_client_config_uuid, ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, sizeof(uint16_t), sizeof(spp_status_ccc), (uint8_t*)spp_status_ccc}},             //SPP -  status characteristic - Client Characteristic Configuration Descriptor
};

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static uint8_t GetDescriptionIndex(uint16_t handle)
{
uint8_t error = 0xff;

  for(int i = 0; i < SPP_IDX_NB; i++){
    if(handle == spp_handle_table[i]){
      return i;
    }
  }
  return error;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void SendTask(void *arg)
{
SendObject_t SendObj;

  xEventGroupWaitBits(m_SystemFlags, SYSFLAG_SYSTEM_UP, pdFALSE, pdFALSE, portMAX_DELAY);
  while(true){
    if(xQueueReceive(m_BTSendQueue, &SendObj, portMAX_DELAY)){
      if(IsConnected){
//        ESP_LOGI(BT_TAG, "%s, %d", (char*)SendObj.pData, SendObj.Length);
        if(esp_ble_gatts_send_indicate(spp_gatts_if, spp_conn_id, spp_handle_table[SPP_IDX_DATA_NTY_VAL], SendObj.Length, (uint8_t*)SendObj.pData, false) == ESP_FAIL){
          ESP_LOGI(BT_TAG, "Falied to send data");
        }
      }
      FreePtr(SendObj.pData);
    }
  }
  vTaskDelete(NULL);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void GapEventHandler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
esp_err_t err;

  ESP_LOGI(BT_TAG, "GAP event:%d", event);
  switch(event){
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
      esp_ble_gap_start_advertising(&ssp_advertising_params);
      break;
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
      if((err = param->adv_start_cmpl.status) != ESP_BT_STATUS_SUCCESS){              //advertising start complete event to indicate advertising start successfully or failed
        ESP_LOGE(BT_TAG, "Advertising start failed: %s", esp_err_to_name(err));
      }
      break;
    default:
      break;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void GattsEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
  ESP_LOGI(BT_TAG, "GATT event:%d, if:%d", event, gatts_if);
  if(event == ESP_GATTS_REG_EVT){                                                     // If this event is 'register event' store the gatts_if for each profile
    if(param->reg.status == ESP_GATT_OK){
      spp_profile_tab[SPP_PROFILE_APP_IDX].gatts_if = gatts_if;
    }
    else{
      ESP_LOGI(BT_TAG, "Reg app failed, app_id %04x, status %d", param->reg.app_id, param->reg.status);
      return;
    }
  }
  for(int idx = 0; idx < SPP_PROFILE_NUM; idx++){
    if(gatts_if == ESP_GATT_IF_NONE || gatts_if == spp_profile_tab[idx].gatts_if){    // ESP_GATT_IF_NONE, gatt_if not specified, we then need to call every profile cb function
      if(spp_profile_tab[idx].gatts_cb){
        spp_profile_tab[idx].gatts_cb(event, gatts_if, param);
      }
    }
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void GattsProfileEventHandler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
esp_ble_gatts_cb_param_t *p_data = (esp_ble_gatts_cb_param_t*)param;
uint8_t res = 0xff;
CmdObject_t RecObj = {.Srce = CD_GATT, .pData = NULL, .Length = 0};

  ESP_LOGI(BT_TAG, "Profile event:%d", event);
  switch(event){
    case ESP_GATTS_REG_EVT:
      ESP_LOGI(BT_TAG, "%s %d", __func__, __LINE__);
      esp_ble_gap_set_device_name(DEVICE_NAME);
      ESP_LOGI(BT_TAG, "%s %d", __func__, __LINE__);
      esp_ble_gap_config_adv_data_raw((uint8_t*)AdvertisingData, sizeof(AdvertisingData));
      ESP_LOGI(BT_TAG, "%s %d", __func__, __LINE__);
      esp_ble_gatts_create_attr_tab(SPPGattDB, gatts_if, SPP_IDX_NB, SPP_SVC_INST_ID);
      break;
    case ESP_GATTS_READ_EVT:
      res = GetDescriptionIndex(p_data->read.handle);
      if(res == SPP_IDX_STATUS_VAL){     //TODO: client read the status characteristic
        ESP_LOGI(BT_TAG, "ESP_GATTS_READ_EVT : handle = %d, value:", res);
      }
      break;
    case ESP_GATTS_WRITE_EVT: {
      res = GetDescriptionIndex(p_data->write.handle);
      if(p_data->write.is_prep == false){
        ESP_LOGI(BT_TAG, "ESP_GATTS_WRITE_EVT : handle = %d, value len %d, value:", res, p_data->write.len);
        switch(res){
          case SPP_IDX_COMMAND_VAL:{
            ESP_LOGI(BT_TAG, "command value");
            esp_log_buffer_char(BT_TAG, (char *)(p_data->write.value), p_data->write.len);
            break;
          }
          case SPP_IDX_DATA_NTF_CFG:{
            ESP_LOGI(BT_TAG, "data notify");
            if((p_data->write.len == 2) && (p_data->write.value[1] == 0x00)){
              EnableDataNtf = (p_data->write.value[0] == 0x01);
            }
            break;
          }
          case SPP_IDX_DATA_RECV_VAL:{
            ESP_LOGI(BT_TAG, "data receive");
            uint8_t *pCmdBuff = NULL;
            pCmdBuff = (uint8_t*)malloc((p_data->write.len + 1) * sizeof(uint8_t));
            if(pCmdBuff == NULL){
              ESP_LOGE(BT_TAG, "%s malloc failed", __func__);
              break;
            }
            memset(pCmdBuff, 0x0, (p_data->write.len + 1));
            memcpy(pCmdBuff, (char *)p_data->write.value, p_data->write.len);
            RecObj.pData = pCmdBuff;
            RecObj.Length = p_data->write.len;
            xQueueSend(m_CmdQueue, &RecObj, 10 / portTICK_PERIOD_MS);
            break;
          }
          default:{       //TODO:
            ESP_LOGI(BT_TAG, "unkown descr value");
            break;
          }
        }
      }
      else{
        if((p_data->write.is_prep == true) && (res == SPP_IDX_DATA_RECV_VAL)){
          ESP_LOGI(BT_TAG, "ESP_GATTS_PREP_WRITE_EVT : handle = %d", res);
        }
      }
      break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT: {
      ESP_LOGI(BT_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
      if(p_data->exec_write.exec_write_flag){
      }
      break;
    }
    case ESP_GATTS_MTU_EVT:
      spp_mtu_size = p_data->mtu.mtu;
      break;
    case ESP_GATTS_CONF_EVT:
      break;
    case ESP_GATTS_UNREG_EVT:
      break;
    case ESP_GATTS_DELETE_EVT:
      break;
    case ESP_GATTS_START_EVT:
      break;
    case ESP_GATTS_STOP_EVT:
      break;
    case ESP_GATTS_CONNECT_EVT:
      spp_conn_id = p_data->connect.conn_id;
      spp_gatts_if = gatts_if;
      IsConnected = true;
      memcpy(&spp_remote_bda, &p_data->connect.remote_bda, sizeof(esp_bd_addr_t));
      SendMessage(CD_GATT, "Stepper connected");
      break;
    case ESP_GATTS_DISCONNECT_EVT:
      IsConnected = false;
      EnableDataNtf = false;
      esp_ble_gap_start_advertising(&ssp_advertising_params);
      break;
    case ESP_GATTS_OPEN_EVT:
      break;
    case ESP_GATTS_CANCEL_OPEN_EVT:
      break;
    case ESP_GATTS_CLOSE_EVT:
      break;
    case ESP_GATTS_LISTEN_EVT:
      break;
    case ESP_GATTS_CONGEST_EVT:
      break;
    case ESP_GATTS_CREAT_ATTR_TAB_EVT: {
      ESP_LOGI(BT_TAG, "The number handle =%x", param->add_attr_tab.num_handle);
      if(param->add_attr_tab.status != ESP_GATT_OK){
        ESP_LOGE(BT_TAG, "Create attribute table failed, error code=0x%x", param->add_attr_tab.status);
      }
      else if(param->add_attr_tab.num_handle != SPP_IDX_NB){
        ESP_LOGE(BT_TAG, "Create attribute table abnormally, num_handle (%d) doesn't equal to HRS_IDX_NB(%d)", param->add_attr_tab.num_handle, SPP_IDX_NB);
      }
      else{
        memcpy(spp_handle_table, param->add_attr_tab.handles, sizeof(spp_handle_table));
        esp_ble_gatts_start_service(spp_handle_table[SPP_IDX_SVC]);
      }
      break;
    }
    default:
      break;
  }
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void BluetoothInit(void)
{
esp_err_t ret;

  ret = nvs_flash_init();                                                     // Initialise NVS
  if(ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND){
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
  esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
  ret = esp_bt_controller_init(&bt_cfg);
  if(ret){
    ESP_LOGE(BT_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
    return;
  }
  ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
  if(ret){
    ESP_LOGE(BT_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
    return;
  }
  ESP_LOGI(BT_TAG, "%s init bluetooth", __func__);
  ret = esp_bluedroid_init();
  if(ret){
    ESP_LOGE(BT_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
    return;
  }
  ret = esp_bluedroid_enable();
  if(ret){
    ESP_LOGE(BT_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
    return;
  }
  ret = esp_ble_gatts_register_callback(GattsEventHandler);
  if(ret){
    ESP_LOGE(BT_TAG, "gatts_register failed, return code = %x", ret);
    return;
  }
  ret = esp_ble_gap_register_callback(GapEventHandler);
  if(ret){
    ESP_LOGE(BT_TAG, "gap register failed, return code = %x", ret);
    return;
  }
  ret = esp_ble_gatts_app_register(SPP_PROFILE_APP_IDX);
  if(ret){
    ESP_LOGE(BT_TAG, "gatts_app_register failed, return code = %x", ret);
    return;
  }
  esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
  if(local_mtu_ret){
    ESP_LOGE(BT_TAG, "set local  MTU failed, return code = %x", local_mtu_ret);
  }
  xTaskCreate(SendTask, "Gatt Send", 2048, NULL, 10, NULL);
  return;
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


