// #include "ble.h"

// // #define EN_LOGGING
// // #define SAMPLE_BLE_CODE
// /*
//     Find sample code at: https://github.com/zephyrproject-rtos/zephyr/blob/main/samples/bluetooth/st_ble_sensor/src/main.c

// */
// // #define MY_BLE_CODE

// #ifdef MY_BLE_CODE
// #ifdef EN_LOGGING
// #include <zephyr/logging/log.h>
// LOG_MODULE_REGISTER(debug, LOG_LEVEL_INF);
// #endif

// static volatile bool ble_ready = false;

// adv_packet_str ADV_PACKET ;

// static struct bt_data ad[] = {
//     BT_DATA_BYTES(BT_DATA_FLAGS,(BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
//     BT_DATA(BT_DATA_MANUFACTURER_DATA, ADV_PACKET.adv_data, sizeof(ADV_PACKET.adv_data))
// };


// static const struct bt_data sd[] = {
//     BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
// };

// #define ADV_INTERVAL_MIN  3200//   
// #define ADV_INTERVAL_MAX  6400 //

// static const struct bt_le_adv_param adv_param = BT_LE_ADV_PARAM_INIT(
//     BT_LE_ADV_OPT_CONNECTABLE | BT_LE_ADV_OPT_USE_IDENTITY | BT_LE_ADV_OPT_NO_2M,
//     ADV_INTERVAL_MIN,    
//     ADV_INTERVAL_MAX,
//     NULL);

// const static struct bt_gatt_attr *tx_attr;


// static ssize_t on_write(struct bt_conn *conn, const struct bt_gatt_attr *attr, const void *buf,
//     uint16_t len, uint16_t offset, uint8_t flags)   // received data from device
// {
//     const uint8_t *data = buf;
//     for (int i = 0; i < len; i++) {
//         LOG_INF("0x%02X ", data[i]);
//     }
//     LOG_INF("\n");
//     if (data[0] == 0x2A && data[1] == 0x80){
//         uint8_t data_size = data[2];
//         uint8_t received_crc = 0;
//     }
//     return len;
// }

// static ssize_t on_read(struct bt_conn *conn, const struct bt_gatt_attr *attr, 
//     void *buf, uint16_t len, uint16_t offset)       // write data to device
// {
//     // Trả về dữ liệu (read_value) khi có yêu cầu đọc
//     iData_str data;
//     uint8_t read_value[16];
//     read_value[0] = 0x2A;
//     read_value[1] = 0x80;
//     read_value[2] = 0x0C;// size
//     read_value[3] = data.counter & 0xFF;
//     read_value[4] = (data.counter >> 8) & 0xFF;
//     read_value[5] = (data.counter >> 16) & 0xFF;
//     read_value[6] = (data.counter >> 24) & 0xFF;
//     read_value[7] = data.year;
//     read_value[8] = data.month;
//     read_value[9] = data.day;
//     read_value[10] = data.hour;
//     read_value[11] = data.minute;
//     read_value[12] = data.second;
//     read_value[13] = data.temp & 0xFF;
//     read_value[14] = (data.temp >> 8) & 0xFF;
//     uint8_t crc = 0;
//     for (int i = 2; i <= 14; i++){
//         crc ^= read_value[i];
//     }
//     read_value[15] = crc;

//     size_t read_len = sizeof(read_value);

//     // Nếu offset lớn hơn kích thước dữ liệu, không có gì để trả về
//     if (offset > read_len) {
//         return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
//     }

//     // Trả về phần dữ liệu từ offset
//     return bt_gatt_attr_read(conn, attr, buf, len, offset, read_value, read_len);
// }


// void send_notify_data(struct bt_conn *conn)
// {
//     int err;
//     // err = bt_gatt_notify(conn, tx_attr, msg, sizeof(msg));
//     uint8_t msg[2];
//     msg[0] = ADV_PACKET.data_temp&0xFF;
//     msg[1] = (ADV_PACKET.data_temp>>8)&0xFF;
//     err = bt_gatt_notify(conn, tx_attr, msg, sizeof(msg));
//     if (err) {
//         LOG_ERR("Notify msg failed (err %d)", err);
//         return;
//     } else {
//         LOG_INF("Notify ok");
//     }
// }


// struct bt_conn *default_conn;
// static void tx_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
// {
//     if (value == BT_GATT_CCC_NOTIFY) {
//         LOG_INF("Notify enabled");
//         send_notify_data(default_conn);
//     }
// }

// BT_GATT_SERVICE_DEFINE(my_service,
//     BT_GATT_PRIMARY_SERVICE(BT_UUID_DECLARE_16(0x1234)),  // có thể thay đổi tùy ý
//     BT_GATT_CHARACTERISTIC(BT_UUID_DECLARE_16(0x9876),   // UUID của characteristic
//                            BT_GATT_CHRC_WRITE | BT_GATT_CHRC_READ | BT_GATT_CHRC_NOTIFY,            // Cho phép ghi vào
//                            BT_GATT_PERM_WRITE | BT_GATT_PERM_READ | BT_GATT_PERM_NONE,           // Quyền ghi
//                            on_read, on_write, NULL),        // Callback khi nhận dữ liệu
//     BT_GATT_CCC(tx_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
// ); 

// #ifdef EN_TX_POWER
// void set_tx_power(uint8_t handle_type, uint16_t handle, int8_t tx_pwr_lvl)
// {
//     struct bt_hci_cp_vs_write_tx_power_level *cp;
//     struct bt_hci_rp_vs_write_tx_power_level *rp;
//     struct net_buf *buf, *rsp = NULL;
//     int err;

//     buf = bt_hci_cmd_create(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL,
//                 sizeof(*cp));
//     if (!buf) {
//         // LOG_ERR("Unable to allocate command buffer");
//         return;
//     }

//     cp = net_buf_add(buf, sizeof(*cp));
//     cp->handle = sys_cpu_to_le16(handle);
//     cp->handle_type = handle_type;
//     cp->tx_power_level = tx_pwr_lvl;

//     err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_WRITE_TX_POWER_LEVEL,
//                    buf, &rsp);
//     if (err) {
//         // LOG_ERR("Set Tx power err: %d", err);
//         return;
//     }

//     rp = (void *)rsp->data;
//     // LOG_INF("Actual Tx Power: %d", rp->selected_tx_power);

//     net_buf_unref(rsp);
// }

// void get_tx_power(uint8_t handle_type, uint16_t handle, int8_t *tx_pwr_lvl)
// {
//     struct bt_hci_cp_vs_read_tx_power_level *cp;
//     struct bt_hci_rp_vs_read_tx_power_level *rp;
//     struct net_buf *buf, *rsp = NULL;
//     int err;

//     *tx_pwr_lvl = 0xFF;
//     buf = bt_hci_cmd_create(BT_HCI_OP_VS_READ_TX_POWER_LEVEL,
//                 sizeof(*cp));
//     if (!buf) {
//         // LOG_ERR("Unable to allocate command buffer");
//         return;
//     }

//     cp = net_buf_add(buf, sizeof(*cp));
//     cp->handle = sys_cpu_to_le16(handle);
//     cp->handle_type = handle_type;

//     err = bt_hci_cmd_send_sync(BT_HCI_OP_VS_READ_TX_POWER_LEVEL,
//                    buf, &rsp);
//     if (err) {
//         // LOG_ERR("Read Tx power err: %d", err);
//         return;
//     }

//     rp = (void *)rsp->data;
//     *tx_pwr_lvl = rp->tx_power_level;

//     net_buf_unref(rsp);
// }
// #endif

// static void connected(struct bt_conn *conn, uint8_t err)
// {
//     if (err) {
//         return;
//     }
//     gpio_pin_configure_dt(&led_blue_spec, GPIO_OUTPUT);
//     gpio_pin_set_dt(&led_blue_spec, 1);
//     default_conn = bt_conn_ref(conn);
//     // LOG_INF("Device connected");
// }

// static void disconnected(struct bt_conn *conn, uint8_t reason)
// {
//     // LOG_INF("Device disconnected (reason %u)", reason);

//     if (default_conn) {
//         bt_conn_unref(default_conn);
//         default_conn = NULL;
//     }
//     gpio_pin_configure_dt(&led_blue_spec, GPIO_DISCONNECTED);
// }

// // Đăng ký callback BLE CONNECTION
// static struct bt_conn_cb conn_callbacks = {
//     .connected = connected,
//     .disconnected = disconnected,
// };

// static void bt_ready(int err) {
//     if (err) {
//         // LOG_ERR("Bluetooth init failed (err %d)", err);
//         return;
//     }
//     // LOG_INF("Bluetooth initialized");
//     ble_ready = true;
// }

// void ble_update_data(){
//     ADV_PACKET.adv_data[0] = ADV_PACKET.data_len;
//     ADV_PACKET.adv_data[1] = ADV_PACKET.data_type;
//     ADV_PACKET.adv_data[2] = ADV_PACKET.data_company_id & 0xFF;
//     ADV_PACKET.adv_data[3] = (ADV_PACKET.data_company_id>>8) & 0xFF;
//     ADV_PACKET.adv_data[4] = ADV_PACKET.data_sync;
//     ADV_PACKET.adv_data[5] = ADV_PACKET.data_func;
//     ADV_PACKET.adv_data[6] = ADV_PACKET.data_counter & 0xFF;
//     ADV_PACKET.adv_data[7] = (ADV_PACKET.data_counter>>8) & 0xFF;
//     ADV_PACKET.adv_data[8] = (ADV_PACKET.data_counter>>16) & 0xFF;
//     ADV_PACKET.adv_data[9] = (ADV_PACKET.data_counter>>24) & 0xFF;
//     ADV_PACKET.adv_data[10] = ADV_PACKET.data_status & 0xFF;
//     ADV_PACKET.adv_data[11] = (ADV_PACKET.data_status>>8) & 0xFF;
//     ADV_PACKET.adv_data[12] = ADV_PACKET.year;
//     ADV_PACKET.adv_data[13] = ADV_PACKET.month;
//     ADV_PACKET.adv_data[14] = ADV_PACKET.day;
//     ADV_PACKET.adv_data[15] = ADV_PACKET.hour;
//     ADV_PACKET.adv_data[16] = ADV_PACKET.minute;
//     ADV_PACKET.adv_data[17] = ADV_PACKET.second;
//     ADV_PACKET.adv_data[18] = ADV_PACKET.data_temp & 0xFF;
//     ADV_PACKET.adv_data[19] = (ADV_PACKET.data_temp>>8) & 0xFF;
//     ADV_PACKET.adv_data[20] = ADV_PACKET.data_bat;
// }

// void ble_init(void) {
//     ADV_PACKET.data_len = 0x15;
//     ADV_PACKET.data_type = 0xFF;
//     ADV_PACKET.data_company_id = 0xE1F5;
//     ADV_PACKET.data_sync = 0x2A;
//     ADV_PACKET.data_func = 0x80;
//     ADV_PACKET.data_status = 0x99;

//     int err = bt_enable(bt_ready);
//     if (err) {
//         // LOG_ERR("Bluetooth init failed (err %d)", err);
//         return;
//     }
    
//     while (!ble_ready) {
//         // LOG_INF("Waiting for Bluetooth stack...");
//         k_busy_wait(100);
//     }
    
//     err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
//     if (err) {
//         // LOG_ERR("Advertising failed to start (err %d)", err);
//     } else {
//         // LOG_INF("BLE Advertising started");
//     }

// }
// #endif

// #ifdef SAMPLE_BLE_CODE

// /* Button value. */
// static uint16_t but_val;

// /* Prototype */
// static ssize_t recv(struct bt_conn *conn,
// 		    const struct bt_gatt_attr *attr, const void *buf,
// 		    uint16_t len, uint16_t offset, uint8_t flags);

// /* Custom Service  */
// static const struct bt_uuid_128 st_service_uuid = BT_UUID_INIT_128(
// 	BT_UUID_128_ENCODE(0x0000fe40, 0xcc7a, 0x482a, 0x984a, 0x7f2ed5b3e58f));

// /* LED service */
// static const struct bt_uuid_128 led_char_uuid = BT_UUID_INIT_128(
// 	BT_UUID_128_ENCODE(0x0000fe41, 0x8e22, 0x4541, 0x9d4c, 0x21edae82ed19));

// /* Notify button service */
// static const struct bt_uuid_128 but_notif_uuid = BT_UUID_INIT_128(
// 	BT_UUID_128_ENCODE(0x0000fe42, 0x8e22, 0x4541, 0x9d4c, 0x21edae82ed19));

// #define DEVICE_NAME CONFIG_BT_DEVICE_NAME
// #define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)
// #define ADV_LEN 12

// /* Advertising data */
// static uint8_t manuf_data[ADV_LEN] = {
// 	0x01 /*SKD version */,
// 	0x83 /* STM32WB - P2P Server 1 */,
// 	0x00 /* GROUP A Feature  */,
// 	0x00 /* GROUP A Feature */,
// 	0x00 /* GROUP B Feature */,
// 	0x00 /* GROUP B Feature */,
// 	0x00, /* BLE MAC start -MSB */
// 	0x00,
// 	0x00,
// 	0x00,
// 	0x00,
// 	0x00, /* BLE MAC stop */
// };

// static const struct bt_data ad[] = {
// 	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
// 	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
// 	BT_DATA(BT_DATA_MANUFACTURER_DATA, manuf_data, ADV_LEN)
// };

// /* BLE connection */
// struct bt_conn *ble_conn;
// /* Notification state */
// volatile bool notify_enable;

// static void mpu_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
// {
// 	ARG_UNUSED(attr);
// 	notify_enable = (value == BT_GATT_CCC_NOTIFY);
// 	LOG_INF("Notification %s", notify_enable ? "enabled" : "disabled");
// }

// /* ST BLE Sensor GATT services and characteristic */

// BT_GATT_SERVICE_DEFINE(stsensor_svc,
// BT_GATT_PRIMARY_SERVICE(&st_service_uuid),
// BT_GATT_CHARACTERISTIC(&led_char_uuid.uuid,
// 		       BT_GATT_CHRC_READ | BT_GATT_CHRC_WRITE_WITHOUT_RESP,
// 		       BT_GATT_PERM_WRITE, NULL, recv, (void *)1),
// BT_GATT_CHARACTERISTIC(&but_notif_uuid.uuid, BT_GATT_CHRC_NOTIFY,
// 		       BT_GATT_PERM_READ, NULL, NULL, &but_val),
// BT_GATT_CCC(mpu_ccc_cfg_changed, BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
// );

// static void button_callback(const struct device *gpiob, struct gpio_callback *cb,
// 		     uint32_t pins)
// {
// 	int err;

// 	LOG_INF("Button pressed");
// 	if (ble_conn) {
// 		if (notify_enable) {
// 			err = bt_gatt_notify(NULL, &stsensor_svc.attrs[4],
// 					     &but_val, sizeof(but_val));
// 			if (err) {
// 				LOG_ERR("Notify error: %d", err);
// 			} else {
// 				LOG_INF("Send notify ok");
// 				but_val = (but_val == 0) ? 0x100 : 0;
// 			}
// 		} else {
// 			LOG_INF("Notify not enabled");
// 		}
// 	} else {
// 		LOG_INF("BLE not connected");
// 	}
// }

// static void bt_ready(int err)
// {
// 	if (err) {
// 		LOG_ERR("Bluetooth init failed (err %d)", err);
// 		return;
// 	}
// 	LOG_INF("Bluetooth initialized");
// 	/* Start advertising */
// 	err = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad), NULL, 0);
// 	if (err) {
// 		LOG_ERR("Advertising failed to start (err %d)", err);
// 		return;
// 	}

// 	LOG_INF("Configuration mode: waiting connections...");
// }

// static void connected(struct bt_conn *connected, uint8_t err)
// {
// 	if (err) {
// 		LOG_ERR("Connection failed (err %u)", err);
// 	} else {
// 		LOG_INF("Connected");
// 		if (!ble_conn) {
// 			ble_conn = bt_conn_ref(connected);
// 		}
// 	}
// }

// static void disconnected(struct bt_conn *disconn, uint8_t reason)
// {
// 	if (ble_conn) {
// 		bt_conn_unref(ble_conn);
// 		ble_conn = NULL;
// 	}

// 	LOG_INF("Disconnected, reason %u %s", reason, bt_hci_err_to_str(reason));
// }

// BT_CONN_CB_DEFINE(conn_callbacks) = {
// 	.connected = connected,
// 	.disconnected = disconnected,
// };

// #endif