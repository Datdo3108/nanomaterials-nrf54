#ifndef BLE_H
#define BLE_H

#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/services/bas.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/hci_vs.h>

#include <zephyr/drivers/gpio.h>

#define MY_BLE_CODE
// #define SAMPLE_BLE_CODE

#ifdef MY_BLE_CODE

extern void ble_init(void);
extern bool update_dac_ble;

#define PACKET_SIZE 2

typedef struct {
    // uint8_t data_len;
    // uint8_t data_type;
    // uint16_t data_company_id;  // 0xE1F5
    // uint8_t data_sync;           // 0x2A
    // uint8_t data_func;           // thermal : 0x80
    // uint32_t data_counter;
    // uint16_t data_status;
    // uint8_t year;
    // uint8_t month;
    // uint8_t day;
    // uint8_t hour;
    // uint8_t minute;
    // uint8_t second;
    // uint16_t data_temp;
    // uint8_t data_bat;
    // uint8_t data_crc;
    uint8_t conn_tx_data[2];
    uint8_t adv_data[PACKET_SIZE];
    uint8_t ntf_data[10];       /* 2-byte each channel data, 5 channels (2 DACs, 3 ADCs)*/
    uint8_t ntf_read_data[4];
}ble_packet_str;

extern ble_packet_str BLE_PACKET;
extern struct bt_conn *default_conn;
// extern void send_notify_data(struct bt_conn *conn);
extern void send_notify_data(void);

#endif

#ifdef SAMPLE_BLE_CODE

static void bt_ready(int err);

#endif

#endif // BLE_H 