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

#define MY_BLE_CODE
// #define SAMPLE_BLE_CODE

#ifdef MY_BLE_CODE

extern void ble_init(void);

#define PACKET_SIZE 22

typedef struct {
    uint8_t data_len;
    uint8_t data_type;
    uint16_t data_company_id;  // 0xE1F5
    uint8_t data_sync;           // 0x2A
    uint8_t data_func;           // thermal : 0x80
    uint32_t data_counter;
    uint16_t data_status;
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint16_t data_temp;
    uint8_t data_bat;
    uint8_t data_crc;
    uint8_t adv_data[PACKET_SIZE];
}adv_packet_str;

extern adv_packet_str ADV_PACKET;
extern struct bt_conn *default_conn;
extern void send_notify_data(struct bt_conn *conn);

#endif

#ifdef SAMPLE_BLE_CODE

static void bt_ready(int err);

#endif

#endif // BLE_H 