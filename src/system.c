#include "ad5761.h"
#include "adc.h"
#include "ble.h"
#include "button.h"
#include "switch.h"
#include "system.h"

void system_ble_machine()
{
    // uint32_t ble_cmd =
    //   ((uint32_t)BLE_PACKET.ntf_read_data_9000[1] << 24)
    // | ((uint32_t)BLE_PACKET.ntf_read_data_9000[0] << 16)
    // | ((uint32_t)BLE_PACKET.ntf_read_data_9000[3] << 8)
    // | ((uint32_t)BLE_PACKET.ntf_read_data_9000[2]);

    uint8_t ble_cmd = (uint8_t)BLE_PACKET.ntf_read_data_test[0];
    switch (ble_cmd)
    {
        case 0x01:
            switch_voltage_config();
            break;

        case 0x02:
            switch_inject_current_config();
            break;

        case 0x00:
            ad5761_software_full_reset(&ad5761_dev_i);
            ad5761_software_full_reset(&ad5761_dev_ii);
            switch_reset();
            break;

        default:
            break;
    }
}
