#include "feetech_sts.h"
#include "config.h"
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include "chprintf.h"

struct feetech_sts_t feetech_sts = {0};

bool reached = false;
size_t num_bytes = 0;
int8_t statuss = 0;

// Function to split a 16-bit value into two 8-bit values
static void SplitByte(uint8_t *DataL, uint8_t *DataH, uint16_t Data) {
    *DataH = (Data >> 8);
    *DataL = (Data & 0xff);
}

// Function to send an instruction to the servo
static void send_instruction(uint8_t servo_id, uint8_t instruction, uint8_t *params, uint8_t param_count) {
    uint8_t buffer_tx[64];
    int buffer_tx_idx = 0;
    buffer_tx[buffer_tx_idx++] = 0xFF;
    buffer_tx[buffer_tx_idx++] = 0xFF;
    buffer_tx[buffer_tx_idx++] = servo_id;
    buffer_tx[buffer_tx_idx++] = param_count + 2;
    buffer_tx[buffer_tx_idx++] = instruction;

    for (int i = 0; i < param_count; i++) {
        buffer_tx[buffer_tx_idx++] = params[i];
    }

    uint8_t checksum = 0;
    for (int i = 2; i < buffer_tx_idx; i++) {
        checksum += buffer_tx[i];
    }
    buffer_tx[buffer_tx_idx++] = ~checksum;

    size_t len = buffer_tx_idx;
    uartSendFullTimeout(feetech_sts.port, &len, buffer_tx, TIME_MS2I(1));
}

// Function to parse servo response and update status
static void parse_servo_response(uint8_t *response, uint8_t length) {
    // Feetech STS response format:
    // [0xFF] [0xFF] [ID] [Length] [Error] [Param1] [Param2] ... [Checksum]
    
    if (length < 6) return; // Minimum response length
    
    // Verify header
    if (response[0] != 0xFF || response[1] != 0xFF) return;
    
    // Verify servo ID
    if (response[2] != feetech_sts.servo_id) return;
    
    uint8_t data_length = response[3];
    uint8_t error = response[4];
    
    // // Calculate and verify checksum
    // uint8_t checksum = 0;
    // for (int i = 2; i < length - 1; i++) {
    //     checksum += response[i];
    // }
    // checksum = ~checksum;
    
    // if (checksum != response[length - 1]) return; // Checksum mismatch
    
    // If no error and we have position data (8 bytes: pos, speed, load, voltage)
    feetech_sts.present_temperature = 100;
    if (error == 0 && data_length >= 9) { // 1 error byte + 8 data bytes
        uint8_t *data = &response[5]; // Start of data
        
        // Parse position (2 bytes, little endian)
        feetech_sts.present_position = (int16_t)(data[0] | (data[1] << 8));
        
        // Parse speed (2 bytes, little endian) 
        feetech_sts.present_speed = (int16_t)(data[2] | (data[3] << 8));
        
        // Parse load (2 bytes, little endian)
        feetech_sts.present_load = (int16_t)(data[4] | (data[5] << 8));
        
        // Parse voltage and temperature (if available)
        if (data_length >= 11) { // 1 error + 8 data + voltage + temp
            feetech_sts.present_voltage = data[6];
            // feetech_sts.present_temperature = data[7];
        }
    }
}

// Thread to handle communication with the servo
static THD_WORKING_AREA(feetech_sts_wa, 512);
static THD_FUNCTION(feetech_sts_thd, arg) {
    (void)arg;
    chRegSetThreadName("feetech_sts");

    while (true) {
        // Set servo to wheel mode
        uint8_t params[2] = {0};
        params[0] = STS_WORK_MODE;
        params[1] = 1; // Motor mode
        send_instruction(feetech_sts.servo_id, INST_WRITE, params, sizeof(params));
        chThdSleepMicroseconds(350);

        // --- Write target speed based on UAVCAN commands ---
        if (feetech_sts.torque_enabled) {
            uint8_t data_write[7]= {0};
            volatile int sign = 0;
            if (feetech_sts.target_speed < 0)
            {
            sign = 1;
            feetech_sts.target_speed *= -1;
            }
            data_write[0] = STS_GOAL_POSITION_L;
            SplitByte(&data_write[1], &data_write[2], 0);
            SplitByte(&data_write[3], &data_write[4], 0);
            SplitByte(&data_write[5], &data_write[6], feetech_sts.target_speed);
            if (sign)
            {
                data_write[6] |= (1 << 7);  // Set bit 7 (replace bitSet)
            }

            // Send the command to the servo
            send_instruction(feetech_sts.servo_id, INST_WRITE, data_write, sizeof(data_write));
        }
         else {
            // Disable torque
            uint8_t data_disable[2] = {STS_TORQUE_ENABLE, 0};
            send_instruction(feetech_sts.servo_id, INST_WRITE, data_disable, sizeof(data_disable));
        }
        chThdSleepMicroseconds(350);

        // --- Read servo feedback ---
        uint8_t data_read[2]= {0};
        data_read[0] = STS_PRESENT_POSITION_L;
        data_read[1] = 8;
        send_instruction(feetech_sts.servo_id, INST_READ, data_read, sizeof(data_read));
        
        // Wait for response and read it
        chThdSleepMicroseconds(350);        

        // Try to receive response from servo
        size_t recv_size = 50;
        uint8_t buf[50] = {0};
        // if(uartReceiveTimeout(drs_parachute.port, &recv_size, (void *)buf, TIME_MS2I(20)) != MSG_RESET) {

        // rx_bytes = sizeof(rx_buffer);
        msg_t response = uartReceiveTimeout(feetech_sts.port, &recv_size, (void *)buf, TIME_MS2I(1));

        if (response == MSG_OK){
            // Successfully received data
            statuss = 1; // Reset status
        } else if (response == MSG_TIMEOUT) {
            statuss = 2; // Timeout, no data received 
        }
        else if (response == MSG_RESET) {
            // No data received, reset status
            statuss = 3;
        } else {
            // Error receiving data
            statuss = 4;
        }
        // uartStartReceive(feetech_sts.port, recv_size, (void *)buf);
        // chThdSleepMicroseconds(5000);;
        // uartStopReceive(feetech_sts.port);
        if (recv_size > 0) {
            // Parse the response and update servo status
            reached = true;
            parse_servo_response((void *)buf, recv_size);
        }
        
        num_bytes = recv_size;
        chThdSleepMicroseconds(500);;
    }
}

// Broadcast Feetech servo status over UAVCAN
void broadcast_feetech_status(struct uavcan_iface_t *iface) {
    // Only broadcast if the servo is configured and active
    if (feetech_sts.port == NULL) {
        return;
    }

    static uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_STATUS_MAX_SIZE];
    struct uavcan_equipment_actuator_Status status = {0};
    
    // Fill in the status message
    status.actuator_id = feetech_sts.index;
    
    // Convert servo position to radians (assuming 4096 counts per revolution)
    status.position = (float)feetech_sts.present_position * 2.0f * M_PI / 4096.0f;
    
    // Convert speed to rad/s (STS speed units to rad/s)
    // Note: STS speed is in steps/sec, adjust conversion factor as needed
    status.speed = (float)feetech_sts.present_speed * 2.0f * M_PI / 4096.0f;
    
    // Normalize load to 0-1 range
    status.force = (float)feetech_sts.present_load / 1000.0f;
    
    // Set power and other fields
    status.power_rating_pct = feetech_sts.torque_enabled ? 100 : 0;
    
    // Add temperature and voltage as debug info
    char debug_msg[100];
    chsnprintf(debug_msg, 99, "Pos:%d Spd:%d Load:%d V:%dmV T:%dC, Reached:%d, Bytes:%d, Status:%d", 
              feetech_sts.present_position,
              feetech_sts.present_speed, 
              feetech_sts.present_load,
              feetech_sts.present_voltage * 100, // Convert to mV
              feetech_sts.present_temperature, reached, num_bytes, statuss);
    
    // Send debug message periodically
    static uint8_t debug_counter = 0;
    if (++debug_counter >= 1) { // Every 1 status messages (~5 seconds)
        uavcanDebugIface(iface, UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO, "FEETECH", debug_msg);
        debug_counter = 0;
    }
    
    // Encode and broadcast the message
    uint16_t total_size = uavcan_equipment_actuator_Status_encode(&status, buffer);
    uavcanBroadcast(iface,
                    UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE,
                    UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID,
                    &feetech_sts.status_transfer_id,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    total_size);
    
    // Increment transfer ID for next message
    feetech_sts.status_transfer_id++;
}

void feetech_sts_init(void) {
    // Get configuration
    feetech_sts.index = config_get_by_name("FEETECH index", 0)->val.i;
    feetech_sts.servo_id = config_get_by_name("FEETECH id", 0)->val.i;
    feetech_sts.uart_cfg.speed = config_get_by_name("FEETECH baudrate", 0)->val.i;
    feetech_sts.uart_cfg.cr1 = USART_CR1_UE | USART_CR1_RE | USART_CR1_TE; // Enable UART and TX only for half-duplex
    // feetech_sts.uart_cfg.cr3 = USART_CR3_HDSEL; // Enable half-duplex mode

    // Initialize command fields
    feetech_sts.target_speed = 500;
    feetech_sts.torque_enabled = false;
    feetech_sts.present_position = 0;
    feetech_sts.present_speed = 0;
    feetech_sts.present_load = 0;
    feetech_sts.present_voltage = 0;
    feetech_sts.present_temperature = 0;
    feetech_sts.status_transfer_id = 0;

    // Configure the port and pins
    uint8_t port = config_get_by_name("FEETECH port", 0)->val.i;
    if(port == 1) {
        // In half-duplex mode, only TX pin is used for bidirectional communication
        palSetLineMode(SERIAL1_RX_LINE, PAL_MODE_INPUT);
        palSetLineMode(SERIAL1_TX_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
        feetech_sts.port = &UARTD1;
    } else if(port == 2) {
        palSetLineMode(SERIAL2_RX_LINE, PAL_MODE_INPUT);
        palSetLineMode(SERIAL2_TX_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
        feetech_sts.port = &UARTD2;
    } else if(port == 3) {
        palSetLineMode(SERIAL3_TX_LINE, PAL_MODE_STM32_ALTERNATE_OPENDRAIN);
        feetech_sts.port = &UARTD3;
    } else {
        feetech_sts.port = NULL;
    }

    // Start UART and servo thread only if port is configured
    if(feetech_sts.port != NULL) {
        uartStart(feetech_sts.port, &feetech_sts.uart_cfg);
        chThdCreateStatic(feetech_sts_wa, sizeof(feetech_sts_wa), NORMALPRIO-2, feetech_sts_thd, NULL);
    }
}