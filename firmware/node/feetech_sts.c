#include <feetech_sts.h>
#include <config.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <chprintf.h>
#include <stm32f105xc.h>


struct feetech_sts_t feetech_sts = {0};
struct debug_message_t debug_msg = {0};

// Add per-message transfer IDs
static uint8_t tid_status = 0;
static uint8_t tid_debug  = 0;
static uint8_t tid_config = 0;

// Global initialization state
bool feetech_initialized = false;
bool first_position_update = true;

// ============================================================================
// INITIALIZATION AND MAIN THREAD (TOP LEVEL)
// ============================================================================

void feetech_sts_init(void) {
    // Load all configuration settings from config
    feetech_sts.index = config_get_by_name("FEETECH index", 0)->val.i;
    feetech_sts.servo_id = config_get_by_name("FEETECH id", 0)->val.i;
    feetech_sts.uart_cfg.speed = config_get_by_name("FEETECH baudrate", 0)->val.i;
    feetech_sts.uart_cfg.cr1 = USART_CR1_UE | USART_CR1_TE;
    feetech_sts.uart_cfg.cr3 = USART_CR3_HDSEL;
    
    // Load servo configuration from config
    feetech_sts.config.gear_ratio = config_get_by_name("FEETECH gear ratio", 0)->val.i;
    feetech_sts.config.max_speed = config_get_by_name("FEETECH max speed", 0)->val.i;
    feetech_sts.config.can_frequency = config_get_by_name("FEETECH CAN frequency", 0)->val.f;
    feetech_sts.config.serial_frequency = config_get_by_name("FEETECH serial frequency", 0)->val.f;
    feetech_sts.config.load_threshold = config_get_by_name("FEETECH load threshold", 0)->val.i;
    feetech_sts.config.integral_load_threshold = config_get_by_name("FEETECH integral load threshold", 0)->val.i;
    feetech_sts.config.load_timeout = config_get_by_name("FEETECH load timeout", 0)->val.i;
    feetech_sts.config.log_level = feetech_get_log_level(config_get_by_name("FEETECH log level", 0)->val.i);
    feetech_sts.config.min_angle = 0;
    feetech_sts.config.max_angle = 90;
    feetech_sts.config.log_level = long_log_message;
    feetech_sts.config.max_speed = 4000; 
    
    // Initialize control state
    feetech_sts.armed = true;
    feetech_sts.target_wing_angle = 0;
    feetech_sts.target_servo_position = 0;
    feetech_sts.calculation_offset = 0;
    feetech_sts.current_position_global = 0;
    feetech_sts.revolution_count = 0;
    feetech_sts.previous_servo_position = 0;

    // Initialize status fields
    feetech_sts.latest_servo_position = 0;
    feetech_sts.latest_servo_speed = 0;
    feetech_sts.latest_wing_angle = 0;
    feetech_sts.latest_load = 0;
    feetech_sts.latest_voltage = 0;
    feetech_sts.latest_temperature = 0;
    feetech_sts.latest_error_code = 0;
    feetech_sts.latest_health_state = 0;

    debug_msg.send = false;
    debug_msg.message[0] = '\0';

    // Configure the port and pins
    uint8_t port = config_get_by_name("FEETECH port", 0)->val.i;
    if(port == 1) {
        palSetLineMode(SERIAL1_TX_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
        palSetLineMode(SERVO1_LINE, PAL_MODE_INPUT_PULLUP);
        feetech_sts.port = &UARTD1;
    } else if(port == 2) {
        palSetLineMode(SERIAL2_TX_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
        feetech_sts.port = &UARTD2;
    } else if(port == 3) {
        palSetLineMode(SERIAL3_TX_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
        feetech_sts.port = &UARTD3;
    } else {
        feetech_sts.port = NULL;
    }

    // Start UART and threads only if port is configured
    if(feetech_sts.port != NULL) {
        uartStart(feetech_sts.port, &feetech_sts.uart_cfg);
        
        feetech_init_sequence();

        // Start serial thread (lower priority)
        chThdCreateStatic(feetech_serial_wa, sizeof(feetech_serial_wa), NORMALPRIO-2, feetech_serial_thd, NULL);
    }
}

void feetech_init_sequence(void) {

    feetech_setup_angle();

    reach_endstop();

    servo_update_status();

    // Set initialized flag
    feetech_initialized = true;
}

// ============================================================================
// THREAD FUNCTIONS
// ============================================================================
static THD_FUNCTION(feetech_serial_thd, arg) {
    (void)arg;
    chRegSetThreadName("feetech_serial");
    // Main serial loop (only accessible after init is complete)

    while (true) {
        // #TODO: Main serial loop implementation will go here
        // For now, just update servo status periodically

        feetech_move_to_target();

        servo_update_status();
        // feetech_debug_status();

        // Sleep based on configured serial frequency
        uint32_t period_ms = (uint32_t)(1000.0f / feetech_sts.config.serial_frequency);
        chThdSleepMilliseconds(period_ms);
    }
}

// ============================================================================
// UTILITY FUNCTIONS
// ============================================================================

// Function to split a 16-bit value into two 8-bit values
void SplitByte(uint8_t *DataL, uint8_t *DataH, int16_t Data){
    *DataH = (uint8_t)(Data >> 8);
    *DataL = (uint8_t)(Data & 0xFF);
}

void JoinBytes(uint8_t DataL, uint8_t DataH, int16_t *Data) {
    *Data = (int16_t)((DataH << 8) | DataL);
}

void SignedIntToFeetechSigned(uint8_t *DataL,uint8_t *DataH, int16_t Data)
{
    if (Data < 0){
        SplitByte(DataL, DataH, abs(Data));
        *DataH |= 0x80; // Set sign bit if negative
    }
    else{
        SplitByte(DataL, DataH, abs(Data));
    }
}

void FeetechSignedtoSignedInt(uint8_t DataL,uint8_t DataH, int16_t *Data)
{
    // Check if sign bit is set
    if (DataH & 0x80) {
        // Negative value 
        JoinBytes(DataL, DataH & 0x7F, Data);
        *Data += INT16_MIN;
    } else {
        // Positive value
        JoinBytes(DataL, DataH, Data);
    }
}

int16_t ServoPositionToWingPosition(int16_t servo_position) {
    // Convert servo position to wing position
    // Assuming 0-4096 range for angles, adjust based on gear ratio
    servo_position -= feetech_sts.calculation_offset;
    int16_t wing_target = (int16_t)((float)(servo_position / (feetech_sts.config.gear_ratio * DEGREE_TO_FEETECH)));
    return wing_target;
}

int16_t WingPositionToServoPosition(int16_t wing_position) {
    // Convert wing position back to servo position
    // Assuming 0-4096 range for angles, adjust based on gear ratio
    int16_t servo_target = (int16_t)((float)(wing_position * feetech_sts.config.gear_ratio * DEGREE_TO_FEETECH));
    servo_target += feetech_sts.calculation_offset;

    return servo_target;
}


// ============================================================================
// LOW-LEVEL COMMUNICATION FUNCTIONS
// ============================================================================

// Function to send an instruction to the servo
void send_instruction(uint8_t servo_id, uint8_t instruction, uint8_t *params, uint8_t param_count) {
    if (feetech_sts.port == NULL) return;
    
    uint8_t buffer_tx[64];
    int buffer_tx_idx = 0;
    
    // Build packet
    buffer_tx[buffer_tx_idx++] = 0xFF;
    buffer_tx[buffer_tx_idx++] = 0xFF;
    buffer_tx[buffer_tx_idx++] = servo_id;
    buffer_tx[buffer_tx_idx++] = param_count + 2;
    buffer_tx[buffer_tx_idx++] = instruction;

    for (int i = 0; i < param_count; i++) {
        buffer_tx[buffer_tx_idx++] = params[i];
    }

    // Calculate checksum
    uint8_t checksum = 0;
    for (int i = 2; i < buffer_tx_idx; i++) {
        checksum += buffer_tx[i];
    }
    buffer_tx[buffer_tx_idx++] = ~checksum;

    // Send packet
    size_t len = buffer_tx_idx;
    uartSendFullTimeout(feetech_sts.port, &len, buffer_tx, TIME_MS2I(1));
}

// Function to read servo response
bool receive_servo_response(uint8_t *response, uint8_t *length, uint32_t timeout_ms) {
    if (feetech_sts.port == NULL) return false;
    
    size_t recv_size = 50;
    msg_t result = uartReceiveTimeout(feetech_sts.port, &recv_size, response, TIME_MS2I(timeout_ms));
    
    if (result == MSG_RESET || recv_size == 0) {
        return false;
    }
    
    *length = (uint8_t)recv_size;
    return true;
}

/**
 * @brief Parses servo response packet and updates servo status information
 * 
 * This function processes incoming response packets from Feetech STS servos,
 * performing packet validation including structure verification and checksum
 * calculation. Upon successful validation, it extracts and updates the servo's
 * operational parameters including current position, movement speed, load factor,
 * supply voltage, and internal temperature.
 * 
 * Feetech STS response format:
 * [0xFF] [0xFF] [ID] [Length] [Error] [Param1] [Param2] ... [Checksum]
 * 
 * @param[in] packet Pointer to the received servo response packet buffer
 * @param[in] packet_len Length of the received packet in bytes
 * 
 * @return servo_response_t enum indicating success or specific error type
 * @retval SERVO_RESPONSE_SUCCESS Successfully parsed and updated servo status
 * @retval SERVO_RESPONSE_INVALID_HEADER Invalid packet header
 * @retval SERVO_RESPONSE_CHECKSUM_MISMATCH Packet checksum verification failed
 * @retval SERVO_RESPONSE_INSUFFICIENT_LENGTH Packet too short to contain valid data
 * @retval SERVO_RESPONSE_INVALID_SERVO_ID Response from wrong servo ID
 * @retval SERVO_RESPONSE_ERROR Servo reported an error condition
 */
servo_response_t parse_servo_status(uint8_t *response, uint8_t length) {
    // Check for insufficient length
    if (length < 6) {
        return SERVO_RESPONSE_INSUFFICIENT_LENGTH;
    }
    
    // Verify packet header structure
    if (response[0] != 0xFF || response[1] != 0xFF) {
        return SERVO_RESPONSE_INVALID_HEADER;
    }
    
    // Verify servo ID matches expected
    if (response[2] != feetech_sts.servo_id) {
        return SERVO_RESPONSE_INVALID_SERVO_ID;
    }
    
    // // Check if servo reported an error
    // if (feetech_sts.latest_error_code != 0) {
    //     return SERVO_RESPONSE_ERROR;
    // }

    // Verify checksum
    uint8_t calculated_checksum = 0;
    for (int i = 2; i < length - 1; i++) {
        calculated_checksum += response[i];
    }
    calculated_checksum = ~calculated_checksum;

    if (calculated_checksum != response[length - 1]) {
        return SERVO_RESPONSE_CHECKSUM_MISMATCH;
    }

        
    uint8_t data_length = response[3];
    feetech_sts.latest_error_code = response[4];

    // Parse data if available (8 bytes: pos, speed, load, voltage, temp)
    if (data_length >= 9) // 1 error byte + 8 data bytes
    {
        feetech_sts.previous_servo_position = feetech_sts.latest_servo_position;
        feetech_sts.latest_servo_position = (int16_t)(response[5] | (response[6] << 8));
        
        // Check for revolution crossing
        revolution_count_update();
        
        global_position_update();
        
        feetech_sts.latest_wing_angle = ServoPositionToWingPosition(feetech_sts.current_position_global);

        FeetechSignedtoSignedInt(response[7], response[8], &feetech_sts.latest_servo_speed);
        FeetechSignedtoSignedInt(response[9], response[10], &feetech_sts.latest_load);

        feetech_sts.latest_voltage = response[11]+1;
        feetech_sts.latest_temperature = response[12];
    }
    
    return SERVO_RESPONSE_SUCCESS;
}

// ============================================================================
// HIGH-LEVEL SERVO CONTROL FUNCTIONS
// ============================================================================
void reach_endstop(void) {
    feetech_set_position(STS_MAX_POSITION_GLOBAL, 200);
    while(palReadLine(SERVO1_LINE)) {
        // Update status
        servo_update_status();
    }
    feetech_sts.target_servo_position = feetech_sts.current_position_global;
    feetech_set_position(feetech_sts.target_servo_position, 200);
    feetech_sts.calculation_offset = feetech_sts.current_position_global;
    feetech_sts.target_wing_angle = ServoPositionToWingPosition(feetech_sts.target_servo_position);
}

// Request servo response for a specific address and length
void request_servo_response(uint8_t read_address, uint8_t read_length) {
    uint8_t data_read[2] = {read_address, read_length}; // Read specified length from position
    send_instruction(feetech_sts.servo_id, INST_READ, data_read, sizeof(data_read));
}



/**
 * @brief Updates the servo status by requesting and parsing the latest status data
 * 
 * This function sends a request to the servo to retrieve its current status,
 * including position, speed, load, voltage, and temperature. It waits for a
 * response, parses, and then updates the internal state of the Feetech STS servo accordingly.
 * 
 * @return servo_response_t enum indicating success or specific error type
 * @retval SERVO_RESPONSE_SUCCESS Successfully updated servo status
 * @retval SERVO_RESPONSE_* Various error conditions as defined in servo_response_t
 */
servo_response_t servo_update_status(void) {
    if (feetech_sts.port == NULL) {
        return false;
    }

    uint8_t response[50];
    uint8_t length = 0;
    uint32_t timeout_ms = 50;

    // Request status data
    request_servo_response(STS_CURRENT_POSITION, 8); 
    receive_servo_response(response, &length, timeout_ms);
    // Parse the response
    servo_response_t result = parse_servo_status(response, length);

    return result;
}

// ============================================================================
// CONTROL STATE FUNCTIONS
// ============================================================================


// ============================================================================
// UAVCAN INTERFACE FUNCTIONS
// ============================================================================

// Handle incoming servo instruction messages
void handle_can_servo_instruction(struct uavcan_iface_t *iface, CanardRxTransfer* transfer) {
    struct com_feetech_servo_Instruction instruction;
    
    if (com_feetech_servo_Instruction_decode(transfer, &instruction)) {
        return; // Decode failed
    }
    
    // Check if this instruction is for our servo
    if (instruction.actuator_id != feetech_sts.index) {
        return;
    }
    
    switch (instruction.message_type) {
        case COM_FEETECH_SERVO_INSTRUCTION_MSG_TYPE_ARM_DISARM:
            feetech_sts.armed = (instruction.data[0] != 0);
            break;
            
        case COM_FEETECH_SERVO_INSTRUCTION_MSG_TYPE_SET_TARGET_ANGLE:
            {
                FeetechSignedtoSignedInt(instruction.data[0], instruction.data[1], &feetech_sts.target_wing_angle);
                feetech_sts.target_servo_position = WingPositionToServoPosition(feetech_sts.target_wing_angle);
                int16_t abs_target_angle;
                JoinBytes(instruction.data[0], instruction.data[1], &abs_target_angle);
                char targetmsg[100]; 
                chsnprintf(targetmsg, sizeof(targetmsg), 
                        "LL: %02x, HH: %02x, Target angle: %04x, Target angleDec: %d Abs:%04x", instruction.data[0], instruction.data[1], feetech_sts.target_wing_angle, feetech_sts.target_wing_angle, abs(abs_target_angle));
                
                feetech_debug_message(targetmsg);
                // F6 FF
                // 0A 00
            }
            break;
            
        case COM_FEETECH_SERVO_INSTRUCTION_MSG_TYPE_SET_SERIAL_FREQ:
            feetech_sts.config.serial_frequency = instruction.data[0] | (instruction.data[1] << 8);
            break;
        
        case COM_FEETECH_SERVO_INSTRUCTION_MSG_TYPE_SET_CAN_FREQ:
            feetech_sts.config.can_frequency = instruction.data[0] | (instruction.data[1] << 8);
            break;

        case COM_FEETECH_SERVO_INSTRUCTION_MSG_TYPE_SET_LOG_LEVEL:
            if (instruction.data[0] == 0) {
                feetech_sts.config.log_level = short_log_message;
            } else {
                feetech_sts.config.log_level = long_log_message;
            }
            break;

        case COM_FEETECH_SERVO_INSTRUCTION_MSG_TYPE_SET_TARGET_POSITION:
            {
                FeetechSignedtoSignedInt(instruction.data[0], instruction.data[1], &feetech_sts.target_servo_position );
                feetech_sts.target_wing_angle = ServoPositionToWingPosition(feetech_sts.target_servo_position );
            }
            break;

        default:
            break;
    }
    
    // Debug message
    char debug_msg[90];
    chsnprintf(debug_msg, 89, "Servo CMD: ID=%d, Type=%d", 
              instruction.actuator_id, instruction.message_type);
    uavcanDebugIface(iface, UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_DEBUG, "SERVO", debug_msg);
}

// Broadcast servo status
void broadcast_feetech_status(struct uavcan_iface_t *iface) {
    if (feetech_sts.port == NULL) return;

    static uint8_t buffer[COM_FEETECH_SERVO_STATUS_MAX_SIZE > COM_FEETECH_SERVO_DEBUG_MAX_SIZE ?
                          COM_FEETECH_SERVO_STATUS_MAX_SIZE : COM_FEETECH_SERVO_DEBUG_MAX_SIZE];
    uint16_t total_size = 0;

    switch (feetech_sts.config.log_level) {
    case long_log_message: {
        struct com_feetech_servo_Debug msg = {0};

        feetech_sts.latest_health_state ++;

        // Fill in the long message fields
        msg.actuator_id = feetech_sts.index;
        msg.armed = feetech_sts.armed;
        msg.target_servo_position = feetech_sts.target_servo_position;
        msg.target_wing_angle = feetech_sts.target_wing_angle;
        msg.calculation_offset = feetech_sts.calculation_offset;
        msg.current_position_global = feetech_sts.current_position_global;
        msg.revolution_count = feetech_sts.revolution_count;
        msg.previous_servo_position = feetech_sts.previous_servo_position;
        msg.current_angle = feetech_sts.latest_wing_angle;
        msg.current_position = feetech_sts.latest_servo_position;
        msg.current_speed = feetech_sts.latest_servo_speed;
        msg.latest_load = feetech_sts.latest_load;
        msg.voltage = feetech_sts.latest_voltage;
        msg.temp = feetech_sts.latest_temperature;
        msg.errcode = feetech_sts.latest_error_code;
        msg.health_state = feetech_sts.latest_health_state;
        msg.integral_load = feetech_sts.integral_load;

        // Encoding the long message
        total_size = com_feetech_servo_Debug_encode(&msg, buffer);

        // Broadcast the long message
        uavcanBroadcast(iface,
                        COM_FEETECH_SERVO_DEBUG_SIGNATURE,
                        COM_FEETECH_SERVO_DEBUG_ID,
                        &tid_debug,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        buffer,
                        total_size);
        break;
    }
    default: {
        struct com_feetech_servo_Status msg = {0};

        // Fill in the short message fields
        msg.actuator_id = feetech_sts.index;
        msg.current_angle = feetech_sts.latest_wing_angle;

        // Encoding the short message
        total_size = com_feetech_servo_Status_encode(&msg, buffer);
    
        // Broadcast the short message
        uavcanBroadcast(iface,
                        COM_FEETECH_SERVO_STATUS_SIGNATURE,
                        COM_FEETECH_SERVO_STATUS_ID,
                        &tid_status,
                        CANARD_TRANSFER_PRIORITY_LOW,
                        buffer,
                        total_size);
        break;
    }
    }
}

// Broadcast servo config
void broadcast_servo_config(struct uavcan_iface_t *iface) {
    if (feetech_sts.port == NULL) return;
    static uint8_t buffer[COM_FEETECH_SERVO_CONFIG_MAX_SIZE];
    struct com_feetech_servo_Config cfg = {0};
    
    cfg.actuator_id = feetech_sts.index;
    cfg.gear_ratio = feetech_sts.config.gear_ratio;
    cfg.min_angle = feetech_sts.config.min_angle;
    cfg.max_angle = feetech_sts.config.max_angle;
    cfg.serial_frequency = feetech_sts.config.serial_frequency;
    cfg.can_frequency = feetech_sts.config.can_frequency;
    
    uint16_t total = com_feetech_servo_Config_encode(&cfg, buffer);
    uavcanBroadcast(iface,
                    COM_FEETECH_SERVO_CONFIG_SIGNATURE,
                    COM_FEETECH_SERVO_CONFIG_ID,
                    &tid_config,
                    CANARD_TRANSFER_PRIORITY_LOW,
                    buffer,
                    total);
}

void broadcast_feetech_debug(struct uavcan_iface_t *iface) {
    if (debug_msg.send == true){
        uavcanDebugIface(iface, UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO, "FEETECH", debug_msg.message);
        debug_msg.send = false;
    }
}

void feetech_debug_message(const char *message) {
    if (feetech_sts.port == NULL) return;

    // Prepare debug message
    chsnprintf(debug_msg.message, sizeof(debug_msg.message), "%s", message);
    debug_msg.send = true;
}

// ============================================================================
// CONFIGURATION FUNCTIONS
// ============================================================================

void feetech_setup_angle(void) {
    if (feetech_sts.port == NULL) return;
    
    uint8_t write_unlock[2] = {0x37, 0};
    send_instruction(feetech_sts.servo_id, INST_WRITE, write_unlock, 2);
    chThdSleepMicroseconds(SERVO_SLEEP_TIME_US);
    
    uint8_t angle_limit[3] = {STS_MAXIMUM_ANGLE, 0, 0};
    send_instruction(feetech_sts.servo_id, INST_WRITE, angle_limit, 3);
    chThdSleepMicroseconds(SERVO_SLEEP_TIME_US);
    
    uint8_t write_lock[2] = {0x37, 1};
    send_instruction(feetech_sts.servo_id, INST_WRITE, write_lock, 2);
    chThdSleepMicroseconds(SERVO_SLEEP_TIME_US);

}

loglevel_t feetech_get_log_level(int log_int) {
    switch (log_int){
        case 0:
            return short_log_message;
        case 1:
            return long_log_message;
        default:
            return short_log_message;
    }
}

void feetech_disarm(void) {
    if (feetech_sts.port == NULL) return;

    uint8_t params[2] = {STS_TORQUE_SWITCH, 0};
    send_instruction(feetech_sts.servo_id, INST_WRITE, params, sizeof(params));
    chThdSleepMicroseconds(SERVO_SLEEP_TIME_US);
}

void feetech_set_position(int16_t position, int16_t speed) {
    if (feetech_sts.port == NULL) return;

    if(feetech_sts.armed == true) {
        uint8_t data_write[9] = {0};

        data_write[0] = STS_TORQUE_SWITCH;
        data_write[1] = feetech_sts.armed;
        data_write[2] = 0;
        SignedIntToFeetechSigned(&data_write[3], &data_write[4], position);
        SplitByte(&data_write[5], &data_write[6], 0);
        SignedIntToFeetechSigned(&data_write[7], &data_write[8], speed);

        // Send the command to the servo
        send_instruction(feetech_sts.servo_id, INST_WRITE, data_write, sizeof(data_write));
        chThdSleepMicroseconds(SERVO_SLEEP_TIME_US);
    }
    else {
        feetech_disarm();
    }
}

void feetech_move_to_target(void) {
    feetech_set_position(feetech_sts.target_servo_position, 
                         feetech_sts.config.max_speed);
}

void feetech_debug_status(void){
    char status_msg[100];
    chsnprintf(status_msg, 99, "A:%d CurA:%d TgtA:%d Off:%d CurP:%d TgtP:%d Spd:%d Ld:%d T:%d V:%d Err:%d H:%d",
              feetech_sts.armed,
              feetech_sts.latest_wing_angle,
              feetech_sts.target_wing_angle,
              feetech_sts.calculation_offset,
              feetech_sts.latest_servo_position,
              feetech_sts.target_servo_position,
              feetech_sts.latest_servo_speed,
              (uint8_t)((feetech_sts.latest_load >= 0 ? feetech_sts.latest_load : -feetech_sts.latest_load) / 10),
              feetech_sts.latest_temperature,
              feetech_sts.latest_voltage,
              feetech_sts.latest_error_code,
              feetech_sts.latest_health_state);
    feetech_debug_message(status_msg);
}

int16_t local_to_global_position(int16_t revolution_count, int16_t local_position) {
    int16_t global_position = (revolution_count * STS_MAX_POSITION_LOCAL) + local_position;

    return global_position;
}

int16_t global_to_local_position(int16_t revolution_count, int16_t global_position) {
    // Convert global position to local position
    int16_t local_position = global_position - (revolution_count * STS_MAX_POSITION_LOCAL);
    
    return local_position;
}

void revolution_count_update(void) {
    if(first_position_update){
        feetech_sts.previous_servo_position = feetech_sts.latest_servo_position;
        feetech_sts.current_position_global = feetech_sts.latest_servo_position;
        first_position_update = false;
    }
    else{
        int16_t position_difference = feetech_sts.latest_servo_position - feetech_sts.previous_servo_position;
        if (abs(position_difference) > REVOLUTION_CROSSING_THRESHOLD) {
            // If the position has wrapped around (from 4095 to 0), increment revolution count
            if (position_difference < 0) {
                feetech_sts.revolution_count++;
            } else {
            // If the position has wrapped around (from 0 to 4095), decrement revolution count
                feetech_sts.revolution_count--;
            }
        }
    }
}

void global_position_update(void) {
    // Update the global position based on the current local position and revolution count
    feetech_sts.current_position_global = local_to_global_position(feetech_sts.revolution_count, feetech_sts.latest_servo_position);
    
    // // Check for overflow/underflow
    // if (feetech_sts.current_position_global < STS_MIN_POSITION_GLOBAL) {
    //     feetech_sts.current_position_global = STS_MIN_POSITION_GLOBAL;
    // } else if (feetech_sts.current_position_global > STS_MAX_POSITION_GLOBAL) {
    //     feetech_sts.current_position_global = STS_MAX_POSITION_GLOBAL;
    // }
}