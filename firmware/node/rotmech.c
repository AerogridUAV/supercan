#include <rotmech.h>
#include <config.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>
#include <chprintf.h>
#include <stm32f105xc.h>
#include "thread_utils.h"
#include <stdarg.h>

struct rotmech_t rotmech = {0};

// Thread function declaration
static THD_FUNCTION(small_rotmech_serial_thd, arg);
static THD_FUNCTION(big_rotmech_serial_thd, arg);
static THD_FUNCTION(rotmech_telem_thd, arg);

// Global initialization state
bool rotmech_initialized = false;
bool endstop_reached = false;
bool first_position_update = true;

// ============================================================================
// INITIALIZATION AND MAIN THREAD (TOP LEVEL)
// ============================================================================
void rotmech_init(void){
    // Rotation mechanism actuator index
    rotmech.index = config_get_by_name("ROTMECH index", 0)->val.i;

    // Determine rotation mechanism type, if >=3 no rotmech is used
    uint8_t rotmech_type = config_get_by_name("ROTMECH type", 0)->val.i;
    switch (rotmech_type) {
        case 0:
            rotmech.type = small_rotmech;
            small_rotmech_init();
            break;
        case 1:
            rotmech.type = big_rotmech;
            big_rotmech_init();
            break;
        case 2:
            rotmech.type = big_rotmech_sensor;
            big_rotmech_init();
            break;
        default:
            // No rotation mechanism configured
            break;
    }
}

/**
 * @brief Common initialization for all rotation mechanisms
 */
void common_rotmech_init(void) {
    // Configure UART
    rotmech.servo_id = config_get_by_name("ROTMECH servo id", 0)->val.i;
    rotmech.uart_cfg.speed = 1000000;
    rotmech.uart_cfg.cr1 = USART_CR1_UE | USART_CR1_TE;
    rotmech.uart_cfg.cr3 = USART_CR3_HDSEL;

    // Load general configuration from config
    rotmech.config.min_angle = config_get_by_name("ROTMECH min angle", 0)->val.i;
    rotmech.config.max_angle = config_get_by_name("ROTMECH max angle", 0)->val.i;
    rotmech.config.can_frequency = config_get_by_name("ROTMECH telem frequency", 0)->val.f;

    // Set servo configuration
    rotmech.config.serial_frequency = 100;
    rotmech.config.load_threshold = 130;

    // Initialize control state
    rotmech.armed = true;
    rotmech.target_wing_angle = 0;
    rotmech.target_servo_position = 0;
    rotmech.calculation_offset = 0;
    rotmech.current_position_global = 0;
    rotmech.revolution_count = 0;
    rotmech.previous_servo_position = 0;

    // Initialize status fields
    rotmech.latest_servo_position = 0;
    rotmech.latest_servo_speed = 0;
    rotmech.latest_wing_angle = 0;
    rotmech.latest_load = 0;
    rotmech.latest_voltage = 0;
    rotmech.latest_temperature = 0;
    rotmech.latest_error_code = 0;
    rotmech.latest_health_state = 0;
}

/**
 * @brief Initialize small rotation mechanism
 * Uses servo with internal encoder for direct position control
 */
void small_rotmech_init(void) {
    common_rotmech_init();
    
    // Load servo configuration from config
    rotmech.config.gear_ratio = 9;
    rotmech.config.physical_offset = -2310;
    rotmech.safe_load = true;
    rotmech.config.init_speed = 2*config_get_by_name("ROTMECH init speed", 0)->val.i;
    if (rotmech.config.init_speed == 0) {
        rotmech.config.init_speed = 200; // Default init speed
    }

    // Configure the port and pins

    // Encoder pins
    palSetLineMode(SERIAL2_TX_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    palSetLineMode(SERIAL2_RX_LINE, PAL_MODE_INPUT_PULLUP);
    rotmech.port = &UARTD2;

    // Switch Pins
    palSetLineMode(SERIAL1_RX_LINE, PAL_MODE_INPUT_PULLUP);
    rotmech.switch0_pin = SERIAL1_RX_LINE;
   
    // Start UART and threads
    uartStart(rotmech.port, &rotmech.uart_cfg);
    small_rotmech_init_sequence();

    // Start serial thread (lower priority) with dynamic allocation
    CREATE_DYNAMIC_THREAD("small_rotmech_serial", 512,  NORMALPRIO-2, small_rotmech_serial_thd, NULL);
    CREATE_DYNAMIC_THREAD("rotmech_telem", 256, NORMALPRIO-6, rotmech_telem_thd, NULL);
}

void small_rotmech_init_sequence(void) {
    if(rotmech_is_0_switch_triggered()) {
        small_rotmech_unreach_endstop();
        chThdSleepMilliseconds(2000);
    }
    small_rotmech_setup_angle();
    small_rotmech_reach_endstop();

    servo_update_status();

    // Set initialized flag
    rotmech_initialized = true;
}

/**
 * @brief Initialize big rotation mechanism
 * Uses encoder and an esc to control wing position using PID
 */
void big_rotmech_init(void) {    
    common_rotmech_init();

    // Load servo configuration from config
    rotmech.config.gear_ratio = 12;
    // TO BE CHANGED AFTER CALIBRATION
    rotmech.config.physical_offset = 0;
    rotmech.safe_load = true;
    rotmech.config.init_speed = config_get_by_name("ROTMECH init speed", 0)->val.i;
    if (rotmech.config.init_speed == 0) {
        rotmech.config.init_speed = 100; // Default init speed
    }


    // Encoder pins
    palSetLineMode(SERIAL2_TX_LINE, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    palSetLineMode(SERIAL2_RX_LINE, PAL_MODE_INPUT_PULLUP);
    rotmech.port = &UARTD2;

    // Switch Pins
    rotmech.switch0_pin = SERIAL1_TX_LINE;
    rotmech.switch90_pin = SERIAL1_RX_LINE;
    palSetLineMode(rotmech.switch0_pin, PAL_MODE_INPUT_PULLUP);
    palSetLineMode(rotmech.switch90_pin, PAL_MODE_INPUT_PULLUP);

    // ESC Pins
    if(rotmech.type == big_rotmech) {        
        rotmech.esc_port = config_get_by_name("ROTMECH esc port", 0)->val.i;
         switch (rotmech.esc_port)
        {
        case 1:
            rotmech.esc_pin = SERVO1_LINE;
            break;

        case 2:
            rotmech.esc_pin = SERVO2_LINE;
            break;

        case 3:
            rotmech.esc_pin = SERVO3_LINE;
            break;

        case 4:
            rotmech.esc_pin = SERVO4_LINE;
            break;

        case 5:
            rotmech.esc_pin = SERVO5_LINE;
            break;

        case 6:
            rotmech.esc_pin = SERVO6_LINE;
            break;

        case 7:
            rotmech.esc_pin = SERVO7_LINE;
            break;

        case 8:
            rotmech.esc_pin = SERVO8_LINE;
            break;

        case 9:
            rotmech.esc_pin = SERVO9_LINE;
            break;

        case 10:
            rotmech.esc_pin = SERVO10_LINE;
            break;

        default:
            break;
        }
        palSetLineMode(rotmech.esc_pin, PAL_MODE_STM32_ALTERNATE_PUSHPULL);
    }

    // Start UART
    uartStart(rotmech.port, &rotmech.uart_cfg);

    // Initialize PID and arm ESC for the controller
    if (rotmech.type == big_rotmech) {
        pid_init();
    }
    
    chThdSleepMilliseconds(2000);
    // Wait for board servos to be initialized
    while (!board_servos_initialized()) {
        chThdSleepMilliseconds(100);
    }
    rotmech_debug_message("Board servos initialized\n");

    // Start initialization sequence
    big_rotmech_init_sequence();

    // Start serial thread (lower priority) with dynamic allocation
    CREATE_DYNAMIC_THREAD("big_rotmech_serial", 512,  NORMALPRIO-2, big_rotmech_serial_thd, NULL);
    // CREATE_DYNAMIC_THREAD("rotmech_telem", 256, NORMALPRIO-6, rotmech_telem_thd, NULL);
}

void big_rotmech_init_sequence(void) {

    // Arm ESC
    big_rotmech_arm_esc();
    rotmech_debug_message("ESC armed\n");

    if(rotmech_is_0_switch_triggered()) {
        rotmech_debug_message("Unreaching endstop\n");
        big_rotmech_unreach_endstop();
        rotmech_debug_message("Unreached endstop\n");
        chThdSleepMilliseconds(2000);
    }
    rotmech_debug_message("Reaching endstop\n");
    big_rotmech_reach_endstop();
    rotmech_debug_message("Reached endstop\n");
}

// ============================================================================
// THREAD FUNCTIONS
// ============================================================================
static THD_FUNCTION(small_rotmech_serial_thd, arg) {
    (void)arg;
    chRegSetThreadName("small_rotmech_serial");
    // Main serial loop (only accessible after init is complete)
    while (rotmech.safe_load) {
        small_rotmech_move_to_target();
        servo_update_status();
        // Sleep based on configured serial frequency
        uint32_t period_ms = (uint32_t)(1000.0f / rotmech.config.serial_frequency);
        chThdSleepMilliseconds(period_ms);
    }
    small_rotmech_disarm();
}

static THD_FUNCTION(big_rotmech_serial_thd, arg) {
    (void)arg;
    chRegSetThreadName("big_rotmech_serial");
    pid_set_output_limits(150.0f);
    while (true) {
    // Only run PID control for big_rotmech (not big_rotmech_sensor)
    if (rotmech.type == big_rotmech) {
        if (config_get_by_name("ROTMECH arm", 0)->val.i == 1) {
            pid_update();
            rotmech_set_esc_pwm(rotmech.pwm_output);
        } else {
            servo_update_status();
            rotmech_set_esc_pwm(rotmech.pid.output_zero);
        }
    } else {
        // Just update encoder position for big_rotmech_sensor
        servo_update_status();
    }

    // Sleep based on configured serial frequency
    uint32_t period_ms = (uint32_t)(1000.0f / rotmech.config.serial_frequency);
    chThdSleepMilliseconds(period_ms);
    }
}

static THD_FUNCTION(rotmech_telem_thd, arg) {
  (void)arg;
  chRegSetThreadName("rotmech_telem");

  while (true) {
    broadcast_rotmech_status();
    uint32_t sleeptime = 1000 / rotmech.config.can_frequency;
    chThdSleepMilliseconds(sleeptime);
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
        JoinBytes(DataL, DataH & 0x7F, Data);
        *Data = -(*Data);
    } else {
        // Positive value
        JoinBytes(DataL, DataH, Data);
    }
}

void FeetechSignedLoadtoSignedInt(uint8_t DataL,uint8_t DataH, int16_t *Data)
{
    // Check if sign bit is set
    if (DataH & 0x04) {
        JoinBytes(DataL, DataH & 0xFB, Data);
        *Data = -(*Data);
    } else {
        // Positive value
        JoinBytes(DataL, DataH, Data);
    }
}

void ClampWingPosition(int16_t *wing_position_cdg) {
    if (*wing_position_cdg < rotmech.config.min_angle) {
        *wing_position_cdg = rotmech.config.min_angle;
    } else if (*wing_position_cdg > rotmech.config.max_angle) {
        *wing_position_cdg = rotmech.config.max_angle;
    }
}

int16_t ServoPositionToWingPosition(int16_t servo_position) {
    // Convert servo position to wing position
    // Assuming 0-4096 range for angles, adjust based on gear ratio
    servo_position -= rotmech.calculation_offset;
    int16_t wing_target_cdg = (int16_t)((float)(servo_position *DEGREE_TO_CENTIDEGFREE / (rotmech.config.gear_ratio * DEGREE_TO_FEETECH)));
    return wing_target_cdg;
}

int16_t WingPositionToServoPosition(int16_t wing_position_cdg) {
    // Convert wing position back to servo position
    // Assuming 0-4096 range for angles, adjust based on gear ratio
    int16_t servo_target = (int16_t)((float)(wing_position_cdg * rotmech.config.gear_ratio * CENTIDEGREE_TO_DEGREE * DEGREE_TO_FEETECH));
    servo_target += rotmech.calculation_offset;

    return servo_target;
}


// ============================================================================
// LOW-LEVEL COMMUNICATION FUNCTIONS
// ============================================================================

// Function to send an instruction to the servo
void send_instruction(uint8_t servo_id, uint8_t instruction, uint8_t *params, uint8_t param_count) {
    if (rotmech.port == NULL) return;
    
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
    uartSendFullTimeout(rotmech.port, &len, buffer_tx, TIME_MS2I(1));
}

// Function to read servo response
bool receive_servo_response(uint8_t *response, uint8_t *length, uint32_t timeout_ms) {
    if (rotmech.port == NULL) return false;
    
    size_t recv_size = 50;
    msg_t result = uartReceiveTimeout(rotmech.port, &recv_size, response, TIME_MS2I(timeout_ms));
    
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
    if (response[2] != rotmech.servo_id) {
        return SERVO_RESPONSE_INVALID_SERVO_ID;
    }
    
    // // Check if servo reported an error
    // if (rotmech.latest_error_code != 0) {
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
    rotmech.latest_error_code = response[4];

    // Parse data if available (8 bytes: pos, speed, load, voltage, temp)
    if (data_length >= 9) // 1 error byte + 8 data bytes
    {
        rotmech.previous_servo_position = rotmech.latest_servo_position;
        rotmech.latest_servo_position = (int16_t)(response[5] | (response[6] << 8));
        
        // Check for revolution crossing
        revolution_count_update();
        
        global_position_update();
        
        rotmech.latest_wing_angle = ServoPositionToWingPosition(rotmech.current_position_global);

        FeetechSignedtoSignedInt(response[7], response[8], &rotmech.latest_servo_speed);
        FeetechSignedLoadtoSignedInt(response[9], response[10], &rotmech.latest_load);

        rotmech.latest_voltage = response[11];
        rotmech.latest_temperature = response[12];
    }
    
    return SERVO_RESPONSE_SUCCESS;
}

// ============================================================================
// HIGH-LEVEL SERVO CONTROL FUNCTIONS
// ============================================================================
void small_rotmech_unreach_endstop(void) {
    endstop_reached = true;
    servo_update_status();
    rotmech.target_wing_angle = 1500; // Angle to move away from endstop
    rotmech.target_servo_position = WingPositionToServoPosition(rotmech.target_wing_angle);
    small_rotmech_set_position(rotmech.target_servo_position, 200);
    while(endstop_reached){
        // Update status
        servo_update_status();
        if (rotmech.target_wing_angle - rotmech.latest_wing_angle < 5){
            endstop_reached = false;
        }
    }
}


void small_rotmech_reach_endstop(void) {
    small_rotmech_set_position(STS_MAX_POSITION_GLOBAL, 200);
    while(palReadLine(rotmech.switch0_pin) & rotmech.safe_load) {
        // Update status
        servo_update_status();
    }
    rotmech.target_servo_position = rotmech.current_position_global;
    small_rotmech_set_position(rotmech.target_servo_position, 200);
    rotmech.calculation_offset = rotmech.current_position_global + WingPositionToServoPosition(rotmech.config.physical_offset);
    rotmech.target_wing_angle = ServoPositionToWingPosition(rotmech.target_servo_position);
}

// Request servo response for a specific address and length
void request_servo_response(uint8_t read_address, uint8_t read_length) {
    uint8_t data_read[2] = {read_address, read_length}; // Read specified length from position
    send_instruction(rotmech.servo_id, INST_READ, data_read, sizeof(data_read));
}

// ============================================================================
// HIGH-LEVEL SERVO CONTROL FUNCTIONS
// ============================================================================
void big_rotmech_arm_esc(void) {
    if (rotmech.port == NULL || rotmech.type != big_rotmech) {
        return;
    }
    rotmech_set_esc_pwm(rotmech.pid.output_zero);
    chThdSleepMilliseconds(2000); // Wait for ESC to arm
}

void big_rotmech_unreach_endstop(void) {
    if (rotmech.port == NULL || rotmech.type == small_rotmech) {
        return;
    }
    endstop_reached = true;
    if (rotmech.type == big_rotmech) {
        uint16_t speed = rotmech.pid.output_zero - rotmech.config.init_speed;
        rotmech_set_esc_pwm(speed);
    }
    while(endstop_reached){
        // Update status
        if (!rotmech_is_0_switch_triggered()){
            endstop_reached = false;
        }
    }
}


void big_rotmech_reach_endstop(void) {
    // Move clockwise until endstop is reached
    if (rotmech.port == NULL || rotmech.type == small_rotmech) {
        return;
    }

    if (rotmech.type == big_rotmech) {
        uint16_t speed = rotmech.pid.output_zero + rotmech.config.init_speed;
        rotmech_set_esc_pwm(speed);
    }
    while(!rotmech_is_0_switch_triggered()) {
        // Update status
        chThdSleepMilliseconds(10);
    }
    servo_update_status();

    rotmech.current_position_global = rotmech.latest_servo_position;
    rotmech.calculation_offset = rotmech.current_position_global + WingPositionToServoPosition(rotmech.config.physical_offset);
    rotmech.target_servo_position = rotmech.current_position_global;
    rotmech.target_wing_angle = ServoPositionToWingPosition(rotmech.target_servo_position);

    rotmech_debug_message("Big rotmech endstop reached. Offset: %d\n", rotmech.calculation_offset);
    rotmech_debug_message("Big rotmech current position: %d\n", rotmech.current_position_global);
    rotmech_debug_message("Big rotmech target position: %d\n", rotmech.target_servo_position);
    rotmech_debug_message("Big rotmech target angle: %d\n", rotmech.target_wing_angle);
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
    if (rotmech.port == NULL) {
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


    // Check small rotmech load conditions
    if (!rotmech_initialized && rotmech.type == small_rotmech){
        if (abs(rotmech.latest_load) >= rotmech.config.load_threshold){
            // Trigger unsafe load condition if load exceeds threshold
            rotmech.safe_load = false;
        }
    }

    return result;
}

// ============================================================================
// UAVCAN INTERFACE FUNCTIONS
// ============================================================================

void broadcast_rotmech_status(void) {
  // Set the values
  struct uavcan_equipment_actuator_Status actuatorStatus;
  actuatorStatus.actuator_id = rotmech.index;
  actuatorStatus.position = (float)rotmech.latest_wing_angle * M_PI / 18000.0f; // Convert centidegrees to radians
  actuatorStatus.speed = (float)rotmech.latest_servo_speed;
  actuatorStatus.force = (float)rotmech.latest_load;

  uint8_t buffer[UAVCAN_EQUIPMENT_ACTUATOR_STATUS_MAX_SIZE];
  uint16_t total_size = uavcan_equipment_actuator_Status_encode(&actuatorStatus, buffer);

  static uint8_t transfer_id;
  uavcanBroadcastAll(
      UAVCAN_EQUIPMENT_ACTUATOR_STATUS_SIGNATURE,
      UAVCAN_EQUIPMENT_ACTUATOR_STATUS_ID, &transfer_id,
      CANARD_TRANSFER_PRIORITY_LOW, buffer, total_size);

}

// ============================================================================
// DEBUGGING FUNCTIONS
// ============================================================================

void rotmech_debug_message(const char *fmt, ...) {
    if (rotmech.port == NULL) return;

    char buffer[128];  // adjust size as needed
    va_list args;
    va_start(args, fmt);
    chvsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    uavcanDebug(UAVCAN_PROTOCOL_DEBUG_LOGLEVEL_INFO, "ROTMECH", buffer);
}

void small_rotmech_debug_status(void) {
    rotmech_debug_message(
        "A:%d CurA:%d TgtA:%d Off:%d CurP:%d TgtP:%d Spd:%d Ld:%d "
        "T:%d V:%d Err:%d H:%d",
        rotmech.armed,
        rotmech.latest_wing_angle,
        rotmech.target_wing_angle,
        rotmech.calculation_offset,
        rotmech.latest_servo_position,
        rotmech.target_servo_position,
        rotmech.latest_servo_speed,
        (uint8_t)((rotmech.latest_load >= 0 ? rotmech.latest_load 
                   : -rotmech.latest_load) / 10),
        rotmech.latest_temperature,
        rotmech.latest_voltage,
        rotmech.latest_error_code,
        rotmech.latest_health_state
    );
}

void big_rotmech_debug_status(void) {
    // Convert speed from servo units to centidegree/s
    // Same scaling as position conversion but without offset
    float speed_cdeg_s = (float)rotmech.latest_servo_speed * 
                         DEGREE_TO_CENTIDEGFREE / 
                         (rotmech.config.gear_ratio * DEGREE_TO_FEETECH);
    
    // Convert floats to int with decimal places
    int dt_ms = (int)(rotmech.pid.dt * 1000);
    
    int error_int = (int)rotmech.pid.error;
    
    int speed_int = (int)speed_cdeg_s;
    
    int kp_int = (int)(rotmech.pid.kp * 1000);
    int ki_int = (int)(rotmech.pid.ki * 1000);
    int kd_int = (int)(rotmech.pid.kd * 1000);
    
    int integral_int = (int)(rotmech.pid.integral * 100);
    
    rotmech_debug_message(
        "Ang:%d->%d Spd:%d Err:%d dt:%dms P:%d I:%d D:%d Int:%d PWM:%d",
        rotmech.latest_wing_angle,
        rotmech.target_wing_angle,
        speed_int,           // Speed in centidegree/s
        error_int,
        dt_ms,               // dt in milliseconds
        kp_int,              // Kp * 1000
        ki_int,              // Ki * 1000
        kd_int,              // Kd * 1000
        integral_int,        // Integral * 100
        (int)rotmech.pwm_output
    );
}

// ============================================================================
// CONFIGURATION FUNCTIONS
// ============================================================================

void small_rotmech_setup_angle(void) {
    if (rotmech.port == NULL || rotmech.type != small_rotmech) {
        return;
    }
    
    uint8_t write_unlock[2] = {0x37, 0};
    send_instruction(rotmech.servo_id, INST_WRITE, write_unlock, 2);
    chThdSleepMicroseconds(SERVO_SLEEP_TIME_US);
    
    uint8_t angle_limit[3] = {STS_MAXIMUM_ANGLE, 0, 0};
    send_instruction(rotmech.servo_id, INST_WRITE, angle_limit, 3);
    chThdSleepMicroseconds(SERVO_SLEEP_TIME_US);
    
    uint8_t write_lock[2] = {0x37, 1};
    send_instruction(rotmech.servo_id, INST_WRITE, write_lock, 2);
    chThdSleepMicroseconds(SERVO_SLEEP_TIME_US);

}

void small_rotmech_disarm(void) {
    if (rotmech.port == NULL || rotmech.type != small_rotmech) {
        return;
    }

    uint8_t params[2] = {STS_TORQUE_SWITCH, 0};
    send_instruction(rotmech.servo_id, INST_WRITE, params, sizeof(params));
    chThdSleepMicroseconds(SERVO_SLEEP_TIME_US);
}

void small_rotmech_set_position(int16_t position, int16_t speed) {
    if (rotmech.port == NULL || rotmech.type != small_rotmech) {
        return;
    }

    if(rotmech.armed == true) {
        uint8_t data_write[9] = {0};

        data_write[0] = STS_TORQUE_SWITCH;
        data_write[1] = rotmech.armed;
        data_write[2] = 0;
        SignedIntToFeetechSigned(&data_write[3], &data_write[4], position);
        SplitByte(&data_write[5], &data_write[6], 0);
        SignedIntToFeetechSigned(&data_write[7], &data_write[8], speed);

        // Send the command to the servo
        send_instruction(rotmech.servo_id, INST_WRITE, data_write, sizeof(data_write));
        chThdSleepMicroseconds(SERVO_SLEEP_TIME_US);
    }
    else {
        small_rotmech_disarm();
    }
}

void small_rotmech_move_to_target(void) {
    small_rotmech_set_position(rotmech.target_servo_position, 0);
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
        rotmech.previous_servo_position = rotmech.latest_servo_position;
        rotmech.current_position_global = rotmech.latest_servo_position;
        first_position_update = false;
    }
    else{
        int16_t position_difference = rotmech.latest_servo_position - rotmech.previous_servo_position;
        if (abs(position_difference) > REVOLUTION_CROSSING_THRESHOLD) {
            // If the position has wrapped around (from 4095 to 0), increment revolution count
            if (position_difference < 0) {
                rotmech.revolution_count++;
            } else {
            // If the position has wrapped around (from 0 to 4095), decrement revolution count
                rotmech.revolution_count--;
            }
        }
    }
}

void global_position_update(void) {
    // Update the global position based on the current local position and revolution count
    rotmech.current_position_global = local_to_global_position(rotmech.revolution_count, rotmech.latest_servo_position);
    
    // Check for overflow/underflow
    if (rotmech.current_position_global < STS_MIN_POSITION_GLOBAL) {
        rotmech.current_position_global = STS_MIN_POSITION_GLOBAL;
    } else if (rotmech.current_position_global > STS_MAX_POSITION_GLOBAL) {
        rotmech.current_position_global = STS_MAX_POSITION_GLOBAL;
    }
}

// ============================================================================
// PID CONTROL FUNCTIONS
// ============================================================================

/**
 * @brief Initialize PID controller
 */
void pid_init(void) {
    rotmech.pid.kp = 0.10f;
    rotmech.pid.ki = 0.0f;
    rotmech.pid.kd = 0.0f;
    
    // Set optimization flags
    rotmech.pid.use_p = (rotmech.pid.kp != 0.0f);
    rotmech.pid.use_i = (rotmech.pid.ki != 0.0f);
    rotmech.pid.use_d = (rotmech.pid.kd != 0.0f);
    
    rotmech.pid.error = 0.0f;
    rotmech.pid.error_deadband = 50.0f;
    rotmech.pid.prev_error = 0.0f;
    rotmech.pid.integral = 0.0f;
    rotmech.pid.derivative = 0.0f;
    rotmech.pid.output_zero = 1500.0f;
    
    // Hardcoded output limits
    rotmech.pid.output_min = 1400.0f;
    rotmech.pid.output_max = 1600.0f;
    
    // Hardcoded integral limits (anti-windup)
    rotmech.pid.integral_min = -100.0f;
    rotmech.pid.integral_max = 100.0f;
    
    rotmech.pid.last_time_ms = chVTGetSystemTime();
    rotmech.pid.dt = 0.0f;

    rotmech.pid.lin_min_kp = 0.005f;
    rotmech.pid.lin_max_kp = 10.0f;
    rotmech.pid.lin_max_error = 500.0f;
    rotmech.pid.lin_min_error = 0.0f;


    rotmech.pid.exp_min_kp = 0.05f;
    rotmech.pid.exp_max_kp = 0.5f;
    rotmech.pid.exp_max_error = 500.0f;
    rotmech.pid.exp_min_error = 0.0f;
}

/**
 * @brief Update PID controller
 * @return PID output
 */
void pid_update(void) {
    // Calculate time delta using ChibiOS time handling
    servo_update_status();
    systime_t current_time = chVTGetSystemTime();
    systime_t time_diff = chVTTimeElapsedSinceX(rotmech.pid.last_time_ms);
    rotmech.pid.last_time_ms = current_time;
    
    // Convert to seconds (handles wraparound properly)
    rotmech.pid.dt = (float)TIME_I2MS(time_diff) / 1000.0f;
    
    // Prevent division by zero and handle first call
    if (rotmech.pid.dt <= 0.0f || rotmech.pid.dt > 1.0f) {
        rotmech.pid.dt = 0.001f;
    }
    
    // Calculate error (target - current)
    rotmech.pid.error = (float)rotmech.target_wing_angle - 
                             (float)rotmech.latest_wing_angle;


    rotmech.pid.kp = pid_calculate_exponential_kp(rotmech.pid.error);

    if (rotmech.pid.error > -rotmech.pid.error_deadband && rotmech.pid.error < rotmech.pid.error_deadband) {
        rotmech.pid.error = 0;
    }
    
    // Initialize output
    float output = rotmech.pid.output_zero;
    
    // Proportional term
    if (rotmech.pid.use_p) {
        output += rotmech.pid.kp * rotmech.pid.error;
    }
    
    // Integral term
    if (rotmech.pid.use_i) {
        rotmech.pid.integral += rotmech.pid.error * rotmech.pid.dt;
        
        // Anti-windup
        if (rotmech.pid.integral > rotmech.pid.integral_max) {
            rotmech.pid.integral = rotmech.pid.integral_max;
        } else if (rotmech.pid.integral < rotmech.pid.integral_min) {
            rotmech.pid.integral = rotmech.pid.integral_min;
        }
        
        output += rotmech.pid.ki * rotmech.pid.integral;
    } else {
        rotmech.pid.integral = 0.0f;
    }
    
    // Derivative term
    if (rotmech.pid.use_d) {
        rotmech.pid.derivative = (rotmech.pid.error - 
                                      rotmech.pid.prev_error) / 
                                      rotmech.pid.dt;
        output += rotmech.pid.kd * rotmech.pid.derivative;
    } else {
        rotmech.pid.derivative = 0.0f;
    }

    // Invert output for proper ESC direction
    output = rotmech.pid.output_zero + 
            -1.0f * (output - rotmech.pid.output_zero);
    
    // Clamp output
    if (output > rotmech.pid.output_max) {
        output = rotmech.pid.output_max;
    } else if (output < rotmech.pid.output_min) {
        output = rotmech.pid.output_min;
    }
    
    // Store error for next iteration
    rotmech.pid.prev_error = rotmech.pid.error;

    // Set PWM output
    rotmech.pwm_output = output;
}

/**
 * @brief Reset PID controller
 */
void pid_reset(void) {
    rotmech.pid.error = 0.0f;
    rotmech.pid.prev_error = 0.0f;
    rotmech.pid.integral = 0.0f;
    rotmech.pid.derivative = 0.0f;
    rotmech.pid.last_time_ms = chVTGetSystemTime();
    rotmech.pwm_output = rotmech.pid.output_zero;

}

/**
 * @brief Update PID gains and optimization flags
 */
void pid_set_gains(float kp, float ki, float kd) {
    rotmech.pid.kp = kp;
    rotmech.pid.ki = ki;
    rotmech.pid.kd = kd;
    
    // Update optimization flags
    rotmech.pid.use_p = (kp != 0.0f);
    rotmech.pid.use_i = (ki != 0.0f);
    rotmech.pid.use_d = (kd != 0.0f);
    
    // Reset controller state
    pid_reset();
}

/**
 * @brief Update PID output limits
 */
void pid_set_output_limits(float range) {
    rotmech.pid.output_min = rotmech.pid.output_zero - range;
    rotmech.pid.output_max = rotmech.pid.output_zero + range;
}

/**
 * @brief Update PID integral limits (anti-windup)
 */
void pid_set_integral_limits(float limit) {
    rotmech.pid.integral_min = -limit;
    rotmech.pid.integral_max = limit;

    // Clamp current integral if needed
    if (rotmech.pid.integral > limit) {
        rotmech.pid.integral = limit;
    } else if (rotmech.pid.integral < -limit) {
        rotmech.pid.integral = -limit;
    }
}

bool rotmech_is_0_switch_triggered(void) {
    if (rotmech.port == NULL) return false;

    return !palReadLine(rotmech.switch0_pin);
}

bool rotmech_is_90_switch_triggered(void) {
    if (rotmech.port == NULL) return false;

    return !palReadLine(rotmech.switch90_pin);
}

void rotmech_set_esc_pwm(uint16_t value) {
    if (rotmech.type == big_rotmech) {
        board_set_servo_raw(rotmech.esc_port, value);
        big_rotmech_debug_status();
    }
}

/**
 * @brief Dynamically adjust Kp based on error magnitude
 * @param error Current position error
 * @return Adjusted Kp value
 */
float pid_calculate_linear_kp(float error) {
    
    float abs_error = fabsf(error);
    float kp;

    if (abs_error <= rotmech.pid.lin_min_error) {
        // Low error plateau
        kp = rotmech.pid.lin_min_kp;
    } 
    else if (abs_error >= rotmech.pid.lin_max_error) {
        // High error plateau
        kp = rotmech.pid.lin_max_kp;
    } 
    else {
        // Linear ramp region
        float ramp_progress = (abs_error - rotmech.pid.lin_min_error) / (rotmech.pid.lin_max_error - rotmech.pid.lin_min_error);
        kp = rotmech.pid.lin_min_kp + (rotmech.pid.lin_max_kp - rotmech.pid.lin_min_kp) * ramp_progress;
    }
    
    return kp;
}

/**
 * @brief Exponentially adjust Kp based on error magnitude with plateau
 * @param error Current position error
 * @return Adjusted Kp value
 */
float pid_calculate_exponential_kp(float error) {
    float abs_error = fabsf(error);
    float kp;

    if (abs_error <= rotmech.pid.exp_min_error) {
        // Low error plateau
        kp = rotmech.pid.exp_min_kp;
    } 
    else if (abs_error >= rotmech.pid.exp_max_error) {
        // High error plateau
        kp = rotmech.pid.exp_max_kp;
    } 
    else {
        // Exponential growth region
        // lambda = ln(100) / error_plateau gives ~99% at plateau
        float lambda = 4.605f / rotmech.pid.exp_max_error;
        
        // Exponential formula: kp = KP_PLATEAU * (1 - e^(-lambda * error))
        kp = rotmech.pid.exp_max_kp * 
             (1.0f - expf(-lambda * abs_error));

        return kp;
    }
}