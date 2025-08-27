#ifndef FEETECH_STS_H
#define FEETECH_STS_H

#include <hal.h>
#include <uavcan.h>
#include <com.feetech.servo.Instruction.h>
#include <com.feetech.servo.Config.h>
#include <com.feetech.servo.Status.h>
#include <com.feetech.servo.Debug.h>

// ============================================================================
// CONSTANTS AND DEFINITIONS
// ============================================================================
typedef enum {
    INST_PING = 0x01,
    INST_READ = 0x02,
    INST_WRITE = 0x03,
    INST_REG_WRITE = 0x04,
    INST_REG_ACTION = 0x05,
    INST_SYNC_WRITE = 0x83
} instruction_t;

// Memory Address
//-------EPROM(Read only)--------
#define STS_FIRMWARE_MAJOR 0x00
#define STS_FIRMWARE_MINOR 0x01
#define STS_SERVO_MAJOR 0x03
#define STS_SERVO_MINOR 0x04

//-------EPROM(Read And write)--------
#define STS_ID 0x05
#define STS_BAUDRATE 0x06
#define STS_RESPONSE_DELAY 0x07
#define STS_RESPONSE_STATUS_LEVEL 0x08
#define STS_MINIMUM_ANGLE 0x09
#define STS_MAXIMUM_ANGLE 0x0B
#define STS_MAXIMUM_TEMPERATURE 0x0D
#define STS_MAXIMUM_VOLTAGE 0x0E
#define STS_MINIMUM_VOLTAGE 0x0F
#define STS_MAXIMUM_TORQUE 0x10
#define STS_UNLOADING_CONDITION 0x13
#define STS_LED_ALARM_CONDITION 0x14
#define STS_POS_PROPORTIONAL_GAIN 0x15
#define STS_POS_DERIVATIVE_GAIN 0x16
#define STS_POS_INTEGRAL_GAIN 0x17
#define STS_MINIMUM_STARTUP_FORCE 0x18
#define STS_CK_INSENSITIVE_AREA 0x1A
#define STS_CCK_INSENSITIVE_AREA 0x1B
#define STS_CURRENT_PROTECTION_TH 0x1C
#define STS_ANGULAR_RESOLUTION 0x1E
#define STS_POSITION_CORRECTION 0x1F
#define STS_OPERATION_MODE 0x21
#define STS_TORQUE_PROTECTION_TH 0x22
#define STS_TORQUE_PROTECTION_TIME 0x23
#define STS_OVERLOAD_TORQUE 0x24
#define STS_SPEED_PROPORTIONAL_GAIN 0x25
#define STS_OVERCURRENT_TIME 0x26
#define STS_SPEED_INTEGRAL_GAIN 0x27

//-------SRAM(Read & Write)--------
#define STS_TORQUE_SWITCH 0x28
#define STS_TARGET_ACCELERATION 0x29
#define STS_TARGET_POSITION 0x2A
#define STS_RUNNING_TIME 0x2C
#define STS_RUNNING_SPEED 0x2E
#define STS_TORQUE_LIMIT 0x30
#define STS_WRITE_LOCK 0x37

//-------SRAM(Read Only)--------
#define STS_CURRENT_POSITION 0x38
#define STS_CURRENT_SPEED 0x3A
#define STS_CURRENT_DRIVE_VOLTAGE 0x3C
#define STS_CURRENT_VOLTAGE 0x3E
#define STS_CURRENT_TEMPERATURE 0x3F
#define STS_ASYNCHRONOUS_WRITE_ST 0x40
#define STS_STATUS 0x41
#define STS_MOVING_STATUS 0x42
#define STS_CURRENT_CURRENT 0x45
#define STS_MOVEMENT_SPEED_THRESHOLD 0x50
#define STS_DT 0x51
#define STS_SPEED_UNIT_COEFFICIENT 0x52
#define STS_HTS 0x53
#define STS_MAXIMUM_SPEED_LIMIT 0x54
#define STS_ACCELERATION_LIMIT 0x55
#define STS_ACCELERATION_MULTIPLIER 0x56


#define SERVO_SLEEP_TIME_MS 1 // us
#define SERVO_SLEEP_TIME_US 1000 // us

#define DEGREE_TO_FEETECH -4096.0f/360.0f // Conversion factor from degrees to Feetech servo units

// Minimum and maximum local positions
#define STS_MIN_POSITION_LOCAL 0
#define STS_MAX_POSITION_LOCAL 4095

// Minimum and maximum global positions
#define STS_MIN_POSITION_GLOBAL -30719
#define STS_MAX_POSITION_GLOBAL 30719

// If revolution crossing threshold is exceeded in a single tick update, it is considered a revolution crossing
#define REVOLUTION_CROSSING_THRESHOLD 1100

// Response states
typedef enum {
    SERVO_RESPONSE_SUCCESS = 0,
    SERVO_RESPONSE_INVALID_HEADER,
    SERVO_RESPONSE_CHECKSUM_MISMATCH,
    SERVO_RESPONSE_INSUFFICIENT_LENGTH,
    SERVO_RESPONSE_INVALID_SERVO_ID,
    SERVO_RESPONSE_TIMEOUT,
    SERVO_RESPONSE_ERROR
} servo_response_t;

typedef enum{
    short_log_message,
    long_log_message,
} loglevel_t;

// Servo configuration structure
struct servo_config_t {
    uint8_t gear_ratio;
    uint8_t min_angle;
    uint8_t max_angle;
    float serial_frequency;
    float can_frequency;
    loglevel_t log_level;
    uint16_t max_speed;
    uint32_t load_threshold;
    uint32_t integral_load_threshold;
    uint32_t integral_load_min_threshold;
    uint8_t load_timeout;
};

// Main servo state structure
struct feetech_sts_t {
    UARTDriver *port;
    UARTConfig uart_cfg;
    uint8_t index;
    uint8_t servo_id;
    
    // Configuration
    struct servo_config_t config;
    
    // Control state
    bool armed;
    int16_t target_servo_position;
    int16_t target_servo_speed;
    int16_t target_wing_angle;
    int16_t calculation_offset;

    int16_t current_position_global;
    int16_t revolution_count;
    
    
    int16_t previous_servo_position;

    // Latest readings from servo
    int16_t latest_servo_position;
    int16_t latest_servo_speed;
    int16_t latest_wing_angle;
    int16_t latest_load;
    uint8_t latest_voltage;
    uint8_t latest_temperature;
    uint8_t latest_error_code;
    uint8_t latest_health_state;

    bool safe_load;
    bool integral_load_active;
    uint16_t integral_load; 
    systime_t load_timer;
};

struct debug_message_t {
    bool send;
    char message[100];
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

extern struct feetech_sts_t feetech_sts;
extern struct debug_message_t debug_msg;
extern bool feetech_initialized;


// ============================================================================
// Main initialization and thread functions
static THD_WORKING_AREA(feetech_serial_wa, 512);
static THD_FUNCTION(feetech_serial_thd, arg);

void feetech_init_sequence(void);
void feetech_sts_init(void);


// Utility functions
void SplitByte(uint8_t *DataL, uint8_t *DataH, int16_t Data);
void JoinBytes(uint8_t DataL, uint8_t DataH, int16_t *Data);
void SignedIntToFeetechSigned(uint8_t *DataL,uint8_t *DataH, int16_t Data);
void FeetechSignedtoSignedInt(uint8_t DataL,uint8_t DataH, int16_t *Data);
int16_t ServoPositionToWingPosition(int16_t servo_position);
int16_t WingPositionToServoPosition(int16_t wing_position);

// Low-level communication functions
void send_instruction(uint8_t servo_id, uint8_t instruction, uint8_t *params, uint8_t param_count);
void request_servo_response(uint8_t read_address, uint8_t read_length);
bool receive_servo_response(uint8_t *response, uint8_t *length, uint32_t timeout_ms);

// Parse servo response
servo_response_t parse_servo_status(uint8_t *response, uint8_t length);
servo_response_t servo_update_status(void);

// High-level servo control functions
void unreach_endstop(void);
void reach_endstop(void);

// Configuration functions
void feetech_setup_angle(void);
loglevel_t feetech_get_log_level(int log_int);

// Control functions
void feetech_disarm(void);
void feetech_set_position(int16_t position, int16_t speed);
void feetech_move_to_target(void);

// UAVCAN interface functions
void handle_can_servo_instruction(struct uavcan_iface_t *iface, CanardRxTransfer* transfer);
void broadcast_feetech_status(struct uavcan_iface_t *iface);
void broadcast_servo_config(struct uavcan_iface_t *iface);
void broadcast_feetech_debug(struct uavcan_iface_t *iface);

// Debugging functions
void feetech_debug_message(const char *message);
void feetech_debug_status(void);

// Position calculation functions
int16_t local_to_global_position(int16_t revolution_count, int16_t local_position);
int16_t global_to_local_position(int16_t revolution_count, int16_t global_position);
void revolution_count_update(void);
void global_position_update(void);
#endif /* FEETECH_STS_H */