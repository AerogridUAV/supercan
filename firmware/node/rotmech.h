#ifndef ROTMECH_H
#define ROTMECH_H

#include <hal.h>
#include <uavcan.h>

// ============================================================================
// CONSTANTS AND DEFINITIONS
// ============================================================================
typedef enum
{
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

#define SERVO_SLEEP_TIME_MS 1    ///< Servo sleep time in milliseconds
#define SERVO_SLEEP_TIME_US 1000 ///< Servo sleep time in microseconds

#define DEGREE_TO_FEETECH -4096.0f / 360.0f         ///< Conversion factor from degrees to Feetech servo units
#define DEGREE_TO_CENTIDEGFREE 100.0f               ///< Conversion factor from degrees to centidegrees      
#define CENTIDEGREE_TO_DEGREE 0.01f                 ///< Conversion factor from centidegrees to degrees
#define CENTIDEGREE_TO_FEETECH -4096.0f / 36000.0f  ///< Conversion factor from centidegrees to Feetech servo units

// Minimum and maximum local positions
#define STS_MIN_POSITION_LOCAL 0        ///< Minimum local position (servo units)
#define STS_MAX_POSITION_LOCAL 4095     ///< Maximum local position (servo units)

// Minimum and maximum global positions
#define STS_MIN_POSITION_GLOBAL -30719  ///< Minimum global position (servo units)
#define STS_MAX_POSITION_GLOBAL 30719   ///< Maximum global position (servo units)

// If revolution crossing threshold is exceeded in a single tick update, it is considered a revolution crossing
#define REVOLUTION_CROSSING_THRESHOLD 1100 ///< Threshold for detecting revolution crossing (servo units)

// Response states
typedef enum
{
    SERVO_RESPONSE_SUCCESS = 0,
    SERVO_RESPONSE_INVALID_HEADER,
    SERVO_RESPONSE_CHECKSUM_MISMATCH,
    SERVO_RESPONSE_INSUFFICIENT_LENGTH,
    SERVO_RESPONSE_INVALID_SERVO_ID,
    SERVO_RESPONSE_TIMEOUT,
    SERVO_RESPONSE_ERROR
} servo_response_t;

typedef enum
{
    small_rotmech,
    big_rotmech,
    big_rotmech_sensor
} rotmech_type_t;

// Servo configuration structure
struct controller_config_t
{
    uint8_t gear_ratio;         ///< Gear Ratio from servo to wing rotation mechanism
    uint16_t min_angle;         ///< Minimum angle in centidegrees
    uint16_t max_angle;         ///< Maximum angle in centidegrees
    int16_t physical_offset;    ///< Physical offset in centidegrees
    float serial_frequency;     ///< Serial communication frequency
    float can_frequency;        ///< CAN communication frequency
    uint16_t max_speed;         ///< Maximum speed
    uint32_t load_threshold;    ///< Load threshold

    float pid_kp;               ///< PID proportional gain
    float pid_ki;               ///< PID integral gain
    float pid_kd;               ///< PID derivative gain
    float pid_output_min;       ///< PID output minimum limit
    float pid_output_max;       ///< PID output maximum limit
    float pid_integral_min;     ///< PID integral minimum limit
    float pid_integral_max;     ///< PID integral maximum limit
};

// PID Controller structure
struct pid_controller_t
{
    float kp; ///< Proportional gain
    float ki; ///< Integral gain
    float kd; ///< Derivative gain

    // Optimization flags
    bool use_p; ///< Boolean that determines whether to use P term
    bool use_i; ///< Boolean that determines whether to use I term
    bool use_d; ///< Boolean that determines whether to use D term

    float error;      ///< Current error (target - current)
    float prev_error; ///< Previous error (for derivative calculation)
    float integral;   ///< Integral accumulator (sum of errors over time)
    float derivative; ///< Derivative term (rate of change of error)

    float output_zero;  ///< Base PWM output value
    float output_min;   ///< Minimum output limit
    float output_max;   ///< Maximum output limit

    float integral_min; ///< Anti-windup limit
    float integral_max; ///< Anti-windup limit

    systime_t last_time_ms; ///< Last update time (ChibiOS systime_t)
    float dt;               ///< Delta time in seconds
};

// Main servo state structure
struct rotmech_t
{
    UARTDriver *port;          ///< UART port for communication
    UARTConfig uart_cfg;       ///< UART configuration
    uint8_t index;             ///< Rotation mechanism actuator index
    uint8_t servo_id;          ///< Servo ID
    ioline_t esc_pin;          ///< ESC control pin
    ioline_t switch0_pin;      ///< 0 Degree Wing Switch pin
    ioline_t switch90_pin;     ///< 90 Degree Wing Switch pin
    rotmech_type_t type;       ///< Rotation mechanism type
    uint16_t pwm_output;       ///< PWM output value

    // Configuration
    struct controller_config_t config; ///< Controller configuration

    // Controllers
    struct pid_controller_t pid; ///< PID controller

    // Control state
    bool armed;
    int16_t target_servo_position; ///< Target servo position
    int16_t target_servo_speed;    ///< Target servo speed
    int16_t target_wing_angle;     ///< Target wing angle in centidegrees
    int16_t calculation_offset;    ///< Calculation offset after calibration

    int16_t current_position_global; ///< Current position in global coordinates
    int16_t revolution_count;        ///< Number of revolution crossings

    int16_t previous_servo_position; ///< Previous servo position

    // Latest readings from servo
    int16_t latest_servo_position;  ///< Latest servo position
    int16_t latest_servo_speed;     ///< Latest servo speed
    int16_t latest_wing_angle;      ///< Latest wing angle
    int16_t latest_load;            ///< Latest load
    uint8_t latest_voltage;         ///< Latest voltage
    uint8_t latest_temperature;     ///< Latest temperature
    uint8_t latest_error_code;      ///< Latest error code
    uint8_t latest_health_state;    ///< Latest health state

    bool safe_load;
};

struct debug_message_t
{
    bool send;                 ///< Whether to send debug message
    char message[100];         ///< Buffer containing the debug message
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================

extern struct rotmech_t rotmech;
extern struct debug_message_t debug_msg;
extern bool rotmech_initialized;

// ============================================================================
// Main initialization and thread functions
void rotmech_init(void);
void common_rotmech_init(void);

void small_rotmech_init_sequence(void);
void small_rotmech_init(void);

void big_rotmech_init_sequence(void);
void big_rotmech_init(void);

// Utility functions
void SplitByte(uint8_t *DataL, uint8_t *DataH, int16_t Data);
void JoinBytes(uint8_t DataL, uint8_t DataH, int16_t *Data);
void SignedIntToFeetechSigned(uint8_t *DataL, uint8_t *DataH, int16_t Data);
void FeetechSignedtoSignedInt(uint8_t DataL, uint8_t DataH, int16_t *Data);
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
void small_rotmech_unreach_endstop(void);
void small_rotmech_reach_endstop(void);

// Configuration functions
void small_rotmech_setup_angle(void);

// Control functions
void small_rotmech_disarm(void);
void small_rotmech_set_position(int16_t position, int16_t speed);
void small_rotmech_move_to_target(void);

// UAVCAN interface functions
void broadcast_rotmech_status(void);
void broadcast_rotmech_debug(struct uavcan_iface_t *iface);

// Debugging functions
void rotmech_debug_message(const char *message);
void rotmech_debug_status(void);

// Position calculation functions
int16_t local_to_global_position(int16_t revolution_count, int16_t local_position);
int16_t global_to_local_position(int16_t revolution_count, int16_t global_position);
void revolution_count_update(void);
void global_position_update(void);

// PID controller functions
void pid_init(void);
void pid_update(void);
void pid_reset(void);
void pid_set_gains(float kp, float ki, float kd);
void pid_set_output_limits(float min, float max);
void pid_set_integral_limits(float min, float max);

#endif /* ROTMECH_H */