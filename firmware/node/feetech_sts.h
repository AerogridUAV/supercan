#ifndef FEETECH_STS_H
#define FEETECH_STS_H

#include <hal.h>
#include "uavcan.h"

// Servo Instruction Set
#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_REG_WRITE 0x04
#define INST_REG_ACTION 0x05
#define INST_SYNC_WRITE 0x83

// Memory Address
//-------EPROM(Read only)--------
#define STS_VERSION_L 0x03
#define STS_VERSION_H 0x04

//-------EPROM(Read And write)--------
#define STS_ID 0x05
#define STS_BAUD_RATE 0x06
#define STS_MIN_ANGLE_LIMIT_L 0x09
#define STS_MIN_ANGLE_LIMIT_H 0x0A
#define STS_MAX_ANGLE_LIMIT_L 0x0B
#define STS_MAX_ANGLE_LIMIT_H 0x0C
#define STS_CW_DEAD 0x1A
#define STS_CCW_DEAD 0x1B

#define STS_WORK_MODE 0x21

//-------SRAM(Read & Write)--------
#define STS_TORQUE_ENABLE 0x28
#define STS_GOAL_POSITION_L 0x2A
#define STS_GOAL_POSITION_H 0x2B
#define STS_GOAL_TIME_L 0x2C
#define STS_GOAL_TIME_H 0x2D
#define STS_GOAL_SPEED_L 0x2E
#define STS_GOAL_SPEED_H 0x2F
#define STS_LOCK 0x30

//-------SRAM(Read Only)--------
#define STS_PRESENT_POSITION_L 0x38
#define STS_PRESENT_POSITION_H 0x39
#define STS_PRESENT_SPEED_L 0x3A
#define STS_PRESENT_SPEED_H 0x3B
#define STS_PRESENT_LOAD_L 0x3C
#define STS_PRESENT_LOAD_H 0x3D
#define STS_PRESENT_VOLTAGE 0x3E
#define STS_PRESENT_TEMPERATURE 0x3F
#define STS_MOVING 0x42
#define STS_PRESENT_CURRENT_L 0x45
#define STS_PRESENT_CURRENT_H 0x46


struct feetech_sts_t {
    UARTDriver *port;
    UARTConfig uart_cfg;
    uint8_t servo_id;
    uint8_t index;
    
    // Command handling
    int16_t target_speed;
    bool torque_enabled;
    
    // Servo feedback
    int16_t present_position;
    int16_t present_speed;
    int16_t present_load;
    uint8_t present_voltage;      // Add voltage feedback
    uint8_t present_temperature;  // Add temperature feedback
    uint8_t status_transfer_id;

    // Error handling
    uint8_t error_code; // Add error code field
    uint8_t checksum;
};

extern struct feetech_sts_t feetech_sts;

void feetech_sts_init(void);
void broadcast_feetech_status(struct uavcan_iface_t *iface);

#endif /* FEETECH_STS_H */