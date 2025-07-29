    
struct __attribute__((__packed__)) serial_act_t4_out {  
    //SINGLE SERVO telemetry & update rate 
    int16_t servo_1_angle_int; //Degrees * 10 
    int16_t servo_1_feedback_update_time_us; //MicroSeconds
    int16_t servo_1_load_int; 
    int16_t servo_1_speed_int; 
    int16_t servo_1_volt_int; 
    int16_t servo_1_temp_int; 
    float plate_dist_mm; 
    float plate_speed_mm_s; 
    float plate_load; 
    int8_t system_health; 
    //0 --> Initializing ; 
    //1 --> Operational  
    // -1 --> Failure (servo not communicating properly); 
    //Rolling message in 
    float rolling_msg_out;
    uint8_t rolling_msg_out_id;   
    //CHECKSUM
    uint8_t checksum_out;
};


struct __attribute__((__packed__)) serial_act_t4_in {
    //Single servo cmd
    float plate_target_dist_mm; //Plate distance in mm from the zero point
    //Rolling message out
    float rolling_msg_in;
    uint8_t rolling_msg_in_id;
    //CHECKSUM
    uint8_t checksum_in;
};


// Instruction set
#define INST_PING 0x01
#define INST_READ 0x02
#define INST_WRITE 0x03
#define INST_REG_WRITE 0x04
#define INST_REG_ACTION 0x05
#define INST_SYNC_WRITE 0x83

 // Memory Address
//-------EPROM(Read only)--------
#define SBS_VERSION_L 3
#define SBS_VERSION_H 4

//-------EPROM(Read And write)--------
#define SBS_ID 5
#define SBS_BAUD_RATE 6
#define SBS_MIN_ANGLE_LIMIT_L 9
#define SBS_MIN_ANGLE_LIMIT_H 10
#define SBS_MAX_ANGLE_LIMIT_L 11
#define SBS_MAX_ANGLE_LIMIT_H 12
#define SBS_CW_DEAD 26
#define SBS_CCW_DEAD 27

#define SBS_WORK_MODE 33

//-------SRAM(Read & Write)--------
#define SBS_TORQUE_ENABLE 40
#define SBS_GOAL_POSITION_L 42
#define SBS_GOAL_POSITION_H 43
#define SBS_GOAL_TIME_L 44
#define SBS_GOAL_TIME_H 45
#define SBS_GOAL_SPEED_L 46
#define SBS_GOAL_SPEED_H 47
#define SBS_LOCK 48

//-------SRAM(Read Only)--------
#define SBS_PRESENT_POSITION_L 56
#define SBS_PRESENT_POSITION_H 57
#define SBS_PRESENT_SPEED_L 58
#define SBS_PRESENT_SPEED_H 59
#define SBS_PRESENT_LOAD_L 60
#define SBS_PRESENT_LOAD_H 61
#define SBS_PRESENT_VOLTAGE 62
#define SBS_PRESENT_TEMPERATURE 63
#define SBS_MOVING 66
#define SBS_PRESENT_CURRENT_L 69
#define SBS_PRESENT_CURRENT_H 70
