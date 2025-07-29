/*
 * Copyright (C) 2023 Alessandro Mancinelli
 *
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * This software is published under the GNU General Public License
 * <http://www.gnu.org/licenses/>.
 */
/**
 * @file "servo_control_code_teensy.ino"
 * @author Alessandro Mancinelli
 * @email alessandro.mancinelli@outlook.com
 */

#include <Arduino.h>
#include "Definitions_STS.h"

// -------------------------- COMMUNICATION DEFINED VARIABLES-----------------------------

#define COMMUNICATION_SERIAL Serial4
#define COMMUNICATION_SERIAL_BAUD 921600
#define COMM_REFRESH_TIME 10000 // ~500 Hz message loop
#define PIN_LED_T4 13

byte START_BYTE_SERIAL_ACT_T4 = 0x9A;
elapsedMicros last_time_write_to_pixhawk = 0;
int ack_comm_TX_pixhawk = 0;

struct serial_act_t4_in myserial_act_t4_in;
volatile struct serial_act_t4_out myserial_act_t4_out;
volatile float extra_data_in[255], extra_data_out[255] __attribute__((aligned));

uint8_t rolling_message_out_id_cnt = 0;

uint16_t serial_act_t4_buf_in_cnt = 0;

uint8_t serial_act_t4_msg_buf_in[2 * sizeof(struct serial_act_t4_in)] = {0};
int serial_act_t4_received_packets = 0;
elapsedMillis old_time_frequency_in = 0, time_no_connection_pixhawk = 0, disarm_initialization_timer = 0;
uint16_t serial_act_t4_message_frequency_in;
int serial_act_t4_missed_packets_in;

// --------------------------- PLATE SYSTEM DEFINED VARIABLES-----------------------------------
float max_plate_dist_mm = 140; // Decides the maximum value that the plate can reach after inizialization
float min_plate_dist_mm = 2;   // Decides the minimum value that the plate can reach after inizialization
float p_gain_mm_to_speed = 9000;
float deg_to_mm_ratio = 3.5e-3; // Ratio between a servo rotation in degrees and the vertical movement of the plate in mm
float max_plate_speed_mm_s = 2.5;
int load_threshold_value = 800;                // Threshold value for initialization of servos.
int missed_packets_threshold = 3000;           // A packet error above this value on ANY servo haults the system
float init_speed_servos = -1.2;                // Initialization speed of servos in mm/s, always negative, we want the plate to go up!
float controller_dist_error_tolerance = 0.001; //[mm] A value below that error will not make any plate motion. Useful to remove jitter.

// Not user definable variables:
int N_loops_S1 = 0;
float zero_mm_S1 = 0;
int8_t system_health_state = 0;
int servo_1_initialized = 0;
float servo_1_estimated_dist_mm = 0;
int16_t servo_1_angle_int_old;
float target_plate_dist_mm = 2;

// --------------------------- SERVOS DEFINED VARIABLES-----------------------------------
int servo_arm_bool = 0;

// The Rx and Tx pins at the Teensy must be tied together and connected via 150 Ohm to the FEETECH data line!
#define BAUDRATE_SERVO 1000000 // Baudrate for the servo communication.
#define SERVO_MAX_COMD 4096.0  // It tells us the step we use to control the servo angle
#define SERVO_COMM_MARGIN 300  //[uS] Margin of time given to the servo for the response [300]
#define TIME_OF_SERVO_TX 10    //[uS] Margin of time needed to avoid TX conflicts when the servos are killed [10]
elapsedMicros last_time_write_read_servo_cnt = 0;
int ack_write_read = 0;

// #define VERBOSE_MESSAGE

// Servo 1:
#define SERVO1_serial Serial5
#define SERVO1_serialEnableOpenDrain Serial5EnableOpenDrain

// PREPARE HARDWARE TO BE IN TRISTATE MODE
IMXRT_LPUART_t *s_pkuart_1 = &IMXRT_LPUART6; // underlying hardware for Serial1
IMXRT_LPUART_t *s_pkuart_2 = &IMXRT_LPUART4; // underlying hardware for Serial2
IMXRT_LPUART_t *s_pkuart_3 = &IMXRT_LPUART2; // underlying hardware for Serial3
IMXRT_LPUART_t *s_pkuart_4 = &IMXRT_LPUART3; // underlying hardware for Serial4
IMXRT_LPUART_t *s_pkuart_5 = &IMXRT_LPUART8; // underlying hardware for Serial5
IMXRT_LPUART_t *s_pkuart_6 = &IMXRT_LPUART1; // underlying hardware for Serial6
IMXRT_LPUART_t *s_pkuart_7 = &IMXRT_LPUART7; // underlying hardware for Serial7

// Define time interrupts for the SERVOs:
IntervalTimer SERVO_COMM_WRITE_READ_TIMER;

int iter_counter_SERVO = 0;
int servo_write_read_lock = 0;
int Servo_1_ID = 1;

volatile int Target_speed_servo_1;
volatile int speed_ack_servo_1_lost;
volatile int feedback_servo_1_lost;

elapsedMicros feedback_1_time_uS_counter = 0;

// ------------------------------ GENERAL VARIABLES---------------------------------------

#define DEBUG_serial Serial
#define DEBUG_serial_baud 115200

elapsedMicros timer_count_servo = 0, timer_count_main = 0, timer_count_esc = 0;

elapsedMicros local_time = 0, local_time_1 = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////////////SETUP///////////////////
void setup(void)
{
  //////////////////SETUP DEBUGGING USB
  DEBUG_serial.begin(DEBUG_serial_baud);

  //////////////////SETUP SERVOS
  InitServos();

  //////////////////SETUP CONNECTION WITH PIXHAWK
  COMMUNICATION_SERIAL.begin(COMMUNICATION_SERIAL_BAUD);

  // LED to monitor the connection with Pixhawk
  pinMode(PIN_LED_T4, OUTPUT);
  analogWriteFrequency(PIN_LED_T4, 500);

  // Start the timer for the Communication and Servos
  SERVO_COMM_WRITE_READ_TIMER.begin(ServosAndCommTic, 10); // Interrupt routine for the tick to servos and communication
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////LOOP///////////////////
void loop(void)
{

  writeReadServos();

  Control_servo_speeds();

  ServoRoutine();

  // Display statistics
  if (timer_count_main > 5000)
  { // 200 Hz loop

    // DebugUpdateTimeFeedbackPackets();
    //  DebugServoLostAckPackets();
     DebugServoPositionSpeedLoad();
    //  DebugConnection();
    // DebugServoEstimatedDist();

    timer_count_main = 0;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////USER DEFINED FCN///////////////////

// Generate reference for the servos speed:
void Control_servo_speeds(void)
{

  
  Target_speed_servo_1 = 0;
  }

// Platform communication functions removed - single servo operation only

// Servo routine to be run inside the main loop
void ServoRoutine(void)
{

  if (timer_count_servo >= 2000)
  { // Run at ~500 Hz
    timer_count_servo = 0;
  }
}

// Debug the estimated distance for single servo
void DebugServoEstimatedDist(void)
{
  DEBUG_serial.print("\n  target_plate_dist_mm = ");
  DEBUG_serial.print(target_plate_dist_mm);
  DEBUG_serial.print(" servo_1_estimated_dist_mm:");
  DEBUG_serial.print(servo_1_estimated_dist_mm);
  DEBUG_serial.print("; plate_dist_mm = ");
  DEBUG_serial.print(myserial_act_t4_out.plate_dist_mm);
  DEBUG_serial.print("; plate_speed_mm_s = ");
  DEBUG_serial.print(myserial_act_t4_out.plate_speed_mm_s);
  DEBUG_serial.print("; Plate load = ");
  DEBUG_serial.print(myserial_act_t4_out.plate_load);
  DEBUG_serial.print(", system_health_state:");
  DEBUG_serial.println(system_health_state);
}

// Debug servo position, speed and load for single servo
void DebugServoPositionSpeedLoad(void)
{
  DEBUG_serial.print("\n Servo_1_position_deg:");
  DEBUG_serial.print(myserial_act_t4_out.servo_1_angle_int);
  DEBUG_serial.print("\n Servo_1_speed:");
  DEBUG_serial.print(myserial_act_t4_out.servo_1_speed_int);
  DEBUG_serial.print("\n Servo_1_load:");
  DEBUG_serial.println(myserial_act_t4_out.servo_1_load_int);
}

// Debug lost acknowledgment packets for single servo
void DebugServoLostAckPackets(void)
{
  DEBUG_serial.print("Ack_S1_lost_packets:");
  DEBUG_serial.println(speed_ack_servo_1_lost);
}

// Displays lost position feedback packets for single servo
void DebugServoLostPosPackets(void)
{
  DEBUG_serial.print("Feedback_S1_lost_packets:");
  DEBUG_serial.println(feedback_servo_1_lost);
}

// Displays the refresh rate for single servo
void DebugUpdateTimeFeedbackPackets(void)
{
  DEBUG_serial.print("\n Feedback_S1_time_update:");
  DEBUG_serial.println(myserial_act_t4_out.servo_1_feedback_update_time_us);
}

// Platform connection debug removed - single servo operation only

// Initialize single servo serial including the tristate mode
void InitServos(void)
{

  SERVO1_serial.begin(BAUDRATE_SERVO); // Servo 1
  SERVO1_serialEnableOpenDrain(true);

  // Serial 1 half duplex mode:
  s_pkuart_1->CTRL |= LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC;

  // Serial 2 half duplex mode:
  s_pkuart_2->CTRL |= LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC;

  // Serial 3 half duplex mode:
  s_pkuart_3->CTRL |= LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC;

  // Serial 4 half duplex mode:
  s_pkuart_4->CTRL |= LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC;

  // Serial 5 half duplex mode:
  s_pkuart_5->CTRL |= LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC;

  // Serial 6 half duplex mode:
  // s_pkuart_6->CTRL |= LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC;

  // Serial 7 half duplex mode:
  //  s_pkuart_7->CTRL |= LPUART_CTRL_LOOPS | LPUART_CTRL_RSRC;
}

// Send instructions to servo 1
void SendInstructionServo1(byte u8_ServoID, byte u8_Instruction, byte *u8_Params, int s32_ParamCount)
{
  // -------- SEND INSTRUCTION ------------
  int buffer_tx_idx = 0;
  byte buffer_tx[50];
  buffer_tx[buffer_tx_idx++] = 0xFF;
  buffer_tx[buffer_tx_idx++] = 0xFF;
  buffer_tx[buffer_tx_idx++] = u8_ServoID;
  buffer_tx[buffer_tx_idx++] = s32_ParamCount + 2;
  buffer_tx[buffer_tx_idx++] = u8_Instruction;

  for (int i = 0; i < s32_ParamCount; i++)
  {
    buffer_tx[buffer_tx_idx++] = u8_Params[i];
  }

  byte u8_Checksum = 0;
  for (int i = 2; i < buffer_tx_idx; i++)
  {
    u8_Checksum += buffer_tx[i];
  }
  buffer_tx[buffer_tx_idx++] = ~u8_Checksum;

  // Send the instruction to servo
  SERVO1_serial.clear();
  SERVO1_serialEnableOpenDrain(false);
  SERVO1_serial.write(buffer_tx, buffer_tx_idx);
  SERVO1_serialEnableOpenDrain(true);
#ifdef VERBOSE_MESSAGE
  PrintHex("Instruction sent servo 1: ", buffer_tx, buffer_tx_idx);
#endif
}

// Send instruction functions for servos 2, 3, 4 removed - single servo operation only

// Print a desired buffer in HEX mode
void PrintHex(const char *s8_Text, const byte *data, int count)
{
  DEBUG_serial.print(s8_Text);

  for (int i = 0; i < count; i++)
  {
    if (i > 0)
      DEBUG_serial.print(" ");

    if (data[i] <= 0xF)
      DEBUG_serial.print("0");

    DEBUG_serial.print(data[i], DEC);
  }
  DEBUG_serial.println();
}

// 1 16-bit split into 2 8 digits
void SplitByte(uint8_t *DataL, uint8_t *DataH, uint16_t Data)
{
  *DataH = (Data >> 8);
  *DataL = (Data & 0xff);
}

// 2 8-digit combinations for 1 16-digit number
uint16_t CompactBytes(uint8_t DataL, uint8_t DataH)
{
  uint16_t Data;
  Data = DataL;
  Data <<= 8;
  Data |= DataH;
  return Data;
}

// Keep track of the tick to Servo communication
void ServosAndCommTic(void)
{
  // Servo tic
  if (ack_write_read && last_time_write_read_servo_cnt >= SERVO_COMM_MARGIN)
  {
    iter_counter_SERVO++;
    ack_write_read = 0;
    if (iter_counter_SERVO > 4)
    { // Reduced from 16 to 4 for single servo
      iter_counter_SERVO = 1;
    }
  }
}

// Interrupt routine to read and write to single servo
void writeReadServos(void)
{

  byte buffer_servo[50] = {0};
  int buffer_servo_idx = 0;
  byte u8_Data_1[7] = {0}; // Send position frame
  byte u8_Data_2[2] = {0}; // request data frame

  if (ack_write_read == 0 && servo_write_read_lock == 0)
  {

    if (iter_counter_SERVO == 1)
    { // Send speed target to servo 1 or wait in case of servo KILL
      if (servo_arm_bool)
      {
        volatile int sign = 0;
        if (Target_speed_servo_1 < 0)
        {
          sign = 1;
          Target_speed_servo_1 *= -1;
        }
        u8_Data_1[0] = SBS_GOAL_POSITION_L;
        SplitByte(&u8_Data_1[1], &u8_Data_1[2], 0);
        SplitByte(&u8_Data_1[3], &u8_Data_1[4], 0);
        SplitByte(&u8_Data_1[5], &u8_Data_1[6], Target_speed_servo_1);
        if (sign)
        {
          bitSet(u8_Data_1[6], 7);
        }
        SendInstructionServo1(Servo_1_ID, INST_WRITE, u8_Data_1, sizeof(u8_Data_1)); // Servo 1
      }
      else
      {
        /* disable torque on servo */
        delayMicroseconds(TIME_OF_SERVO_TX);
        u8_Data_2[0] = SBS_TORQUE_ENABLE;
        u8_Data_2[1] = 0;
        SendInstructionServo1(Servo_1_ID, INST_WRITE, u8_Data_2, sizeof(u8_Data_2));
      }
    }

    if (iter_counter_SERVO == 2)
    { // Receive Ack from servo 1
      // Collect Ack response from servo 1
      while (SERVO1_serial.available())
      {
        buffer_servo[buffer_servo_idx] = SERVO1_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for (int i = 2; i < 5; i++)
      { // Calculate checksum
        bitsum_servo += buffer_servo[i];
      }
#ifdef VERBOSE_MESSAGE
      PrintHex("Instruction received from servo 1: ", &buffer_servo[0], 6);
#endif
      if ((255 - bitsum_servo == buffer_servo[5] && buffer_servo[2] == Servo_1_ID) == 0)
      { // If checksum is incorrect, increase counter.
        speed_ack_servo_1_lost++;
      }
      SERVO1_serial.flush();

      // Request to enable motor mode to servo 1
      u8_Data_2[0] = SBS_WORK_MODE;
      u8_Data_2[1] = 1;
      SendInstructionServo1(Servo_1_ID, INST_WRITE, u8_Data_2, sizeof(u8_Data_2));
    }

    if (iter_counter_SERVO == 3)
    { // Request feedback from servo 1
      u8_Data_2[0] = SBS_PRESENT_POSITION_L;
      u8_Data_2[1] = 8; // We want position, speed, load, voltage and temperature out!
      SendInstructionServo1(Servo_1_ID, INST_READ, u8_Data_2, sizeof(u8_Data_2));
    }

    if (iter_counter_SERVO == 4)
    { // Receive feedback from servo 1
      while (SERVO1_serial.available())
      {
        buffer_servo[buffer_servo_idx] = SERVO1_serial.read();
        buffer_servo_idx++;
      }
      uint8_t bitsum_servo = 0;
      for (int i = 2; i < 13; i++)
      { // Sum the received bits
        bitsum_servo += buffer_servo[i];
      }
#ifdef VERBOSE_MESSAGE
      PrintHex("Instruction received from servo 1: ", &buffer_servo[0], buffer_servo_idx);
#endif
      myserial_act_t4_out.servo_1_feedback_update_time_us = (int16_t)feedback_1_time_uS_counter;
      if (255 - bitsum_servo == buffer_servo[13] && buffer_servo[2] == Servo_1_ID)
      {

        // Position
        myserial_act_t4_out.servo_1_angle_int = (int16_t)((CompactBytes(buffer_servo[6], buffer_servo[5]) / SERVO_MAX_COMD) * 360.0);
        // myserial_act_t4_out.servo_1_angle_int = (int16_t)((CompactBytes(buffer_servo[6], buffer_servo[5])));


        // Speed
        int sign = bitRead(buffer_servo[8], 7);
        bitClear(buffer_servo[8], 7);
        myserial_act_t4_out.servo_1_speed_int = (int16_t)((CompactBytes(buffer_servo[8], buffer_servo[7])));
        if (sign)
        {
          myserial_act_t4_out.servo_1_speed_int *= -1;
        }

        // Load
        sign = bitRead(buffer_servo[10], 2);
        bitClear(buffer_servo[10], 2);
        myserial_act_t4_out.servo_1_load_int = (int16_t)((CompactBytes(buffer_servo[10], buffer_servo[9])));
        if (sign)
        {
          myserial_act_t4_out.servo_1_load_int *= -1;
        }

        // Voltage
        myserial_act_t4_out.servo_1_volt_int = (int16_t)(buffer_servo[11]);

        // Temperature
        myserial_act_t4_out.servo_1_temp_int = (int16_t)(buffer_servo[12]);
        feedback_1_time_uS_counter = 0;
      }
      else
      {
        feedback_servo_1_lost++;
      }
      SERVO1_serial.flush();
    }

    // Say to the tick that it can now increase the counter reporting also the time information
    ack_write_read = 1;
    last_time_write_read_servo_cnt = 0;
  }
}

// Active or deactivate Serial 1 tristate mode for half duplex com.
void Serial1EnableOpenDrain(bool bEnable)
{
  if (bEnable)
  {
    Serial1.flush();                        // Make sure we output everything first before changing state.
    s_pkuart_1->CTRL &= ~LPUART_CTRL_TXDIR; // Set in to RX Mode...
  }
  else
  {
    s_pkuart_1->CTRL |= LPUART_CTRL_TXDIR; // Set in to TX Mode...
  }
}

// Active or deactivate Serial 2 tristate mode for half duplex com.
void Serial2EnableOpenDrain(bool bEnable)
{
  if (bEnable)
  {
    Serial2.flush();                        // Make sure we output everything first before changing state.
    s_pkuart_2->CTRL &= ~LPUART_CTRL_TXDIR; // Set in to RX Mode...
  }
  else
  {
    s_pkuart_2->CTRL |= LPUART_CTRL_TXDIR; // Set in to TX Mode...
  }
}

// Active or deactivate Serial 3 tristate mode for half duplex com.
void Serial3EnableOpenDrain(bool bEnable)
{
  if (bEnable)
  {
    Serial3.flush();                        // Make sure we output everything first before changing state.
    s_pkuart_3->CTRL &= ~LPUART_CTRL_TXDIR; // Set in to RX Mode...
  }
  else
  {
    s_pkuart_3->CTRL |= LPUART_CTRL_TXDIR; // Set in to TX Mode...
  }
}

// Active or deactivate Serial 4 tristate mode for half duplex com.
void Serial4EnableOpenDrain(bool bEnable)
{
  if (bEnable)
  {
    Serial4.flush();                        // Make sure we output everything first before changing state.
    s_pkuart_4->CTRL &= ~LPUART_CTRL_TXDIR; // Set in to RX Mode...
  }
  else
  {
    s_pkuart_4->CTRL |= LPUART_CTRL_TXDIR; // Set in to TX Mode...
  }
}

// Active or deactivate Serial 5 tristate mode for half duplex com.
void Serial5EnableOpenDrain(bool bEnable)
{
  if (bEnable)
  {
    Serial5.flush();                        // Make sure we output everything first before changing state.
    s_pkuart_5->CTRL &= ~LPUART_CTRL_TXDIR; // Set in to RX Mode...
  }
  else
  {
    s_pkuart_5->CTRL |= LPUART_CTRL_TXDIR; // Set in to TX Mode...
  }
}

// Active or deactivate Serial 6 tristate mode for half duplex com.
void Serial6EnableOpenDrain(bool bEnable)
{
  if (bEnable)
  {
    Serial6.flush();                        // Make sure we output everything first before changing state.
    s_pkuart_6->CTRL &= ~LPUART_CTRL_TXDIR; // Set in to RX Mode...
  }
  else
  {
    s_pkuart_6->CTRL |= LPUART_CTRL_TXDIR; // Set in to TX Mode...
  }
}

// Active or deactivate Serial 7 tristate mode for half duplex com.
void Serial7EnableOpenDrain(bool bEnable)
{
  if (bEnable)
  {
    Serial7.flush();                        // Make sure we output everything first before changing state.
    s_pkuart_7->CTRL &= ~LPUART_CTRL_TXDIR; // Set in to RX Mode...
  }
  else
  {
    s_pkuart_7->CTRL |= LPUART_CTRL_TXDIR; // Set in to TX Mode...
  }
}