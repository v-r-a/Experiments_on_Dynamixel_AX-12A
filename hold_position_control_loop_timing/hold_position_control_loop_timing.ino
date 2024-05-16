/* 
MIT License

Copyright (c) [2024] [Vyankatesh Ashtekar]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. 
*/

// Bioloid 12 DOF
// Hold position sync write and sequential read from 12 motors
// Control loop timing/ frequency test
// Movement data is written syncronously to motors
// edited on 11 Jan 2024 corrected
/*
Note:
It is enough to write the writecontroltable command once
The data is probably held in Atmega8 in motors until a new value is written over
*/

#include <DynamixelShield.h>

#if defined(ARDUINO_AVR_UNO) || defined(ARDUINO_AVR_MEGA2560)
#include <SoftwareSerial.h>
SoftwareSerial soft_serial(7, 8);  // DYNAMIXELShield UART RX/TX
#define DEBUG_SERIAL soft_serial
#elif defined(ARDUINO_SAM_DUE) || defined(ARDUINO_SAM_ZERO)
#define DEBUG_SERIAL SerialUSB
#else
#define DEBUG_SERIAL Serial
#endif
/***********************************************************************************************/
// Macros
#define DXL_LOBYTE(w) ((uint8_t)(((uint16_t)(w)) & 0xff))
#define DXL_HIBYTE(w) ((uint8_t)((((uint16_t)(w)) >> 8) & 0xff))
/***********************************************************************************************/
// Control table address
const uint8_t ADDR_AX_TORQUE_ENABLE = 24;
const uint8_t ADDR_AX_GOAL_POSITION = 30;
const uint8_t ADDR_AX_MOVING_SPEED = 32;
const uint8_t ADDR_AX_PRESENT_POSITION = 36;
const uint8_t ADDR_AX_CW_COMP_MARGIN = 26;
const uint8_t ADDR_AX_CCW_COMP_MARGIN = 27;
const uint8_t ADDR_AX_CW_COMP_SLOPE = 28;
const uint8_t ADDR_AX_CCW_COMP_SLOPE = 29;
const uint8_t ADDR_AX_PRESENT_LOAD = 40;
const uint8_t ADDR_AX_LOCK_EEPROM = 47;
const uint8_t ADDR_AX_RDT = 5;

// Data Byte Length
const uint8_t LEN_AX_GOAL_POSITION = 2;
const uint8_t LEN_AX_MOVING_SPEED = 2;
const uint8_t LEN_AX_PRESENT_POSITION = 2;
const uint8_t LEN_AX_CW_COMP_MARGIN = 1;
const uint8_t LEN_AX_CCW_COMP_MARGIN = 1;
const uint8_t LEN_AX_CW_COMP_SLOPE = 1;
const uint8_t LEN_AX_CCW_COMP_SLOPE = 1;
const uint8_t LEN_AX_PRESENT_LOAD = 2;
const uint8_t LEN_AX_LOCK_EEPROM = 1;
const uint8_t LEN_AX_RDT = 1;

const uint8_t TORQUE_ENABLE = 1;
const uint8_t TORQUE_DISABLE = 0;
const uint8_t LOCK_EEPROM = 1;

// Default pose
const uint16_t HY_MIN = 620;
const uint16_t HY_MAX = 720;
const uint16_t HY_NOM = 666;

const uint16_t HR_MIN = 472;
const uint16_t HR_MAX = 552;
const uint16_t HR_NOM = 512;

const uint16_t HP_MIN = 490;
const uint16_t HP_MAX = 780;
const uint16_t HP_NOM = 550;

const uint16_t KP_MIN = 560;
const uint16_t KP_MAX = 885;
const uint16_t KP_NOM = 590;

const uint16_t AP_MIN = 300;
const uint16_t AP_MAX = 500;
const uint16_t AP_NOM = 460;

const uint16_t AR_MIN = 472;
const uint16_t AR_MAX = 552;
const uint16_t AR_NOM = 512;

const uint8_t NMOTOR = 12;
const uint8_t MOTOR_ID_LIST[NMOTOR] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };

// Homing position and return to home velocity
uint16_t home_pos_list[NMOTOR] = { HY_NOM, HR_NOM, HP_NOM, KP_NOM, AP_NOM, AR_NOM, HY_NOM, HR_NOM, HP_NOM, KP_NOM, AP_NOM, AR_NOM };
uint16_t home_vel_list[NMOTOR] = { 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50 };

// Other positions
uint16_t sls_int_list[NMOTOR] = { HY_NOM, 500, HP_NOM, KP_NOM, AP_NOM, 455, HY_NOM, 500, HP_NOM, KP_NOM, AP_NOM, 455 };
uint16_t sls_int2_list[NMOTOR] = { HY_NOM, 500, HP_NOM, KP_NOM, AP_NOM, 455, HY_NOM, 500, HP_NOM, KP_NOM, AP_NOM - 70, 455 };
uint16_t sls_leftleg_list[NMOTOR] = { HY_NOM, 535, HP_NOM, KP_NOM, AP_NOM, 455 - 60, HY_NOM, HR_NOM, HP_NOM, KP_NOM + 100, AP_NOM, AR_NOM };

/***********************************************************************************************/
// Protocol version
const float PROTOCOL_VERSION = 1.0;  // See which protocol version is used in the Dynamixel
// Starting address of the Data to write; Goal Position = 30
const uint16_t SW_START_ADDR = ADDR_AX_GOAL_POSITION;
// Length of the Data to write: 4 bytes: Goal Position + Goal Velocity
const uint16_t SW_ADDR_LEN = 4;
// Four columns and NMOTOR rows
volatile uint8_t myswdata[NMOTOR][SW_ADDR_LEN];
/***********************************************************************************************/
/*
  typedef struct InfoSyncWriteInst{
    uint16_t addr;
    uint16_t addr_length;
    XELInfoSyncWrite_t* p_xels;
    uint8_t xel_count;
    bool is_info_changed;
    InfoSyncBulkBuffer_t packet;
  } __attribute__((packed)) InfoSyncWriteInst_t;
*/
DYNAMIXEL::InfoSyncWriteInst_t allmotorSW;

/* syncWrite
  Structures containing the necessary information to process the 'syncWrite' packet.

  typedef struct XELInfoSyncWrite{
    uint8_t* p_data;
    uint8_t id;
  } __attribute__((packed)) XELInfoSyncWrite_t;
*/
DYNAMIXEL::XELInfoSyncWrite_t all_info_xels_sw[NMOTOR];
// Create an instance of Master class/ Dynamixel2Arduino class/ whatever has all the required functions
DynamixelShield dxl;
//This namespace is required to use Control table item names
using namespace ControlTableItem;
/***********************************************************************************************/
unsigned long myTimeStart, myTimeEnd;
int debug_motor_pose;
uint8_t present_pos[16];
bool sync_write_feedback, write_feedback;
int inputpos;
int32_t read_init;
/***********************************************************************************************/
void setup() {
  // put your setup code here, to run once:

  // Communication with PC starts---------------------------->>
  DEBUG_SERIAL.begin(115200);

  // Communication starts------------------------------------>>
  dxl.begin(1000000);  // dxl baudrate
  dxl.setPortProtocolVersion(1.0);
  uint8_t motor_id;
  // Write only once control table items
  for (int i = 0; i < NMOTOR; i++) {
    motor_id = MOTOR_ID_LIST[i];
    DEBUG_SERIAL.print("MOTOR ID: ");
    DEBUG_SERIAL.println(motor_id);
    // Redefine the packet return delay time (1 = 2us)
    write_feedback = dxl.writeControlTableItem(ControlTableItem::RETURN_DELAY_TIME, motor_id, 0);
    DEBUG_SERIAL.print("Return delay time set (1 for success): ");
    DEBUG_SERIAL.println(write_feedback);
    delay(10);
    read_init = dxl.readControlTableItem(ControlTableItem::RETURN_DELAY_TIME, motor_id);
    DEBUG_SERIAL.print("Return delay time is: ");
    DEBUG_SERIAL.println(read_init);
    // Lock the EEPROM
    // write_feedback = dxl.writeControlTableItem(LOCK, motor_id, LOCK_EEPROM);
    // DEBUG_SERIAL.print("EEPROM locked (1 for success): ");
    // DEBUG_SERIAL.println(write_feedback);
    delay(10);

    // Set compliance margin and compliance slope
    write_feedback = dxl.writeControlTableItem(ControlTableItem::CW_COMPLIANCE_MARGIN, motor_id, 2);
    DEBUG_SERIAL.print("CW Compliance Margin set (1 for success): ");
    DEBUG_SERIAL.println(write_feedback);
    delay(10);
    read_init = dxl.readControlTableItem(ControlTableItem::CW_COMPLIANCE_MARGIN, motor_id);
    DEBUG_SERIAL.print("CW_Compliance margin is:");
    DEBUG_SERIAL.println(read_init);

    write_feedback = dxl.writeControlTableItem(ControlTableItem::CCW_COMPLIANCE_MARGIN, motor_id, 2);
    DEBUG_SERIAL.print("CCW Compliance Margin set (1 for success): ");
    DEBUG_SERIAL.println(write_feedback);
    delay(10);
    read_init = dxl.readControlTableItem(ControlTableItem::CCW_COMPLIANCE_MARGIN, motor_id);
    DEBUG_SERIAL.print("CCW_Compliance margin is:");
    DEBUG_SERIAL.println(read_init);

    write_feedback = dxl.writeControlTableItem(ControlTableItem::CW_COMPLIANCE_SLOPE, motor_id, 16);
    DEBUG_SERIAL.print("CW Compliance Slope set (1 for success): ");
    DEBUG_SERIAL.println(write_feedback);
    delay(10);
    read_init = dxl.readControlTableItem(ControlTableItem::CW_COMPLIANCE_SLOPE, motor_id);
    DEBUG_SERIAL.print("CW_Compliance slope is:");
    DEBUG_SERIAL.println(read_init);

    write_feedback = dxl.writeControlTableItem(ControlTableItem::CCW_COMPLIANCE_SLOPE, motor_id, 16);
    DEBUG_SERIAL.print("CCW Compliance Slope set (1 for success): ");
    DEBUG_SERIAL.println(write_feedback);
    delay(10);
    read_init = dxl.readControlTableItem(ControlTableItem::CCW_COMPLIANCE_SLOPE, motor_id);
    DEBUG_SERIAL.print("CCW_Compliance slope is:");
    DEBUG_SERIAL.println(read_init);

    // Enable Torques one by one
    write_feedback = dxl.torqueOn(motor_id);
    DEBUG_SERIAL.print("Torque ON (1 for success): ");
    DEBUG_SERIAL.println(write_feedback);
    delay(10);
  }

  /***********************************************************************************************/
  // Fill the members of structure to syncWrite using internal packet buffer
  // (This part of code is equivalent to 'dummy syncwrite to start off' in c++ Dynamixel SDK.)
  // All motors sync write
  allmotorSW.packet.p_buf = nullptr;
  allmotorSW.packet.is_completed = false;
  allmotorSW.addr = SW_START_ADDR;
  allmotorSW.addr_length = SW_ADDR_LEN;
  allmotorSW.p_xels = all_info_xels_sw;
  allmotorSW.xel_count = 0;

  // AddParam equivalent: Go to the home pose
  for (int i = 0; i < NMOTOR; i++) {
    myswdata[i][0] = DXL_LOBYTE(home_pos_list[i]);  // low byte
    myswdata[i][1] = DXL_HIBYTE(home_pos_list[i]);  // high byte
    myswdata[i][2] = DXL_LOBYTE(home_vel_list[i]);  // low byte
    myswdata[i][3] = DXL_HIBYTE(home_vel_list[i]);  // high byte
    all_info_xels_sw[i].id = MOTOR_ID_LIST[i];
    all_info_xels_sw[i].p_data = myswdata[i];
    allmotorSW.xel_count++;
  }
  allmotorSW.is_info_changed = true;
  dxl.syncWrite(&allmotorSW);
  // Allow some time for going to home position
  delay(1000);
}
/***********************************************************************************************/
void loop() {
  // put your main code here, to run repeatedly:
  // Control loop timing
  myTimeStart = micros();
  allmotorSW.xel_count = 0;
  for (int i = 0; i < NMOTOR; i++) {
    myswdata[i][0] = DXL_LOBYTE(home_pos_list[i]);  // low byte
    myswdata[i][1] = DXL_HIBYTE(home_pos_list[i]);  // high byte
    myswdata[i][2] = DXL_LOBYTE(home_vel_list[i]);  // low byte
    myswdata[i][3] = DXL_HIBYTE(home_vel_list[i]);  // high byte
    all_info_xels_sw[i].id = MOTOR_ID_LIST[i];
    all_info_xels_sw[i].p_data = myswdata[i];
    allmotorSW.xel_count++;
  }
  allmotorSW.is_info_changed = true;
  if (dxl.syncWrite(&allmotorSW)) {
    // syncWrite returns 1 on success.
  } else {
    DEBUG_SERIAL.print("[SyncWrite] Fail, Lib error code: ");
    DEBUG_SERIAL.println(dxl.getLastLibErrCode());
  }
  
  // Debugging: Read motor positions
  for (int i = 0; i < NMOTOR; i++) {
    // debug_motor_pose = dxl.readControlTableItem(PRESENT_POSITION, MOTOR_ID_LIST[i]);
    debug_motor_pose= dxl.read(MOTOR_ID_LIST[i], ADDR_AX_PRESENT_POSITION, 4, present_pos, 16);

    int position = *((int*)present_pos);
    int speed = *((int*)(present_pos + 2));

    // Print the data for debugging
    // DEBUG_SERIAL.print("Motor ID: ");
    // DEBUG_SERIAL.println(MOTOR_ID_LIST[i]);
    // DEBUG_SERIAL.print("Present Position: ");
    // DEBUG_SERIAL.println(position);
    // DEBUG_SERIAL.print("Present Speed: ");
    // DEBUG_SERIAL.println(speed);
    // DEBUG_SERIAL.println(); // Print a blank line for readability
  }

  DEBUG_SERIAL.print("One loop time: ");
  myTimeEnd = micros();
  DEBUG_SERIAL.println(myTimeEnd - myTimeStart);
}


/***********************************************************************************************/
