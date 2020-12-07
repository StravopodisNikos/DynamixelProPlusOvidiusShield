 /*
  * DynamixelProPlusOvidiusShield.cpp  - Library for controlling Dynamixels Pro+ of a Metamorphic Manipulator using DunamixelShield-Dynamixel2Arduino ROBOTIS libraries
  * Created by N.A. Stravopodis, December, 2020.
  */

#ifndef DynamixelProPlusOvidiusShield_h
#define DynamixelProPlusOvidiusShield_h

#include "Arduino.h"
#include <DynamixelShield.h>
#include <Dynamixel2Arduino.h>

using namespace std;
using namespace ControlTableItem;

extern uint8_t dxl_id[];
const int DXL_ID_SIZE = 3;                            // Must be configured based on declaration in test_metamorphic_manipulator.ino

extern int dxl_comm_result;                                                               // Communication result
extern bool dxl_addparam_result;                                                          // addParam result
extern bool dxl_getdata_result;                                                           // GetParam result
extern bool return_function_state;
extern uint8_t dxl_error; 
extern uint16_t dxl_model_number[];
extern uint32_t dxl_present_position[];
extern uint32_t dxl_goal_position[];
extern uint8_t dxl_ledBLUE_value[];
extern uint8_t dxl_ledGREEN_value[];
extern uint8_t dxl_ledRED_value[];
extern int32_t position_in_dxl_pulses;
extern double position_in_radians;
extern unsigned char ping_indicator[];
extern int32_t dxl_vel_limit[DXL_ID_SIZE];
extern int32_t dxl_accel_limit[DXL_ID_SIZE];
extern int32_t dxl_prof_vel[DXL_ID_SIZE];
extern int32_t dxl_prof_accel[DXL_ID_SIZE];

// Data Packaging
typedef struct sw_data{
  int32_t goal_position;
} __attribute__((packed)) sw_data_t;

// Function Nomenclature

/*
 *      Functions with syncSet{Name}: Use GroupSyncWrite with Direct Addressing of only 1 control table item
 *      Functions with syncSet_{Name 1}_{Name 2}_{Name n}: Use GroupSyncWrite with Indirect Addressing of n control table items
 *      
 *      Functions with syncGet{Name}: Use GroupSyncRead with Direct Addressing of only 1 control table item
 *      Functions with syncGet_{Name 1}_{Name 2}_{Name n}: Use GroupSyncRead with Indirect Addressing of n control table items
 */

class DynamixelProPlusOvidiusShield
{ 
 public:
    int led_change = 0;                         // global value to see led colours changing after each movement completes(just for simple visualization)
    // Primary Functions
    DynamixelProPlusOvidiusShield(uint8_t *DxlIDs);

    bool setDynamixelsTorqueON(uint8_t *DxlIDs, int DxlIds_size, Dynamixel2Arduino dxl);

    bool setDynamixelsTorqueOFF(uint8_t *DxlIDs, int DxlIds_size, Dynamixel2Arduino dxl);

    bool setDynamixelLeds(uint8_t *DxlIDs, int DxlIds_size, unsigned char *led_indicator, Dynamixel2Arduino dxl);

    bool blinkDynamixelLeds(uint8_t *DxlIDs, int DxlIds_size, unsigned char *led_indicator, unsigned long interval, int times, Dynamixel2Arduino dxl);

    bool pingDynamixels(uint8_t *DxlIDs, int DxlIds_size, int *error_code, Dynamixel2Arduino dxl);

    bool syncSetDynamixelsGoalPosition(uint8_t *DxlIDs, int DxlIds_size, int32_t *Desired_Goal_Position, sw_data_t *SW_Data_Array,int *error_code, Dynamixel2Arduino dxl);

    // AUXILIARY FUNCTIONS
    bool check_If_OK_for_Errors(int *error_code, Dynamixel2Arduino dxl);

    uint32_t convertRadian2DxlPulses(double position_in_radians);

    double convertDxlPulses2Radian(uint32_t position_in_dxl_pulses);

    unsigned long calculateDxlExecTime(int32_t PV, int32_t PA, int32_t Pos_i, int32_t Pos_f);

private:


};

 #endif
