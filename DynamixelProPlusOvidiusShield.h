 /*
  * DynamixelProPlusOvidiusShield.cpp  - Library for controlling Dynamixels Pro+ of a Metamorphic Manipulator using DunamixelShield-Dynamixel2Arduino ROBOTIS libraries
  * Created by N.A. Stravopodis, December, 2020.
  */

#ifndef DynamixelProPlusOvidiusShield_h
#define DynamixelProPlusOvidiusShield_h

#include "Arduino.h"
#include <DynamixelShield.h>
#include <Dynamixel2Arduino.h>
#include <Array.h>

using namespace std;
using namespace ControlTableItem;

extern uint8_t dxl_id[];
const int DXL_ID_SIZE = 3;                            // Must be configured based on declaration in test_metamorphic_manipulator.ino
const uint16_t user_pkt_buf_cap = 128;

extern int dxl_comm_result;                                                               // Communication result
extern bool dxl_addparam_result;                                                          // addParam result
extern bool dxl_getdata_result;                                                           // GetParam result
//extern bool return_function_state;
extern uint8_t dxl_error; 
extern uint16_t dxl_model_number[];
extern int32_t dxl_present_position[];
extern int32_t dxl_goal_position[];
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
} __attribute__((packed)) sw_data_t_gp;

typedef struct sw_data_pv{
  int32_t profile_velocity;
} __attribute__((packed)) sw_data_t_pv;

typedef struct sw_data_pa{
  int32_t profile_acceleration;
} __attribute__((packed)) sw_data_t_pa;

typedef struct sr_data_pp{
  int32_t present_position;
} __attribute__((packed)) sr_data_t_pp;

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

    bool syncSetDynamixelsGoalPosition(uint8_t *DxlIDs, int DxlIds_size, int32_t *Desired_Goal_Position, sw_data_t_gp *SW_Data_Array,int *error_code, Dynamixel2Arduino dxl);

    bool syncSetDynamixelsProfVel(uint8_t *DxlIDs, int DxlIds_size, int32_t *Desired_PV, sw_data_t_pv *SW_Data_Array,int *error_code, Dynamixel2Arduino dxl);

    bool syncSetDynamixelsProfAccel(uint8_t *DxlIDs, int DxlIds_size, int32_t *Desired_PA, sw_data_t_pa *SW_Data_Array,int *error_code, Dynamixel2Arduino dxl);

    // AUXILIARY FUNCTIONS
    bool check_If_OK_for_Errors(int *error_code, Dynamixel2Arduino dxl);

    int32_t convertRadian2DxlPulses(double position_in_radians);

    double convertDxlPulses2Radian(int32_t position_in_dxl_pulses);

    unsigned long calculateDxlExecTime(int32_t PV, int32_t PA, int32_t Pos_i, int32_t Pos_f);

    //bool calculateProfAccel_preassignedVelTexec(int32_t PV, int32_t & PA, double Texec, double * rel_dpos_dxl, double & max_rel_dpos, int32_t & max_rel_dpos_pulses, int * error_code);
    bool calculateProfVelAccel_preassignedVelTexec(int32_t * PV, int32_t * PA, double * rel_dpos_dxl, double Ta, double Texec, int * error_code);

    bool calculateProfVelAccel_preassignedVelTexec2(int32_t * PV, int32_t * PA, double * rel_dpos_dxl, double Ta, double Texec, int * error_code);

    double convertDxlVelUnits2RadPsec(int32_t dxlVunit);

    int32_t convertRadPsec2DxlVelUnits(double dxlVunit_radsec);

    double convertDxlAccelUnits2RadPsec2(int32_t dxlAunit);

    int32_t convertRadPsec2_2_DxlAccelUnits( double dxlAunit_radsec2);


private:


};

 #endif
