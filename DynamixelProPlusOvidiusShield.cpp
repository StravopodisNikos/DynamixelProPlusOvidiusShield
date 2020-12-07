 /*
  * DynamixelProPlusOvidiusShield.cpp  - Library for controlling Dynamixels Pro+ of a Metamorphic Manipulator using DunamixelShield-Dynamixel2Arduino ROBOTIS libraries
  * Created by N.A. Stravopodis, December, 2020.
  */

#include "Arduino.h"
#include <DynamixelShield.h>
#include <Dynamixel2Arduino.h>
#include "DynamixelProPlusOvidiusShield.h"

// Include Motor Configuration files from folder ~/Arduino/libraries/
#include <definitions.h>                            
#include <motorIDs.h>                               
#include <contolTableItems_LimitValues.h>
#include <StepperMotorSettings.h>

using namespace std;
using namespace ControlTableItem;

DYNAMIXEL::InfoSyncWriteInst_t sw_gp_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw_gp[DXL_ID_SIZE];

// Constructor
DynamixelProPlusOvidiusShield::DynamixelProPlusOvidiusShield(uint8_t *DxlIDs){

    uint8_t _DXL1_ID = DxlIDs[0];
    uint8_t _DXL2_ID = DxlIDs[1];
    uint8_t _DXL3_ID = DxlIDs[2];
}

// =========================================================================================================== //
bool DynamixelProPlusOvidiusShield::setDynamixelsTorqueON(uint8_t *DxlIDs, int DxlIds_size, Dynamixel2Arduino dxl)
{
    for(int dxl_cnt = 0; dxl_cnt < DxlIds_size; dxl_cnt++)
    {
        dxl.torqueOn(DxlIDs[dxl_cnt]);
    }

    return true;
}

bool DynamixelProPlusOvidiusShield::setDynamixelsTorqueOFF(uint8_t *DxlIDs, int DxlIds_size, Dynamixel2Arduino dxl)
{
    for(int dxl_cnt = 0; dxl_cnt < DxlIds_size; dxl_cnt++)
    {
        dxl.torqueOff(DxlIDs[dxl_cnt]);
    }

    return true;
}
// =========================================================================================================== //

bool DynamixelProPlusOvidiusShield::setDynamixelLeds(uint8_t *DxlIDs, int DxlIds_size, unsigned char *led_indicator, Dynamixel2Arduino dxl)
{
 /*
  *  Sets value to LED of Dynamixels given the ID numbers and the desired color(as array 3 elements)
  */
    bool command_execution_success;
    /*
    // 1. turns off led
    for(int dxl_cnt = 0; dxl_cnt < DxlIds_size; dxl_cnt++) {
            dxl.writeControlTableItem(LED_RED, DxlIDs[dxl_cnt], 0); // write RED VALUE to DXL Control Table
            dxl.writeControlTableItem(LED_GREEN, DxlIDs[dxl_cnt], 0); // write GREEN VALUE to DXL Control Table
            dxl.writeControlTableItem(LED_BLUE, DxlIDs[dxl_cnt], 0); // write BLUE VALUE to DXL Control Table
            unsigned long time_now_millis = millis(); while(millis() < time_now_millis + 500){} // waits 500 milliseconds
    }*/
    // 1. sets specified color value
    for(int dxl_cnt = 0; dxl_cnt < DxlIds_size; dxl_cnt++) {
            dxl.writeControlTableItem(LED_RED, DxlIDs[dxl_cnt], led_indicator[0]); // write RED VALUE to DXL Control Table
            dxl.writeControlTableItem(LED_GREEN, DxlIDs[dxl_cnt], led_indicator[1]); // write GREEN VALUE to DXL Control Table
            dxl.writeControlTableItem(LED_BLUE, DxlIDs[dxl_cnt], led_indicator[2]); // write BLUE VALUE to DXL Control Table
            
    }
    //unsigned long time_now_millis = millis(); while(millis() < time_now_millis + 500){} // waits 500 milliseconds

return true;
}
// =========================================================================================================== //


bool DynamixelProPlusOvidiusShield::blinkDynamixelLeds(uint8_t *DxlIDs, int DxlIds_size, unsigned char *led_indicator, unsigned long interval, int times, Dynamixel2Arduino dxl)
{
 /*
  *  Blinks Dynamixels given the ID numbers and the desired color(as array 3 elements) and time interval/times of blink
  *  No timer interrupt used here
  */
    bool command_execution_success;
    unsigned long time_now_millis;
    unsigned char turn_off_led[] = {0, 0, 0};

    for(int times_cnt = 0; times_cnt < times; times_cnt++) {
        // leds off
        DynamixelProPlusOvidiusShield::setDynamixelLeds(DxlIDs, DxlIds_size, turn_off_led, dxl);
        // delay
        time_now_millis = millis(); while(millis() < time_now_millis + interval){} // waits interval milliseconds
        // leds on
        DynamixelProPlusOvidiusShield::setDynamixelLeds(DxlIDs, DxlIds_size, led_indicator, dxl);
        // delay
        time_now_millis = millis(); while(millis() < time_now_millis + interval){} // waits interval milliseconds
    }

    return true;
}
// =========================================================================================================== //

// Ping Dynamixels
bool DynamixelProPlusOvidiusShield::pingDynamixels(uint8_t *DxlIDs, int DxlIds_size,int *error_code, Dynamixel2Arduino dxl) {
/*
 *  Pings Connected Dynamixels given the correct ID number as set using thw Wizard Software
 */
    unsigned char ping_indicator[DXL_ID_SIZE];
    bool dxl_comm_result;
    int dxl_model_number[DXL_ID_SIZE];

    for(int id_count = 0; id_count < DxlIds_size; id_count++){
            dxl_comm_result = dxl.ping(DxlIDs[id_count]);
            if (dxl_comm_result == false)
            {
                //return false;
                delay(500);
            }
            else
            {
                ping_indicator[0] = 0;    //R
                ping_indicator[1] = 255;  //G
                ping_indicator[2] = 0;    //B

                unsigned long ping_interval = 500;
                int ping_times = 1;

                dxl_comm_result = DynamixelProPlusOvidiusShield::blinkDynamixelLeds(DxlIDs, DxlIds_size,ping_indicator, ping_interval, ping_times, dxl);

            }
        
    }

  bool function_state =  DynamixelProPlusOvidiusShield::check_If_OK_for_Errors(*error_code, dxl);
  if (function_state)
  {
     return true;
  }
  else
  {
      return false;
  }
  
}

// =========================================================================================================== //
bool DynamixelProPlusOvidiusShield::syncSetDynamixelsGoalPosition(uint8_t *DxlIDs, int DxlIds_size, int32_t *Desired_Goal_Position, sw_data_t *SW_Data_Array,int *error_code, Dynamixel2Arduino dxl) {
/*
 *  Sends Goal Position to pinged Dynamixels and moves the motor!  Main .ino file must wait depending trajectory execution time!
 */
    // Default for Goal Position
    sw_gp_infos.packet.p_buf = nullptr;
    sw_gp_infos.packet.is_completed = false;
    sw_gp_infos.addr = ADDR_PRO_GOAL_POSITION;
    sw_gp_infos.addr_length = LEN_PRO_GOAL_POSITION;
    sw_gp_infos.p_xels = info_xels_sw_gp;
    sw_gp_infos.xel_count = 0;

    // Give desired values
    for(int id_count = 0; id_count < DxlIds_size; id_count++){
        SW_Data_Array[id_count].goal_position = Desired_Goal_Position[id_count];

        info_xels_sw_gp[id_count].id = DxlIDs[id_count];
        info_xels_sw_gp[id_count].p_data = (uint8_t*)&SW_Data_Array[id_count].goal_position;
        sw_gp_infos.xel_count++;
    }
    sw_gp_infos.is_info_changed = true;

    // Moves motors
    dxl.syncWrite(&sw_gp_infos);

    bool function_state =  DynamixelProPlusOvidiusShield::check_If_OK_for_Errors(*error_code, dxl);
    if (function_state)
    {
        return true;
    }
    else
    {
        return false;
    }
}
// =========================================================================================================== //


// =========================================================================================================== //
//                                              AUXILIARY FUNCTIONS
// =========================================================================================================== //

bool DynamixelProPlusOvidiusShield::check_If_OK_for_Errors(int *error_code, Dynamixel2Arduino dxl)
{
    *error_code = dxl.getLastLibErrCode();

    if ( *error_code == 0)
    {
        return true;
    }
    else
    {
        return false;
    }
    
    
}

// =========================================================================================================== //

uint32_t DynamixelProPlusOvidiusShield::convertRadian2DxlPulses(double position_in_radians)
{
    uint32_t position_in_dxl_pulses;
    //double position_in_radians;

    if (position_in_radians == 0)
    {
        position_in_dxl_pulses = 0;
    }
    else 
    {
        position_in_dxl_pulses = (position_in_radians * DXL_RESOLUTION)/PI;
    }

return position_in_dxl_pulses;
}
  
// =========================================================================================================== //

double DynamixelProPlusOvidiusShield::convertDxlPulses2Radian(uint32_t position_in_dxl_pulses)
{
    double position_in_radians;
    
    if (position_in_dxl_pulses == 0)
    {
        position_in_radians = (double) position_in_dxl_pulses;
    }
    else
    {
        position_in_radians = (double) (position_in_dxl_pulses * PI) / DXL_RESOLUTION ;
    }
    
    return position_in_radians;
}

// =========================================================================================================== //

unsigned long DynamixelProPlusOvidiusShield::calculateDxlExecTime(int32_t PV, int32_t PA, int32_t Pos_i, int32_t Pos_f)
{
    /*
     *  Described in Par. 2.4.34 @ http://emanual.robotis.com/docs/en/dxl/p/ph54-100-s500-r/
     *  All units are [pulses] and time in [ms]
     */

    int32_t Dpos = abs(Pos_f - Pos_i);

    unsigned long t1 = (600*PV)/PA;

    unsigned long t2 = (6000000/(2*DXL_RESOLUTION)) * (Dpos / PV);

    unsigned long t3 = t1 + t2;

    return t3;
}

// =========================================================================================================== //

