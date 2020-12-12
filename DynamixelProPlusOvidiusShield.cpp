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
#include <Array.h>

using namespace std;
using namespace ControlTableItem;

DYNAMIXEL::InfoSyncWriteInst_t sw_gp_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw_gp[DXL_ID_SIZE];

DYNAMIXEL::InfoSyncWriteInst_t sw_pv_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw_pv[DXL_ID_SIZE];

DYNAMIXEL::InfoSyncWriteInst_t sw_pa_infos;
DYNAMIXEL::XELInfoSyncWrite_t info_xels_sw_pa[DXL_ID_SIZE];

const double pi              = 3.14159265359;

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
bool DynamixelProPlusOvidiusShield::syncSetDynamixelsGoalPosition(uint8_t *DxlIDs, int DxlIds_size, int32_t *Desired_Goal_Position, sw_data_t_gp *SW_Data_Array,int *error_code, Dynamixel2Arduino dxl) {
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

bool DynamixelProPlusOvidiusShield::syncSetDynamixelsProfVel(uint8_t *DxlIDs, int DxlIds_size, int32_t *Desired_PV, sw_data_t_pv *SW_Data_Array,int *error_code, Dynamixel2Arduino dxl)
{
/*
 *  Sends ProfileVelocity to pinged Dynamixels!  Main .ino file was given trajectory execution time! Profile Velocity is computed 
 *  form MATLAB(trajectory planning) and profile_acceleration is computed based on velocity profile characteristics
 */
    // Default for Goal Position
    sw_pv_infos.packet.p_buf = nullptr;
    sw_pv_infos.packet.is_completed = false;
    sw_pv_infos.addr = ADDR_PRO_PROF_VEL;
    sw_pv_infos.addr_length = LEN_PRO_PROF_VEL;
    sw_pv_infos.p_xels = info_xels_sw_pv;
    sw_pv_infos.xel_count = 0;

    // Give desired values
    for(int id_count = 0; id_count < DxlIds_size; id_count++){
        SW_Data_Array[id_count].profile_velocity = Desired_PV[id_count];

        info_xels_sw_pv[id_count].id = DxlIDs[id_count];
        info_xels_sw_pv[id_count].p_data = (uint8_t*)&SW_Data_Array[id_count].profile_velocity;
        sw_pv_infos.xel_count++;
    }
    sw_pv_infos.is_info_changed = true;

    // Moves motors
    dxl.syncWrite(&sw_pv_infos);

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

bool DynamixelProPlusOvidiusShield::syncSetDynamixelsProfAccel(uint8_t *DxlIDs, int DxlIds_size, int32_t *Desired_PA, sw_data_t_pa *SW_Data_Array,int *error_code, Dynamixel2Arduino dxl)
{
/*
 *  Sends ProfileAcceleration to pinged Dynamixels!  Main .ino file was given trajectory execution time! Profile Velocity is computed 
 *  form MATLAB(trajectory planning) and profile_acceleration is computed based on velocity profile characteristics
 */
    // Default for Goal Position
    sw_pa_infos.packet.p_buf = nullptr;
    sw_pa_infos.packet.is_completed = false;
    sw_pa_infos.addr = ADDR_PRO_PROF_ACCEL;
    sw_pa_infos.addr_length = LEN_PRO_PROF_ACCEL;
    sw_pa_infos.p_xels = info_xels_sw_pa;
    sw_pa_infos.xel_count = 0;

    // Give desired values
    for(int id_count = 0; id_count < DxlIds_size; id_count++){
        SW_Data_Array[id_count].profile_acceleration = Desired_PA[id_count];

        info_xels_sw_pa[id_count].id = DxlIDs[id_count];
        info_xels_sw_pa[id_count].p_data = (uint8_t*)&SW_Data_Array[id_count].profile_acceleration;
        sw_pa_infos.xel_count++;
    }
    sw_pa_infos.is_info_changed = true;

    // Moves motors
    dxl.syncWrite(&sw_pa_infos);

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

int32_t DynamixelProPlusOvidiusShield::convertRadian2DxlPulses(double position_in_radians)
{
    int32_t position_in_dxl_pulses;
    //double position_in_radians;

    if (position_in_radians == 0)
    {
        position_in_dxl_pulses = 0;
    }
    else 
    {
        position_in_dxl_pulses = (position_in_radians * DXL_RESOLUTION) / pi;
    }

return position_in_dxl_pulses;
}
  
// =========================================================================================================== //

double DynamixelProPlusOvidiusShield::convertDxlPulses2Radian(int32_t position_in_dxl_pulses)
{
    double position_in_radians;
    
    if (position_in_dxl_pulses == 0)
    {
        position_in_radians = (double) position_in_dxl_pulses;
    }
    else
    {
        position_in_radians = (double) (position_in_dxl_pulses * pi) / DXL_RESOLUTION ;
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

//bool DynamixelProPlusOvidiusShield::calculateProfAccel_preassignedVelTexec(int32_t PV, int32_t & PA, double Ta, double * rel_dpos_dxl, double & max_rel_dpos, int32_t & max_rel_dpos_pulses, int * error_code)
bool DynamixelProPlusOvidiusShield::calculateProfVelAccel_preassignedVelTexec(int32_t * PV, int32_t * PA, double * rel_dpos_dxl, double Ta, double Texec, int * error_code)
{
    /*
     *  All units are [pulses] and time in [ms]
     *  Returns final PV(Profile Velocity) and PA(Profile Acceleration) for given from
     *  1. function: syncPreSetStepperGoalPositionVarStep for Stepper: Ta,Texec
     *  2. user: desired Profile Velocity PV (it will change according to sync constraints)
     */
/*
    // take only absolute value
    double abs_rel_dpos_dxl[DXL_MOTORS];

    for (size_t i = 0; i < DXL_MOTORS; i++)
    {
        abs_rel_dpos_dxl[i] = abs(rel_dpos_dxl[i]);
    }
    
    // define the array type
    Array<double> array = Array<double>(abs_rel_dpos_dxl,DXL_MOTORS);

    // convert Txec[sec] to [millis] Texec_millis!
    int32_t Texec_millis = Texec * 1000;

    max_rel_dpos = array.getMax();
    // convert relative angular displacement [rad] to [pulses]
    max_rel_dpos_pulses = DynamixelProPlusOvidiusShield::convertRadian2DxlPulses(max_rel_dpos);

    uint64_t factor1 = 600 * DXL_RESOLUTION * PV * PV;

    uint64_t factor2 = (DXL_RESOLUTION * PV * Texec_millis ) - (6000000 * max_rel_dpos_pulses) ;

    if (factor2 == 0)
    {
        PA = 3992645;                     // exceeds by +1 the maximum accepted value of AccelerationLimit in ControlTableItem -> JUNK VALUE
        (*error_code) = 15;               // custom error code. max error code was 14 for DynamixelShield  
        return false;
    }
    else
    {
        PA =  factor1 / factor2;
        (*error_code) = 0;
        return true;
    }
*/
    // Profile Velocity and Acceleration can only be >=0 !!!
    int32_t abs_PV[DXL_MOTORS];

    for (size_t i = 0; i < DXL_MOTORS; i++)
    {
        abs_PV[i] = abs(PV[i]);
    }
    
    // define the array type
    Array<int32_t> array_pv = Array<int32_t>(abs_PV,DXL_MOTORS);

    // Define Acceleration Profile w.r.t the slowest motor
    int32_t min_PV = array_pv.getMin();
    int min_PV_index = array_pv.getMinIndex();
    
    // convert Ta[sec] to [millis] Ta_millis!
    int32_t Ta_millis = Ta * 1000;
    int32_t min_PA = ( 600 * min_PV) / (  Ta_millis );

    // assign relative values for the rest of the dynamixels! 
    double final_PV_double[DXL_MOTORS];
    double final_PA_double[DXL_MOTORS];

    double min_PA_double;
    double min_PV_double;

    for (size_t i = 0; i < DXL_MOTORS; i++)
    {
        if (i==min_PV_index)
        {
            PA[i] = abs(min_PA);     // change acceleration of this joint
            PV[i] = abs(min_PV);     // keep min value of velocity unchanged
        }
        else
        {
            // calculated based on eq. for (ai,vi) in Melchiorri p.66
            final_PA_double[i] = ( rel_dpos_dxl[i] / ( Ta * (Texec - Ta) ) );       // must be converted to DxlUnits[pulses]
            PA[i] = DynamixelProPlusOvidiusShield::convertRadPsec2_2_DxlAccelUnits( abs(final_PA_double[i]));

            final_PV_double[i] = ( rel_dpos_dxl[i] / (Texec - Ta) );               // must be converted to DxlUnits[pulses]
            PV[i] = DynamixelProPlusOvidiusShield::convertRadPsec2DxlVelUnits(abs(final_PV_double[i]));
        }
        
    }

    return true;
}
// =========================================================================================================== //


//bool DynamixelProPlusOvidiusShield::calculateProfAccel_preassignedVelTexec2(int32_t PV, int32_t & PA, double Ta, double * rel_dpos_dxl, double & max_rel_dpos, int32_t & max_rel_dpos_pulses, int * error_code)
bool DynamixelProPlusOvidiusShield::calculateProfVelAccel_preassignedVelTexec2(int32_t * PV, int32_t * PA, double * rel_dpos_dxl, double Ta, double Texec, int * error_code)
{
    /*
     *  All units are [pulses] and time in [ms]
     *  Returns final PV(Profile Velocity) and PA(Profile Acceleration) for given from
     *  1. function: syncPreSetStepperGoalPositionVarStep for Stepper: Ta,Texec
     *  2. user: desired Profile Velocity PV (it will change according to sync constraints)
     *  The difference with previous function is that calculates PV,PA based on Texec (not only Ta) and largest displacement!!!
     * 
     *  * FOR THE TIME I SET PA UNCHANGED!!!! (edited on 12-12-20 must be revised after Xmas!)
     */

    // take only absolute value
    double abs_rel_dpos_dxl[DXL_MOTORS];

    for (size_t i = 0; i < DXL_MOTORS; i++)
    {
        abs_rel_dpos_dxl[i] = abs(rel_dpos_dxl[i]);
    }
    
    // define the array type
    Array<double> array = Array<double>(abs_rel_dpos_dxl,DXL_MOTORS);

    // convert Txec[sec] to [millis] Texec_millis!
    int32_t Texec_millis = Texec * 1000;

    // specify motor with largest relative displacement
    double max_rel_dpos = array.getMax();
    int max_rel_dpos_index = array.getMaxIndex();

    int32_t max_rel_dpos_PA;
    int32_t max_rel_dpos_PV;

    // convert relative angular displacement [rad] to [pulses]
    int32_t max_rel_dpos_pulses = DynamixelProPlusOvidiusShield::convertRadian2DxlPulses(max_rel_dpos);

    int64_t factor1 = 600 * DXL_RESOLUTION * PV[max_rel_dpos_index] * PV[max_rel_dpos_index];

    int64_t factor2 = (DXL_RESOLUTION * PV[max_rel_dpos_index] * Texec_millis ) - (6000000 * max_rel_dpos_pulses) ;

    if (factor2 == 0)
    {
        max_rel_dpos_PA = (int32_t) 3992645;        // exceeds by +1 the maximum accepted value of AccelerationLimit in ControlTableItem -> JUNK VALUE
        (*error_code) = 15;               // custom error code. max error code was 14 for DynamixelShield  
    }
    else
    {
        max_rel_dpos_PA = (int32_t) factor1 / factor2;
        (*error_code) = 0;
    }

    double final_PV_double[DXL_MOTORS];
    double final_PA_double[DXL_MOTORS];

    for (size_t i = 0; i < DXL_MOTORS; i++)
    {
        if (i==max_rel_dpos_index)
        {
            //PA[i] = abs(max_rel_dpos_PA);     // change acceleration of this joint
            PV[i] = abs(PV[i]);               // keep value of velocity unchanged
        }
        else
        {
            // calculated based on eq. for (ai,vi) in Melchiorri p.66
            //final_PA_double[i] = ( rel_dpos_dxl[i] / ( Ta * (Texec - Ta) ) );       // must be converted to DxlUnits[pulses]
            //PA[i] = DynamixelProPlusOvidiusShield::convertRadPsec2_2_DxlAccelUnits( abs(final_PA_double[i]));

            final_PV_double[i] = ( rel_dpos_dxl[i] / (Texec - Ta) );               // must be converted to DxlUnits[pulses]
            PV[i] = DynamixelProPlusOvidiusShield::convertRadPsec2DxlVelUnits(abs(final_PV_double[i]));
        }
        
    }

    return true;
}
// =========================================================================================================== //

double DynamixelProPlusOvidiusShield::convertDxlVelUnits2RadPsec(int32_t dxlVunit)
{
    // 1 [dxlVunit] -> 0.01 [rev/min] -> 0.00104719755 [rad/sec]

    double dxlVunit_radsec = 0.00104719755 * dxlVunit;

    return dxlVunit_radsec;
}

int32_t DynamixelProPlusOvidiusShield::convertRadPsec2DxlVelUnits(double dxlVunit_radsec)
{
    // 1 [rad/sec] -> 9.5493 [rev/min] -> 954.93 [0.01 rev/min]

    int32_t dxlVunit = 954.93 * dxlVunit_radsec;

    return dxlVunit;
}

double DynamixelProPlusOvidiusShield::convertDxlAccelUnits2RadPsec2(int32_t dxlAunit)
{
    // 1 [dxlAunit] -> 1 [rev/min2] ->  0.00174532925199433 [rad/sec2]
    double dxlAunit_radsec2 = 0.00104719755 * dxlAunit;

    return dxlAunit_radsec2;
}

int32_t DynamixelProPlusOvidiusShield::convertRadPsec2_2_DxlAccelUnits( double dxlAunit_radsec2)
{
    // 1 [rad/sec2] -> 9.5493 [rev/min2]
    int32_t dxlAunit = 9.5493 * dxlAunit_radsec2;

    return dxlAunit;
}
// =========================================================================================================== //
