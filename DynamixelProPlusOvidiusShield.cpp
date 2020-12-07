 /*
  * DynamixelProPlusOvidiusShield.cpp  - Library for controlling Dynamixels Pro+ of a Metamorphic Manipulator using DunamixelShield-Dynamixel2Arduino ROBOTIS libraries
  * Created by N.A. Stravopodis, December, 2020.
  */

#include "Arduino.h"
#include <DynamixelShield.h>
#include <Dynamixel2Arduino.h>
#include <Dynamixel2Arduino.h>
#include "DynamixelProPlusOvidiusShield.h"

// Include Motor Configuration files from folder ~/Arduino/libraries/
#include <definitions.h>                            
#include <motorIDs.h>                               
#include <contolTableItems_LimitValues.h>
#include <StepperMotorSettings.h>

using namespace std;
using namespace ControlTableItem;

// Constructor
DynamixelProPlusOvidiusShield::DynamixelProPlusOvidiusShield(uint8_t *DxlIDs){

    uint8_t _DXL1_ID = DxlIDs[0];
    uint8_t _DXL2_ID = DxlIDs[1];
    uint8_t _DXL3_ID = DxlIDs[2];
}

// =========================================================================================================== //

bool DynamixelProPlusOvidiusShield::blinkDynamixelLeds(uint8_t *DxlIDs, int DxlIds_size, unsigned char *led_indicator, unsigned long interval, int times)
{
 /*
  *  Blinks Dynamixels given the ID numbers and the desired color(as array 3 elements) and time interval/times of blink
  *  No timer interrupt used here
  */


}

// =========================================================================================================== //

// Ping Dynamixels
bool DynamixelProPlusOvidiusShield::pingDynamixels(uint8_t *DxlIDs, int DxlIds_size, DynamixelShield dxl) {

/*
 *  Pings Connected Dynamixels given the correct ID number as set using thw Wizard Software
 */
    Serial.println("[    INFO    ]      Executing: pingDynamixels");

  for(int id_count = 0; id_count < DxlIds_size; id_count++){
        dxl_comm_result = dxl.ping(DxlIDs[id_count]);
        if (dxl_comm_result == false)
        {
            Serial.print("[   ERROR   ]   Ping of Dynamixel:"); Serial.print(DxlIDs[id_count]); Serial.println("    FAILED  ");
            return false;
        }
        else
        {
        ping_indicator[0] = 0;    //R
        ping_indicator[1] = 255;  //G
        ping_indicator[2] = 0;    //B
        dxl.writeControlTableItem(LED_GREEN, DxlIDs[id_count], ping_indicator[1]); // write GREEN VALUE to DXL Control Table
        //dxl.ledOn(DXL_ID);
        delay(500);
        dxl.writeControlTableItem(LED_GREEN, DxlIDs[id_count], 0);
            
            dxl.writeControlTableItem(LED_GREEN, DxlIDs[id_count], ping_indicator[1]); // write GREEN VALUE to DXL Control Table
            // Print serial monitor message
            Serial.print("[ID:"); Serial.print(DxlIDs[id_count]); Serial.print("] ping Succeeded. Dynamixel model number : "); Serial.println(dxl_model_number[id_count]);
        }
    
  }

  return true;
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

