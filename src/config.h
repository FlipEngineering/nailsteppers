#pragma once
#if defined(ARDUINO) && ARDUINO >= 100
    #include <Arduino.h>
#endif

#ifndef CONFIG_FILE_INCLUDED
#define CONFIG_FILE_INCLUDED 1

#include <map>
#include <String>
#define TEST_CONST 1

#define DEBUG_OUTPUT 0 // 0 or 1

//-----------------------------
//------- MAIN SETTINGS -------
//-----------------------------
#define SERIAL_BAUD 256000
#define SETUP_INIT_DELAY 3000 //3000ms setup delay

//----- DEV/DEBUG OPTIONS ----
#define MONITOR_LOOP_TIME false // Debug info of the main loop iteration time
#define DIP_ENABLED false //The program needs the dip to be connected otherwise it gets stuck - so this needs to changed during dev to avoid these issues
#define DISPLAY_TICK_TIMER false // Debug output of the actual tick timing
#define ETHERNET_EVAL false // On/Off for most parts of the program. Just checking if the ethernet plug works. !!! - CONNECTED_STEPPERS 0 //needs to be used when ethernet eval is set to true
#define TICK_TIMER_MIN_INIT 1000 //aim high so it will hit less
#define TICK_TIMER_MAX_INIT 0 //aim low so it will hit more
#define TICK_TIMER_PRIORITY 255 // lowest priority, potentially long caclulations need to be interruptable by TeensyStep
//----- END DEV/DEBUG --------

#define TICK_LOOP_TIME_MIN 10
#define TICK_LOOP_TIME_MAX 3000

#define SW_MOSI          0 // Software Master Out Slave In (MOSI)
#define SW_MISO          1 // Software Master In Slave Out (MISO)
#define SW_SCK           32 // Software Slave Clock (SCK)



#define GEAR_IDX 0
#define EN_IDX 1
#define DIR_IDX 2
#define STEP_IDX 3
#define CS_IDX 4
#define INV_DIR_IDX 5

#define STEPPER_CONFIG_COUNT 6
//----- STEPPER CONFIG  | GEAR TEETH | EN_PIN | DIR_PIN | STEP_PIN | CS_PIN | INV_DIR
#define STEPPER_1       {     20     ,    8   ,    7    ,    6     ,   31   ,  1     }
#define STEPPER_2       {     20     ,    27  ,    25   ,    26    ,   24   ,  1     }     
#define STEPPER_3       {     20     ,    15  ,    17   ,    16    ,   14   ,  1     }
#define STEPPER_4       {     30     ,    20  ,    22   ,    21    ,   19   ,  0     }


//#define CONNECTED_STEPPERS 1
//#define STEPPERS {STEPPER_1}
//#define CONNECTED_STEPPERS 2
//#define STEPPERS {STEPPER_1, STEPPER_2}
//#define CONNECTED_STEPPERS 3
//#define STEPPERS {STEPPER_1, STEPPER_2, STEPPER_3}
#define CONNECTED_STEPPERS 4
#define STEPPERS {STEPPER_1, STEPPER_2, STEPPER_3, STEPPER_4}

#define SCAN_TYPE_FULL_SCAN 0 //Stepper distance scan type - detect both endstops and calculate distance
#define SCAN_TYPE_HOME 1 //Stepper distance scan type - just detect one side and stop there

//#define SCAN_TYPE_ACTIVE SCAN_TYPE_FULL_SCAN
#define SCAN_TYPE_ACTIVE SCAN_TYPE_HOME


#define STEPPER_SCAN_OFFSET true //Offset to move away from endstop after probing the distances
#define STEPPER_SCAN_OFFSET_DISTANCE 0.0 //mm
#define STEPPER_SCAN_OFFSET_DIRECTION 1 //this can change depending on what way around the cable has been connected / made

//#define STEPPER_SCAN true //Defines if during init the stepper will scan for the available distance
#define STEPPER_SCAN false
#define STEPPER_DEFAULT_DISTANCE_MM 168.0 //mm - If STEPPER_SCAN is false this is the fixed distance the steppers will be using for travel

#define BELT_PITCH 2 //mm - Pitch of belt used in mm
#define STEPPER_FULL_STEPS_PER_REV 200.0

//#define MOTOR_TIME_UNTIL_MAX_SPEED 0.25 //seconds until max speed 
#define MOTOR_TIME_UNTIL_MAX_SPEED 0.25 //seconds until max speed 
#define ACC_MULTIPLIER (1.0 / MOTOR_TIME_UNTIL_MAX_SPEED)

#define MOTOR_MAX_SPEED 800.0//400.0 //800.0 //800.0//mm/sec
#define MOTOR_MAX_ACC (MOTOR_MAX_SPEED * ACC_MULTIPLIER)

#define PULL_IN_FACTOR 8.0 //rough guessing - whatever that means? 
#define PULL_IN_SPEED_MM (MOTOR_MAX_SPEED / PULL_IN_FACTOR)

#define TICK_TIMER_UPPER_LIMIT 3000 //us - protection for startup instability

#define MAIN_LOOP_NO_ARTNET_READ_DELAY 400 //microseconds


#ifndef STEPPER_1
#define STEPPER_1 {}
#endif 

#ifndef STEPPER_2
#define STEPPER_2 {}
#endif 

#ifndef STEPPER_3
#define STEPPER_3 {}
#endif 

#ifndef STEPPER_4
#define STEPPER_4 {}
#endif 

//----- NAIL STEPPERS ------
#define NS_STATE_INIT 0 //NailStepper State Init Mode (different Microsteps) 
#define NS_STATE_RUN 1 //NailStepper State Run Mode  (different Microsteps) 
#define NS_DEFAULT_ID 0
#define NS_DEFAULT_CURRENT_DIR -1

//64 is smoooth - but only slow possible
    //32 - smooth too, almost not audible - but more loud then 64 when going slow
    //16 - runs almost perfect (except @ 0.5 fade test - get's stuck)
    //8 - loud - can do everything
    //4 - 
    //int run_microsteps = 8;//32 triggers overshoot/undershoot protection - 8; //4 for all programming, lets see how 8 does
//#define NS_RUN_MICROSTEPS 16
#define NS_RUN_MICROSTEPS 16
//#define NS_INIT_MICROSTEPS 16 //was usually 32
#define NS_INIT_MICROSTEPS 16

#define NS_BREAK_SAFE_MULTIPLIER 0.01

//----- TMC SETTINGS ------

#define TMC_STALL_VALUE 3 //8 was what they used in marlin as default (i think) //3 (was my last steppers sweet spot) // [-64..63] (init was 15) //with one connected 2 was working 
#define TMC_R_SENSE 0.11f // Match to your driver

//------old
#define STALL_VALUE TMC_STALL_VALUE
///-----old

//----- ARTNET/DMX SETTINGS ------
//#define ACTIVE_UNIVERSE 0 - old see dmxAdditional
//#define CHANNEL_START 0

#define RES_SINGLE 1 //Single Channel 8bit
#define RES_DOUBLE 2 //Double CHannel 16bit
#define ARTNET_RESOLUTION 2    // 1 or 2  --- 1 Channel (0-255) default size --- 2 Channel - high resolution (255*256)

#define ARTNET_DATA_LENGTH (CONNECTED_STEPPERS * ARTNET_RESOLUTION)


#define DMX_SETTING_TMC_SETTINGS "tmc"
#define DMX_SETTING_TMC_CONF 111
#define DMX_SETTING_TMC_OLD 99
#define DMX_SETTING_TMC_MINIMAL 66
#define DMX_SETTING_TMC_NEW 33

#define DMX_SETTING_DRIVE_TYPE "drive_type"

#define DMX_SETTING_GEAR_COUNT_0 "g0"
#define DMX_SETTING_GEAR_COUNT_1 "g1"
#define DMX_SETTING_GEAR_COUNT_2 "g2"
#define DMX_SETTING_GEAR_COUNT_3 "g3"

#define DMX_SETTING_TRAVEL_DISTANCE_ADJUST_0 "d0"
#define DMX_SETTING_TRAVEL_DISTANCE_ADJUST_1 "d1"
#define DMX_SETTING_TRAVEL_DISTANCE_ADJUST_2 "d2"
#define DMX_SETTING_TRAVEL_DISTANCE_ADJUST_3 "d3"

#define DMX_SETTING_INVERT_DIR_0 "i0"
#define DMX_SETTING_INVERT_DIR_1 "i1"
#define DMX_SETTING_INVERT_DIR_2 "i2"
#define DMX_SETTING_INVERT_DIR_3 "i3"

#define DMX_SETTINGS_ACTIVE_INIT "dsaI"
#define DMX_SETTINGS_ACTIVE_ALWAYS "dsaA"

#define DMX_SETTINGS_ACTIVE_INIT_VALUE 123
#define DMX_SETTINGS_ACTIVE_ALWAYS_VALUE 123

#define DMX_EMPTY_BUFFER 8

//ADJUST THIS IF THE COUNT CHANGES!!!!
#define DMX_ADDITIONAL_SETTINGS 16

struct dmxAdditional{
    typedef std::pair<int, int> pair;

    std::map<String, pair> settings = {
        {DMX_SETTINGS_ACTIVE_INIT,                std::make_pair(0, 0)},
        {DMX_SETTINGS_ACTIVE_ALWAYS,              std::make_pair(1, 0)},
        {DMX_SETTING_TMC_SETTINGS,                std::make_pair(2, 0)},
        {DMX_SETTING_DRIVE_TYPE,                  std::make_pair(3, 0)},

        {DMX_SETTING_GEAR_COUNT_0,                std::make_pair(4, 0)},
        {DMX_SETTING_GEAR_COUNT_1,                std::make_pair(5, 0)},
        {DMX_SETTING_GEAR_COUNT_2,                std::make_pair(6, 0)},
        {DMX_SETTING_GEAR_COUNT_3,                std::make_pair(7, 0)},
        
        {DMX_SETTING_TRAVEL_DISTANCE_ADJUST_0,    std::make_pair(8, 0)},
        {DMX_SETTING_TRAVEL_DISTANCE_ADJUST_1,    std::make_pair(9, 0)},
        {DMX_SETTING_TRAVEL_DISTANCE_ADJUST_2,    std::make_pair(10,0)},
        {DMX_SETTING_TRAVEL_DISTANCE_ADJUST_3,    std::make_pair(11,0)},
        
        {DMX_SETTING_INVERT_DIR_0,                std::make_pair(12,0)},
        {DMX_SETTING_INVERT_DIR_1,                std::make_pair(13,0)},
        {DMX_SETTING_INVERT_DIR_2,                std::make_pair(14,0)},
        {DMX_SETTING_INVERT_DIR_3,                std::make_pair(15,0)},
    };

    byte ip[4] = {2, 0, 0, 2};
    uint16_t active_universe = 0;
    int channel_start = 0;
    byte subnet[4] = {255, 255, 255, 0};
    byte dns[4] = {2, 0, 0, 1};
    byte gateway[4] = {2, 0, 0, 1};
    const uint8_t oem[2] = {0xFF, 0xFF};

};


struct tmc_driver_conf_init{
    //                  CONF        OLD         MINIMAL
    int toff =          5;       // 4           5
    int blank_time =    36;      // 24          36       
    int rms_current =   500;     // 500         600
    int rms_idle =      0.4;     // -           0.4
    int tcoolthrs =     0xFFFFFF;// 0xFFFFFF
    int thigh =         0;       // 0
    int semin =         5;       // 5       
    int semax =         2;       // 2  
    int sedn =          0b01;    // 0b01  
    int sgt =           3;       // 3  
};


struct tmc_driver_conf_run{
    //                      CONF    OLD     MINIMAL
    int toff =              3;//5;   // 2       -
    
    int blank_time =        36;  // |       -
  //int tbl =               -    // 2 (=36) -

    int rms_current =       1000; // 800     -    
    int rms_idle =          0.4; // -       -         multiplier for ihold
  //int ihold  =            -    // 17      -
  //int iholddelay          -    // 2       -

    int en_pwm_mode =       1;   // 1       -
    int pwm_autoscale =     1;   // 1       -
    int pwm_grad =          4;//1;//4;   // 1       -
    int pwm_ampl =          32;//200; // 255     -
    int pwm_freq =          1;//1;   // 0       -

    int thigh =             0;   // 0       -
    int semin =             5;   // -       -
    int semax =             2;   // -       -

    int sedn =              0b01;// -       -
    int sgt =               3;   // -       -

    int hstrt =             3;//5;//3;   // 4       -
    int hend =              2;//3;//2;//0;   // 0       -
    
    int tcoolthrs =         0;   // 0       -
    int tpwmthrs =          0;   // 0       -
};


#define RMS_MINIMAL 900 //RMS_CURRENT for Minimal TMC settings


//#define IP {2,0,0,2} //needs to be working with dip switch in combi to avoid conflicts
//#define SUBNET {255, 255, 255, 0}
#define DNS {2, 0, 0, 1}
#define GATEWAY {2, 0, 0, 1}

//#define COM_OEM {0xFF, 0xFF}
#define COM_PORT 8888

#define DMX_MAX_SETTINGS_CHANNELS 20

#define TEENSY_0_MAC_END 141
#define TEENSY_0_IP_END 10 //Controller ID is derived from IP - 10

#define TEENSY_1_MAC_END 42
#define TEENSY_1_IP_END 11 //Controller ID is derived from IP - 10

#define TEENSY_2_MAC_END 221
#define TEENSY_2_IP_END 12 //Controller ID is derived from IP - 10

#define TEENSY_3_MAC_END 213
#define TEENSY_3_IP_END 13 //Controller ID is derived from IP - 10

#define TEENSY_4_MAC_END 170
#define TEENSY_4_IP_END 14 //Controller ID is derived from IP - 10

#define TEENSY_5_MAC_END 56
#define TEENSY_5_IP_END 15 //Controller ID is derived from IP - 10

#define TEENSY_6_MAC_END 197
#define TEENSY_6_IP_END 16 //Controller ID is derived from IP - 10


//----------------newer stuff
//------------------
#define TICK_NEW 1
#define TICK_OLD 0
#define TICK_MOTION_GEN 2

//#define ACTIVE_TICK TICK_MOTION_GEN
#define ACTIVE_TICK TICK_OLD

//#define SCAN_INIT_OLD 1
//#define FINISH_INIT_OLD 1

#define SCAN_INIT_OLD 0
#define FINISH_INIT_OLD 0

#define MODE_NAIL 0
#define MODE_RAIN 123

//#define STEP_MODE MODE_RAIN
#define STEP_MODE MODE_NAIL

//#define JUNCTION_DEVIATION 

#define DRIVER_INIT_CONF DMX_SETTING_TMC_CONF
#define DRIVER_INIT_MINIMAL 1
#define DRIVER_INIT_NORMAL 0

//#define DRIVER_INIT DRIVER_INIT_MINIMAL
#define DRIVER_INIT DRIVER_INIT_CONF

class conf{
    int test123 = 4;
};

#endif