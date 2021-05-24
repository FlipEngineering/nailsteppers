#pragma once
#if defined(ARDUINO) && ARDUINO >= 100
    #include <Arduino.h>
#endif
#ifndef NAILSTEPPER
#define NAILSTEPPER

#ifndef CONFIG_FILE_INCLUDED
#include <config.h>
#endif 

#include <avr/io.h>
#include <avr/interrupt.h>
#include <TMCStepper.h>
#include <AccelStepper.h>
#include "TeensyStep.h"
//#include "MovementHandler.h"
#include "TMCHandler.h"
#include "TickHandler.h"
#include "MotionGenerator.h"
#include "Comm.h"

#include <queue>
#include <vector>



class NailStepper : public TMC2130Stepper{
    public:
    int id=NS_DEFAULT_ID;
    int current_dir = NS_DEFAULT_CURRENT_DIR;    
    int run_microsteps = NS_RUN_MICROSTEPS;

    double break_safe_multiplier = NS_BREAK_SAFE_MULTIPLIER;

    int dmx_settings_count = 0;
    int dmx_settings_channels[20][2];
    int dmx_settings_channels_old[20][2];

    double distanceDiff = 0.0;
    double distanceDiff2 = 0.0;
    
    MotionGenerator *trapezoidalProfile;
    Dmx *dmx;

    bool controllerIsStopping = false;
    bool controllerHasStopped = false;
    bool controllerHasStarted = false;

    int state = NS_STATE_INIT; // 0 == init, 1 == run
    //int scanType = SCAN_TYPE_HOME;
    int scanType = SCAN_TYPE_ACTIVE;
    int artnet_resolution = 1;

    //inital homing settings
    int init_timerSpeed = 44;
    int init_microsteps = NS_INIT_MICROSTEPS; //32;
    int init_hzbase = 1000000; //no idea why its 1.000.000,... perhaps 1M = 1sec?

    double current_rpm = 0.0;
    

    //----------tick timer-------------
    double breaking_distance_buffer = 0.0;
    double directStopSpeed = 0.01; // 0 - 1 speedFactor
    double min_distance_to_move_for = 0.02; //was 0.1mm when working quite well
    double safe_buffer = 0.0; //in mm - was 0.1mm when working quite well
    
    int tick_loop_time_min = TICK_LOOP_TIME_MIN; //microsecond
    int tick_loop_time_max = TICK_LOOP_TIME_MAX; //microsecond

    double t_position_follow_error = 0.0;
    double t_position_follow_error_min = 0.01; //was 0.3
    double t_position_follow_error_max = 1.0; // was 0.5
    
    double fade_speed_offset = 0.0;
    double catch_up_fade_acc_needed = 0.0;

    long smoothing_counter = 0;
    long fade_counter = 0;

    bool tick_message = false;
    int tick_message_controller_counter = 0;
    int tick_counter_interval = 30;

    double rough_speed_est = 0.0;
    double artnet_time_speed_estimation = 0.0;

    double distance_factor = 1.0;
    //-----------------------

    double t_acc_mm = 3200.0; //mm/sec


    int run_total_steps_per_revolution; //fullsteps * microsteps;    
    int run_steps_per_mm = 0;

    int gear_tooth_count; //tooth
    int belt_pitch; //mm per tooth pitch
    int fullsteps_per_revolution;
    int one_revolution_distance; //gear tooth * belt pitch (mm)
    int total_steps_per_revolution; //fullsteps * microsteps;    
    double one_step_distance_in_mm; //= (long double)one_rev_distance/(long double)total_steps_per_rev;

    double motor_speed_mm = 0.0;
    double motor_acc_mm = 0.0;
    double motor_pullin_speed_mm = 0.0;
    double motor_speed_steps = 0.0;
    double motor_acc_steps = 0.0;
    double motor_pullin_speed_steps = 0.0;

    float target;
    float newSpeed;
    double speedFac = 0.0;
    float speedFac_old;
    float speedFac_diff;

    int t_current_pos_step_old = 0;
    int last_loop_steps = 0;
    long t_current_pos_step = 0;
    long t_current_pos_step_diff = 0;
    long t_current_speed_step = 0;
    int t_target_pos_step = 0;
    int t_distance_step = 0;
    int t_break_dist_left_step = 0;    
    int est_break_dist_step = 0;
    int t_acc_step = 0;
    double avg_step_time = 0.0;
    
    unsigned long target_position_updated_counter = 0;

    double t_current_pos_mm = 0.0;
    double t_current_speed_mm = 0.0;
    double t_target_pos_mm = 0.0;
    double t_target_pos_mm_buffer = 0.0;
    double t_target_pos_mm_buffer_diff = 0.0;
    double t_target_pos_mm_buffer_diff_last_none_zero = 0.0;
    double t_target_pos_mm_buffer_diff_buffer = 0.0;
    double t_target_pos_mm_buffer_diff_buffer_diff = 0.0;
    double t_distance_mm = 0.0;
    double t_break_dist_left_mm = 0.0;
    double est_break_dist_mm = 0.0;

    bool target_position_updated = false;
    bool fade_in_progress = false;
    double fade_speed_est_mm = 0.0;


    

    int t_target_speed_step = 0;    
    double t_target_speed_mm = 0.0;
    double speedFacMultiplier = 0.0;
    double t_t = 0.0; //sec.
    double t_current_speedFactor = 0.0;

    double old_distance_mm = 0.0;
    double old_break_dist_mm = 0.0;
    double old_speed_mm = 0.0;

    double diff_distance_mm = 0.0;
    double diff_break_dist_mm = 0.0;
    double diff_speed_mm = 0.0;

    double speed_diff_realistic = 0.0;

    double dmx_raw = 0.0;

    long overshoot_counter = 0;
    long undershoot_counter = 0;
    int emergency_stop_counter = 0;
    double speed_before_emergency_stop = 0.0;

    int lc1 = 0;
    int lc2 = 0;
    int lc3 = 0;
    int lc4 = 0;
    int lc5 = 0;
    int lc6 = 0;
    int lc7 = 0;
    int lc8 = 0;
    int lc9 = 0;
    int lc10 = 0;
    int lc11 = 0;
    int lc12 = 0;
    int lc13 = 0;
    int lc14 = 0;

    unsigned long loop_time = micros();

    unsigned long loop_ms1 = micros();
    unsigned long loop_ms1_old = micros();
    unsigned long loop_ms2 = micros();
    unsigned long loop_diff = 0;
    unsigned long loop_ms1_diff = 0.0;

    int max_pos = 0;
    double dmx_step_size = 0.0;

    bool state_stall = false;
    bool state_active = false;
    bool drive_to_start_position = false;

    bool distanceKnown = false;

    int stall_counter = 0;
    int step_counter = 0;
    float distance_diff = -1.0;

    int steps_dir_A = 0;
    int steps_dir_B = 0;

    double distance_dir_A = 0.0;
    double distance_dir_B = 0.0;

    double full_distance = 0.0;
    double default_distance_mm = 0.0;

    int walk_dir;
    
    RotateControl *controller;
        
    int target_pos = 0;
    int target_pos_buffer = 0;

    unsigned long ts_incoming = 0;
    unsigned long ts_incoming_buffer = 0;
    unsigned long ts_incoming_buffer_diff = 0;
    unsigned long ts_processed = 0;

    int current_pos = 0;
    int old_pos = 0;
    

    double max_motor_speed_mm = 0.0;
    double max_motor_acc_mm = 0.0;

    double dmx_speed_factor = 0.0;

    IntervalTimer timer;
    //void timerFunc();
    
    //basically just forwarding to the TMCStepper lib
    uint16_t _pinCS; //chip select
    uint16_t _pinMOSI; // master out slave in
    uint16_t _pinMISO; // master in slave out
    uint16_t _pinSCK; //source clock
    uint16_t _pinStep;
    uint16_t _pinDir;
    uint16_t _pinEn;
    
    NailStepper(uint16_t pinCS, uint16_t pinMOSI, uint16_t pinMISO, uint16_t pinSCK, uint16_t pinStep, uint16_t pinDir, uint16_t pinEn, int max_m_speed, int max_m_acc);
    //:
      //  TMC2130Stepper(pinCS, TMC_R_SENSE, pinMOSI, pinMISO, pinSCK) {}
    //NailUnit(CS_PIN, TMC_R_SENSE, SW_MOSI, SW_MISO, SW_SCK)

    //TMC2130Stepper driver;//(uint16_t, float, uint16_t, uint16_t, uint16_t); 
    //TMC2130Stepper driver(_pinCS, TMC_R_SENSE, _pinMOSI, _pinMISO, _pinSCK);  
    Stepper stepper;
    
    TMC2130_n::DRV_STATUS_t drv_status;

    void initDriver();
    float getRPM();
    void setDistance(int gear_count, int belt_pit, int fs_per_rev);
    void scanDistance();
    void printStallInfo();
    void printDebugInfo(TMC2130_n::DRV_STATUS_t drv_status);
    void printDriverStatus(TMC2130_n::DRV_STATUS_t drv_status);
    void walkTest_setup();
    void walkTest();
    int getAvailableStepsForFullDistance(int ms);
    void finishInit_old();
    void finishInit();
    void scanInit();
    void scanInit_old();
    unsigned long tick_old();
    unsigned long tick();
    unsigned long tick_continues_dir();
    
    unsigned long tick_motion_gen();
    void debug_tick(int i);
    void move_steps(int steps, int dir);
    void move_distance_mm(int mm, int dir);
    void init_run_steps_per_mm();
    void offsetEndstops(int mm, int dir);
    void setState(int s);
    void changeMicrosteps(int ms);
    void prepareRun(double m_sp_mm, double m_acc_mm, double m_pull_sp_mm, int invertDir);
    int mm_to_steps(double mm);
    double steps_to_mm(int st);
    int getMs();
    int getStepsPerRevolution();
    void calculateSizesForMicrostepping(int ms);
    void adjustMicrostepping(int ms_new);
    void handle_dmx_settings(double devider);
    void testQueue();
    void testVector();
    double get_dmx_resolution();
    double calculate_dmx_speed_ratio();
    
    int stepperInvert = 0;
    void setInvert();
    void dmxSettingsUpdate();

    void startController();
    void stopController();
    void evalStopController();
    void updateController();
    void handleController();

    long mt = 0.0; //millisecond timer
    long mt2 = 0.0;
    long mt_diff = 0.0;
    int stp_cnt = 0;
    int esStep = 0;
    double es_msf = 0.0;
    double esMaxSpeedFac = 0.0;

    bool everySecond();
    bool everyMillis(int mil);
    uint32_t tstp =0;

    void print_chopconf();
    void print_pwmconf();

    void setGear();
    //int getStepperSetting(const char* options[4]);
    int getStepperSetting(const char* option0,const char* option1,const char* option2,const char* option3);
    void setFullDistance();
    void setDriver();
    int setMode();
    int driver_dmx_setting = 0;


    int driver_init = DRIVER_INIT;
    int scan_init_old = SCAN_INIT_OLD;
    int finish_init_old = FINISH_INIT_OLD;

    TickHandler th = TickHandler(MOTOR_MAX_SPEED, MOTOR_MAX_ACC);
};

#endif