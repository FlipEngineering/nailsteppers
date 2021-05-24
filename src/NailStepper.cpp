#include "NailStepper.h"
#include <stdio.h> 

asm(".global _printf_float");

int global_step = -1;
int global_step_counter = 0;

float scan_distance_offset = 1.5;//0.9;
float scan_distance_min_distance = 10.0;

void initStepTimer(){
  digitalWrite(global_step, !digitalRead(global_step));
  global_step_counter++;
};


//---------print bytes------

#include <stdio.h>      /* printf */
#include <string.h>     /* strcat */
#include <stdlib.h>     /* strtol */

const char *byte_to_binary(int x){
    static char b[9];
    b[0] = '\0';

    int z;
    for (z = 128; z > 0; z >>= 1)
    {
        strcat(b, ((x & z) == z) ? "1" : "0");
    }

    return b;
}


const char *byte32_to_binary(u_int32_t x){
    static char b[37];
    b[0] = '\0';

    u_int32_t z;
    int counter = 0;
    for (z = 2147483648; z > 0; z >>= 1)
    {   
        counter++;
        if(counter%8 == 1){
          strcat(b, ".");
        }
        strcat(b, ((x & z) == z) ? "1" : "0");
    }

    return b;
}


NailStepper::NailStepper(uint16_t pinCS, uint16_t pinMOSI, uint16_t pinMISO, uint16_t pinSCK, uint16_t pinStep, uint16_t pinDir, uint16_t pinEn, int max_m_speed, int max_m_acc)
  :TMC2130Stepper(pinCS, TMC_R_SENSE, pinMOSI, pinMISO, pinSCK) 
  ,stepper(pinStep, pinDir)
{
    _pinCS = pinCS;
    _pinMOSI = pinMOSI;
    _pinMISO = pinMISO;
    _pinSCK = pinSCK;
    _pinStep = pinStep;
    _pinDir = pinDir; 
    _pinEn = pinEn;
    max_motor_speed_mm = max_m_speed;
    max_motor_acc_mm = max_m_acc;

    controller = new RotateControl;

    pinMode(pinEn, OUTPUT);
    pinMode(pinStep, OUTPUT);
    pinMode(pinCS, OUTPUT);
	  digitalWrite(pinCS, HIGH);   
    pinMode(pinDir, OUTPUT);
    digitalWrite(pinEn, LOW);   

    trapezoidalProfile = new MotionGenerator(MOTOR_MAX_SPEED, MOTOR_MAX_ACC, 0);
	
};

char buf[40];
void out(float v, char const *text){
  dtostrf(v,10,4,buf);
  //snprintf(*buf, 20, "%10.5f", v);
  if(DEBUG_OUTPUT)Serial.print(text);
  if(DEBUG_OUTPUT)Serial.print(buf);
  if(DEBUG_OUTPUT)Serial.print(" ");
}

unsigned long NailStepper::tick_old(){
  //int x = dmx_getSetting(DMX_SETTING_GEAR_COUNT);
  //Serial.println(x);

  loop_ms1_old = loop_ms1;
  loop_ms1 = micros();
  loop_ms1_diff = loop_ms1-loop_ms1_old;

  tick_counter_interval = 300;

  tick_message_controller_counter++;
  if(tick_message_controller_counter >= tick_counter_interval){
    tick_message_controller_counter = 0;
    tick_message = true;
  }else{
    tick_message = false;
  }


  t_current_pos_step_old = t_current_pos_step;
  t_current_pos_step = stepper.getPosition();
  last_loop_steps = std::abs(t_current_pos_step - t_current_pos_step_old);
  t_current_pos_mm = steps_to_mm(t_current_pos_step);

  

  //watch out - dmx_step_size is now in mm not steps
  //t_target_pos_step =  target_pos * dmx_step_size;
  t_target_pos_mm = target_pos * dmx_step_size; // steps_to_mm(t_target_pos_step);

  t_target_pos_mm_buffer = target_pos_buffer * dmx_step_size; //steps_to_mm(target_pos_buffer * dmx_step_size);
  t_target_pos_mm_buffer_diff = t_target_pos_mm - t_target_pos_mm_buffer;
  if(t_target_pos_mm_buffer_diff != 0.0){
    t_target_pos_mm_buffer_diff_last_none_zero = t_target_pos_mm_buffer_diff;
  }

  //t_distance_step = t_target_pos_step - t_current_pos_step;
  t_distance_mm = t_target_pos_mm - t_current_pos_mm; //steps_to_mm(t_distance_step); //convertion from steps to mm at current settings
  
  t_current_speed_step = t_current_speedFactor * motor_speed_steps;
  t_current_speed_mm = t_current_speedFactor * motor_speed_mm; // mm/sec
  
  //double t_offset_prediction = 0.0005;
  t_t = (t_target_speed_mm - t_current_speed_mm) / t_acc_mm;
  //t_t -= t_offset_prediction;

  est_break_dist_mm = ((t_current_speed_mm + t_target_speed_mm) * t_t) / 2.0;
  est_break_dist_mm *= -1.0 ; //1.1 - saefty factor

  //t_break_dist_left_mm = std::abs(t_distance_mm) - std::abs(est_break_dist_mm);
  if(t_distance_mm > 0.0){
    t_break_dist_left_mm = t_distance_mm - est_break_dist_mm;
  }else if(t_distance_mm < 0.0){
    t_break_dist_left_mm = (-t_distance_mm) - est_break_dist_mm;
  }
  //t_break_dist_left_mm = t_distance_mm - est_break_dist_mm;

  diff_break_dist_mm = old_break_dist_mm - est_break_dist_mm;
  old_break_dist_mm = est_break_dist_mm;

  diff_distance_mm = old_distance_mm - t_distance_mm;
  old_distance_mm = t_distance_mm;

  diff_speed_mm = old_speed_mm - t_current_speed_mm;
  old_speed_mm = t_current_speed_mm;

  //overflow safeguard
  if(ts_incoming_buffer >= ts_incoming){
    //calculate the time for the changed value (us)
    ts_incoming_buffer_diff = ts_incoming_buffer - ts_incoming;
    artnet_time_speed_estimation = (1.0 / (double)ts_incoming_buffer_diff) * 1E6;
  }
 
  
  rough_speed_est = (double)std::abs(t_target_pos_mm_buffer_diff) * artnet_time_speed_estimation; //33.0;
  //artnet update interval time, based on diff of last two received signals
  if(rough_speed_est == 0.0){
    //trying to create a backup speed est to use in case we are not there yet
    //naja not really,....
    //rough_speed_est = (double)std::abs(t_target_pos_mm_buffer_diff_last_none_zero) * artnet_time_speed_estimation; //33.0;    
  }
  

  if(target_position_updated){
    target_position_updated = false;
    fade_in_progress = false;
  }
  if(!fade_in_progress){
    fade_speed_offset = 0.0;
    catch_up_fade_acc_needed = 0.0;
  }

  if(t_target_pos_mm_buffer_diff == 0.0){
    //fade_in_progress = false;
    //no fade in progress
    //Serial.println("-");
  }else if(t_distance_mm <= t_position_follow_error_max && t_distance_mm >= t_position_follow_error_min){
    //fade in progress
    //out(t_distance_mm,"Dist:");
    //Serial.println("Fade!");
    //get requested fade speed
    fade_speed_est_mm = (double)t_target_pos_mm_buffer_diff * artnet_time_speed_estimation;
    //compare to current speed

    if((t_current_speed_mm > 0.0 && fade_speed_est_mm > 0.0) || (t_current_speed_mm < 0.0 && fade_speed_est_mm < 0.0)){
      //when driving both same way (things are roughly right)
      fade_in_progress = true;
      fade_speed_offset = fade_speed_est_mm - t_current_speed_mm;
      catch_up_fade_acc_needed = fade_speed_offset / artnet_time_speed_estimation;
      
      //out(catch_up_fade_acc_needed,"Catchup:");
      //Serial.println("");
      // if 
    }else{
     // out(t_distance_mm,"Dist:");
      //Serial.println("x");
    }
    

    //if speed diff is within tolerance don't change anything otherwise adjust acceleration to reach requested speed
  }else{
    fade_in_progress = false;
  }
  
  
  if(false){
    out(ts_incoming,"TS-1:");
    out(ts_incoming_buffer,"TS-2:");
    out(ts_incoming_buffer_diff,"TS-Diff:");
    out(artnet_time_speed_estimation,"TimeEst:");
    out(rough_speed_est,"RoughSpeedEst:");
    out(t_current_speed_mm, "CurrentSpeed: ");
    if(DEBUG_OUTPUT)Serial.println("");
  }
  
  //dynamic now, let's see
  // calculating the speed adjustment needed at current loop time to achive the maximum acc defined for motor
  speed_diff_realistic = (motor_acc_mm * loop_ms1_diff)/ 1E6;

  speedFacMultiplier = speed_diff_realistic/motor_speed_mm;
  double max_speed_speedFacMultiplier = speedFacMultiplier;


  //double break_factor = 
  //break_safe_multiplier = 0.030; // works for high velocity - was 0.005
  
  break_safe_multiplier = 0.010; // 0.01 works for 16ms

  breaking_distance_buffer = std::abs(t_current_speed_mm)*break_safe_multiplier; // 0.5% extra? 3.0; // in mm

  double speedFactorManipulator = 1.0;

  bool smoothing_enabled = true;

  if(fade_in_progress){
    fade_counter++;

    double fade_breaking_distance_buffer = std::abs(fade_speed_est_mm)*break_safe_multiplier; // 0.5% extra? 3.0; // in mm

    if(t_break_dist_left_mm >= breaking_distance_buffer){
      if(t_break_dist_left_mm >= fade_breaking_distance_buffer){
        //if(t_distance_mm <= t_position_follow_error_max && t_distance_mm >= t_position_follow_error_min){
          
          //fade in progress
          //let's adjust the acc and timing so we would get to desired speed within one artnet loop time
          //double fade_speed_diff_realistic = (catch_up_fade_acc_needed * artnet_time_speed_estimation)/ 1E6;
          double fade_speed_diff_realistic = (catch_up_fade_acc_needed * loop_ms1_diff)/ 1E6;
          if(fade_speed_diff_realistic < speed_diff_realistic){
            speedFacMultiplier = fade_speed_diff_realistic/motor_speed_mm;
          }else{
            fade_in_progress = false;
          }
        //}else{
        //  fade_in_progress = false;
        //}
      }else{
        fade_in_progress = false;
      }
    }else{
      fade_in_progress = false;
    }
  }

  if(smoothing_enabled && !fade_in_progress){
    distance_factor = std::abs(t_distance_mm) / full_distance;

    if (distance_factor < 0.5 && t_break_dist_left_mm >= breaking_distance_buffer && t_distance_mm != 0.0){
      smoothing_counter++;
      if(distance_factor >= 0.01){        
        speedFactorManipulator = distance_factor*2; //*2;  
      }else{        
        speedFactorManipulator = distance_factor*2; //distance_factor;
      }
      //speedFactorManipulator = distance_factor;
      t_acc_mm = motor_acc_mm * speedFactorManipulator;
      //speedFactorManipulator = 0.1;
    }else{
      //backup to safe settings
      t_acc_mm = motor_acc_mm;
    }
  }
  
  speedFacMultiplier *= speedFactorManipulator;

  //speedFacMultiplier = 0.0022; //should represent the calcualted acc in relation to loop time (does it?)

  //tick_message = false;
  if(tick_message && id == 0 ){
    
    //Serial.print("LOST STEPS:");
    //Serial.println(LOST_STEPS());

    //Serial.print("PWM_SCALE:");
    //Serial.println(PWM_SCALE());
    
    //Serial.print("TSTEP:");
    //Serial.println(TSTEP());

    //Serial.print("Nail:");
    //Serial.print(id);
    //Serial.print(" ms:");
    //Serial.println(microsteps());
    
  }
  if(tick_message && id == 0 && false && DEBUG_OUTPUT){
    Serial.print(lc1);Serial.print(" ");
    Serial.print(lc2);Serial.print(" ");
    Serial.print(lc3);Serial.print(" 4:");
    Serial.print(lc4);Serial.print(" ");
    Serial.print(lc5);Serial.print(" ");
    Serial.print(lc6);Serial.print(" 7:");
    Serial.print(lc7);Serial.print(" ");
    Serial.print(lc8);Serial.print(" ");
    Serial.print(lc9);Serial.print(" 10:");
    Serial.print(lc10);Serial.print(" ");
    Serial.print(lc11);Serial.print(" ");
    Serial.print(lc12);Serial.print(" 13:");
    Serial.print(lc13);Serial.print(" ");
    Serial.print(lc14);Serial.print(" ");
    Serial.print("S:");
    Serial.print(smoothing_counter);
    Serial.print(" ");
    Serial.print("F:");
    Serial.print(fade_counter);
    Serial.print(" ");
    Serial.print("E:");
    Serial.print(emergency_stop_counter);
    Serial.print(" ");
    
    out(t_current_pos_mm,"CurrentPos:");
    out(t_target_pos_mm,"TarPos:");
    out(speedFac," SpeedFac:");
    
    //out(full_distance," FullDist: ");
    
    Serial.print(" targetPos:");
    Serial.print(target_pos);
    
    //Serial.print(" dmx_raw:");
    //Serial.print(dmx_raw);    
    out(t_distance_mm," dist_mm:");
    out(distance_factor," dist_fac:");
    
    out(speedFacMultiplier," mul:");
    
    Serial.println(" #");
  }


  if(speedFac != 0.0){
    //driving, what to do? speed up, break or keep speed
    if((t_current_pos_mm > (full_distance-safe_buffer)) && speedFac > 0.0){
      //passing safe-buffer. emergency-stop - top
      speed_before_emergency_stop = speedFac;
      speedFac = 0.0;
      emergency_stop_counter++;
      lc1++;
      //Serial.print("1.1 ");
    }else if((t_current_pos_mm < safe_buffer) && speedFac < 0.0){
      //passing safe-buffer. emergency-stop - bottom
      speed_before_emergency_stop = speedFac;
      speedFac = 0.0;
      emergency_stop_counter++;
      lc2++;
      Serial.printf("Emergency Stop %d", id);
    }else if(t_distance_mm < 0.0 && speedFac > 0.0){
      //going the wrong way!!! -> break
      
      //speedFac -= speedFacMultiplier;
      speedFac -= max_speed_speedFacMultiplier;
      if(speedFac < 0.0){
        speedFac = 0.0;
      }
      if(DEBUG_OUTPUT)Serial.println("Overshooting");
      //Serial.print("1.3 ");
      //if(t_current_pos_step > max_pos){
      //  overshoot_counter++;
      //}
      lc3++;
    }else if(t_distance_mm > 0.0 && speedFac < 0.0){
      if(DEBUG_OUTPUT)Serial.println("Undershooting");
      //going the wrong way!!! -> break
      //if(t_current_pos_step < 0){
      //  undershoot_counter++;
      //}
      
      speedFac += max_speed_speedFacMultiplier;
      if(speedFac > 0.0){
        speedFac = 0.0;
      }
      
      //speedFac += speedFacMultiplier;
      lc4++;
      //Serial.print("1.4 ");
    }else if(t_break_dist_left_mm <= breaking_distance_buffer ){ 
      if(speedFac > directStopSpeed || speedFac < -directStopSpeed ){
        if(std::abs(speedFacMultiplier) < std::abs(max_speed_speedFacMultiplier)){
          if(DEBUG_OUTPUT)Serial.println("increased breaking!");
          out(speedFacMultiplier,"Old:");
          out(max_speed_speedFacMultiplier,"New:");
          speedFacMultiplier = max_speed_speedFacMultiplier;
        }
        // driving into the end stop? -> break
        if(speedFac > 0.0){
          speedFac -= speedFacMultiplier;
          if(speedFac < 0.0){
            speedFac = 0.0;
          }
          //if(speedFac <= directStopSpeed){
          //  speedFac = 0.0;
          //}
          
          //Serial.print("1.5 ");
        }else if(speedFac < 0.0){
          speedFac += speedFacMultiplier;
          if(speedFac > 0.0){
            speedFac = 0.0;
          }
          //if(speedFac >= -directStopSpeed){
          //  speedFac = 0.0;
          //}
          
          //Serial.print("1.6 ");
        }
      }else{
        speedFac = 0.0;
        lc5++;
      }
    }else if(t_distance_mm == 0.0){ 
      // arrived at destination?
      if(speedFac < directStopSpeed && speedFac > -directStopSpeed){ 
        //slow enough to stop?
        speedFac = 0.0;
        //Serial.print("0.0 ");
      }else{
        if(DEBUG_OUTPUT)Serial.print("panic?");
        //Serial.print("0.x ");
      }
      
      lc6++;
    }else{ 
      ///-----experimental-start
      //not crashing into something, and already driving in the right direction.
      //let's finetune the speed we are going with.
      //if(rough_speed_est < std::abs(t_current_speed_mm) && rough_speed_est != 0.0){
      
      //if(rough_speed_est != 0.0){
      
        //tuning speed in while still getting new dmx information in
        if(std::abs(t_distance_mm) < t_position_follow_error_max && std::abs(t_distance_mm) > t_position_follow_error_min){
          lc7++;
          
          if(speedFac > 0.0){
            speedFac += speedFacMultiplier;
            if(speedFac >= 1.0){
              speedFac = 1.0;
            }
          }else{
            speedFac -= speedFacMultiplier;
            if(speedFac <= -1.0){
              speedFac = -1.0;
            }
          }
        
        }else if(std::abs(t_distance_mm) > t_position_follow_error_max){
          lc8++;
          speedFacMultiplier *= (std::abs(t_distance_mm)/t_position_follow_error_max);
          if(speedFacMultiplier > max_speed_speedFacMultiplier){
            speedFacMultiplier = max_speed_speedFacMultiplier;
          }
          if(speedFac > 0.0){
            speedFac += speedFacMultiplier;
            if(speedFac >= 1.0){
              speedFac = 1.0;
            }
          }else{
            speedFac -= speedFacMultiplier;
            if(speedFac <= -1.0){
              speedFac = -1.0;
            }
          }
            
        }else if(std::abs(t_distance_mm) <= t_position_follow_error_min){
          lc9++;
          
          if(speedFac > 0.0){
            speedFac -= speedFacMultiplier;
            if(speedFac < 0.0){
              speedFac = 0.0;
            }
            if(speedFac >= 1.0){
              speedFac = 1.0;
            }
          }else{
            speedFac += speedFacMultiplier;

            if(speedFac > 0.0){
              speedFac = 0.0;
            }
            if(speedFac <= -1.0){
              speedFac = -1.0;
            }
          }
        }else{
          lc10++;
          
          //einfach nix machen
        }
      
        ///-----experimental-end
      /*}else if(speedFac > 0.0 && t_distance_mm > min_distance_to_move_for){
        //speeding up
        
        lc8++;
        speedFac += speedFacMultiplier;
        if(speedFac >= 1.0){
          speedFac = 1.0;
        }
        
        //Serial.print("2.1 ");
      }else if(speedFac < 0.0 && t_distance_mm < -min_distance_to_move_for){
        //speeding up
        
        lc9++;
        speedFac -= speedFacMultiplier;
        if(speedFac <= -1.0){
          speedFac = -1.0;
        }
        
        //Serial.print("2.2 ");
      }else{
        
        lc10++;
        //Serial.print("2.x ");
        //keep same speed(?)
      }*/
      //keep speeding up 
      //if not already 1.0
    }
  }else if(speedFac == 0.0){
    
    //standing still, start driving?
    if(t_distance_mm > 0.0 && t_distance_mm > min_distance_to_move_for){
      //forward
      speedFac = speedFacMultiplier;
      lc11++;
      
      //Serial.print("3.1 ");
    }else if(t_distance_mm < 0.0 && t_distance_mm < -min_distance_to_move_for){
      //backwards
      speedFac = -speedFacMultiplier;
      lc12++;
      
      //Serial.print("3.2 ");
    }else{
      
      lc13++;
      //Serial.print("3.x ");
    }
  }else{
    //Serial.print("x.x ");
    
    lc14++;
  }


  distanceDiff2 = distanceDiff;
  distanceDiff = t_current_pos_mm - t_target_pos_mm;
  if( (std::abs(distanceDiff) < 0.2) && controllerHasStopped && distanceDiff2 == distanceDiff){
    speedFac = 0.0;
  }

  if((int)loop_ms1_diff < TICK_LOOP_TIME_MIN && speedFac != 0.0){
    Serial.printf("id: %d, speedFac: %.6f, tick loop timer protection! to short! time: %d \n", id, speedFac, loop_ms1_diff);
    speedFac = 0.0;
  }

  if((int)loop_ms1_diff > TICK_LOOP_TIME_MAX && speedFac != 0.0){
    Serial.printf("id: %d, speedFac: %.6f, tick loop timer protection! to long! time: %d \n", id, speedFac, loop_ms1_diff);
    speedFac = 0.0;
    loop_ms1_diff = TICK_LOOP_TIME_MAX;
  }


  //if(std::abs(speedFac) <= 0.2){    
  //  adjustMicrostepping(32);
  //}else{
  //  adjustMicrostepping(8);
  //}
  handleController();
  

  if(false && everySecond() && id == 0){
  //if(everyMillis(33) && true && id == 0){
    current_rpm = ((esMaxSpeedFac * max_motor_speed_mm) / (gear_tooth_count*2.0) * 60);
    Serial.printf("[NAIL] id: %d, steps per second: %d, RPM: %.2f, Teeth: %d\n", id, esStep, current_rpm, gear_tooth_count);
  }

  t_current_speedFactor = speedFac;

  loop_ms2 = micros();
  
  loop_diff = loop_ms2-loop_ms1;

  return loop_ms1_diff;

}


bool NailStepper::everyMillis(int mil){
  mt = millis();
  if(mt2 == 0.0){
    mt2 = mt;
  }

  mt_diff = mt - mt2;
  if(mt_diff < mil){
    if(es_msf < std::abs(speedFac)){
      es_msf = std::abs(speedFac);
    }
    stp_cnt+= last_loop_steps;
    return false;
  }else{    
    mt2 = mt;
    esStep = stp_cnt;
    stp_cnt = 0;
    esMaxSpeedFac = es_msf;
    es_msf = 0.0;
    return true;
    //Serial.printf("id: %d, steps per second: %d, TSTEP: %u\n", id, stp_cnt, tstp);

    //print_chopconf();
    //print_pwmconf();
  
    //Driver Status output, might be pushing the loop limit,...
    //TMC2130_n::DRV_STATUS_t drv_status{0};
    //drv_status.sr = DRV_STATUS();
    //printDriverStatus(drv_status);


  }
}

bool NailStepper::everySecond(){
  mt = millis();
  if(mt2 == 0.0){
    mt2 = mt;
  }

  mt_diff = mt - mt2;
  if(mt_diff < 1000.0){
    if(es_msf < std::abs(speedFac)){
      es_msf = std::abs(speedFac);
    }
    stp_cnt+= last_loop_steps;
    return false;
  }else{    
    mt2 = mt;
    esStep = stp_cnt;
    stp_cnt = 0;
    esMaxSpeedFac = es_msf;
    es_msf = 0.0;
    return true;
    //Serial.printf("id: %d, steps per second: %d, TSTEP: %u\n", id, stp_cnt, tstp);

    //print_chopconf();
    //print_pwmconf();
  
    //Driver Status output, might be pushing the loop limit,...
    //TMC2130_n::DRV_STATUS_t drv_status{0};
    //drv_status.sr = DRV_STATUS();
    //printDriverStatus(drv_status);


  }
}

void NailStepper::updateController(){
  if(speedFac != 0.0){
    controller->overrideSpeed(speedFac);             // set new speed
    //Serial.printf("%d SF: %.6f \n", id, speedFac);
    controllerHasStopped = false;
    if(!controllerHasStarted){
      startController();
    }
  }else if(!controllerIsStopping && !controllerHasStopped && controller->isRunning()){
    stopController();
  }
}

void NailStepper::handleController(){
  if(controllerIsStopping){
    evalStopController();    
  }else{    
    updateController();
  }
}

void NailStepper::startController(){
  if(!controller->isRunning()){
    controllerHasStarted = true;
    controller->rotateAsync(stepper);	  
    if(DEBUG_OUTPUT)Serial.printf("Go %d SpeedFac:%.6f DistDiff:%.6f \n", id, speedFac, distanceDiff);
  }
}

void NailStepper::stopController(){
  if(DEBUG_OUTPUT)Serial.printf("Stopping: %d Dist Diff: %.4f \n", id, distanceDiff);
  controller->directStop();
  //controller->stopAsync();
  controllerIsStopping = true;
  controllerHasStopped = false;
}

void NailStepper::evalStopController(){
  if(controller->isRunning()){
    if(DEBUG_OUTPUT)Serial.printf("Still Stopping %d \n", id);
  }else{
    //delay(1);
    controllerIsStopping = false;
    controllerHasStopped = true;
    controllerHasStarted = false;
    if(DEBUG_OUTPUT)Serial.printf("Stopped %d \n", id);
  
    //Serial.printf("Id: %d LOST STEPS:%d\n", id, LOST_STEPS());
    //Driver Status output, might be pushing the loop limit,...
    //TMC2130_n::DRV_STATUS_t drv_status{0};
    //drv_status.sr = DRV_STATUS();
    //printDriverStatus(drv_status);
  }
}

void NailStepper::print_chopconf(){
  
    uint32_t tmpC = CHOPCONF();
    Serial.printf("id: %d, CHOPCONF(): %s\n",id, byte32_to_binary(tmpC));
}

void NailStepper::print_pwmconf(){
  
    uint32_t tmpC = PWMCONF();
    Serial.printf("id: %d, PWMCONF(): %s\n",id, byte32_to_binary(tmpC));
}

//RAIN Tick
unsigned long NailStepper::tick_continues_dir(){
  
  loop_ms1_old = loop_ms1;
  loop_ms1 = micros();
  loop_ms1_diff = loop_ms1-loop_ms1_old;

  //t_current_pos_mm = steps_to_mm( stepper.getPosition());

  t_current_pos_step_old = t_current_pos_step;
  t_current_pos_step = stepper.getPosition();
  last_loop_steps = t_current_pos_step - t_current_pos_step_old;
  avg_step_time = loop_ms1_diff / last_loop_steps;

  //t_target_speed_mm = 

  t_current_speedFactor = speedFac;
  //watch out - dmx_step_size is now in mm not steps
  if(target_pos > 15000){
    target_pos = 15000;
  }

  speedFac = target_pos * dmx_speed_factor;

  double accFac = 1.0005;

  if(speedFac > (t_current_speedFactor * accFac)){
    if(t_current_speedFactor >= 0.005){
      speedFac = t_current_speedFactor * accFac;
    }else{
      speedFac = 0.005;
    }
  }else if(speedFac < (t_current_speedFactor / accFac) && t_current_speedFactor >= 0.005){
    speedFac = t_current_speedFactor / accFac;
  }


  float revDist = 40.0; // full rev distance (fixed gear 20t 2mm pitch) 
  float sec = 60.0; //seconds in minute
  current_rpm = ((speedFac * max_motor_speed_mm) /revDist * sec);
  

  mt = millis();
  if(mt2 == 0.0){
    mt2 = mt;
  }

  mt_diff = mt - mt2;
  if(mt_diff < 1000.0){
    stp_cnt+= last_loop_steps;
  }else{
    mt2 = mt;
    //Serial.printf("id: %d, steps per second: %d, TSTEP: %u\n", id, stp_cnt, tstp);
    
    if(DEBUG_OUTPUT)Serial.printf("[CONTINUES] id: %d, steps per second: %d, RPM: %.4f\n", id, stp_cnt, current_rpm);

    //print_chopconf();
    //print_pwmconf();
  
    //Driver Status output, might be pushing the loop limit,...
    //TMC2130_n::DRV_STATUS_t drv_status{0};
    //drv_status.sr = DRV_STATUS();
    //printDriverStatus(drv_status);


    stp_cnt = 0;
  }


  if((int)loop_ms1_diff < TICK_LOOP_TIME_MIN && speedFac != 0.0){
    Serial.printf("id: %d, speedFac: %.6f, tick loop timer protection! to short! time: %d \n", id, speedFac, loop_ms1_diff);
    speedFac = 0.0;
  }

  if((int)loop_ms1_diff > TICK_LOOP_TIME_MAX && speedFac != 0.0){
    Serial.printf("id: %d, speedFac: %.6f, tick loop timer protection! to long! time: %d \n", id, speedFac, loop_ms1_diff);
    speedFac = 0.0;
    loop_ms1_diff = TICK_LOOP_TIME_MAX;
  }
  

  //calibrate / get f_clk
  if(1 == 0 && id == 0 && target_pos == 1){
    speedFac = t_current_speedFactor;
    //Serial.print(".");

    if(speedFac == 0.00){
      //this is 1000steps/sec at 16 microsteps and 20teeth
      //speedFac = 0.015625;


      //speedFac = 0.053510274;
      speedFac = 0.0033444;
      //!!! switcht ms to 256 watch out
      //this shoud be 1000steps/sec at 256microsteps 20teeth
      //speedFac = 0.0009765625;
    }
    /*else if(speedFac == 0.01){
      speedFac = 0.02;
    }else if(speedFac == 0.02){
      speedFac = 0.03;
    }else if(speedFac == 0.03){
      speedFac = 0.04;
    }else if(speedFac == 0.04){
      speedFac = 0.05;
    }else if(speedFac == 0.05){
      speedFac = 0.06;
    }else if(speedFac == 0.06){
      speedFac = 0.07;
    }else if(speedFac == 0.07){
      speedFac = 0.08;
    }*/

    //speedFac = 0.08;
    tstp = TSTEP();

    //Serial.printf("id: %d, TSTEP: %u, pos: %d, pos_old: %d, loopTime: %u \n", id, tstp, t_current_pos_step, t_current_pos_step_old, loop_ms1_diff);
    //Serial.printf("id: %d, pos: %d, pos_old: %d, loopTime: %u \n", id, t_current_pos_step, t_current_pos_step_old, loop_ms1_diff);
  }

  handleController();

  return loop_ms1_diff;
}

unsigned long NailStepper::tick_motion_gen(){

  loop_ms1_old = loop_ms1;
  loop_ms1 = micros();
  loop_ms1_diff = loop_ms1-loop_ms1_old;

  //t_current_pos_mm = steps_to_mm( stepper.getPosition());

  t_current_pos_step_old = t_current_pos_step;
  t_current_pos_step = stepper.getPosition();
  t_current_pos_mm = steps_to_mm(t_current_pos_step);
  last_loop_steps = t_current_pos_step - t_current_pos_step_old;
  avg_step_time = loop_ms1_diff / last_loop_steps;

  //t_target_speed_mm = 
  t_target_pos_mm = target_pos * dmx_step_size;

  //Serial.println("going to update!");
  //delay(2000);
  //Serial.println("going to update!");
  //delay(2000);
  //Serial.println("going to update!");
  //delay(2000);
  //Serial.println("going to update!");
  trapezoidalProfile->setInitPosition(t_current_pos_mm);
  
  //double mg_pos = trapezoidalProfile->update(t_target_pos_mm);
  
  double mg_vel = trapezoidalProfile->getVelocity();
  
  //Serial.print(mg_pos);
  //Serial.print(" ");
  //Serial.println(mg_vel);
  speedFac = (1.0/max_motor_speed_mm) * mg_vel;

  if(speedFac > 1.0){
    speedFac = 1.0;
  }

  //watch out - dmx_step_size is now in mm not steps
  //speedFac = target_pos * dmx_speed_factor; 
  

  current_rpm = ((speedFac * max_motor_speed_mm) / 40.0 * 60.0);
  

  //t_target_pos_mm_buffer = target_pos_buffer * dmx_step_size;
  //t_target_pos_mm_buffer_diff = t_target_pos_mm - t_target_pos_mm_buffer;
  
  //t_distance_mm = t_target_pos_mm - t_current_pos_mm; 
  
  //t_current_speed_mm = t_current_speedFactor * motor_speed_mm;
  //-------------- OLD, but good enough --------------------
  //-------------- NEW -------------------------------------

  //speedFac = th.tick(t_current_speed_mm, t_current_pos_mm, t_target_pos_mm, t_target_pos_mm_buffer, (ts_incoming_buffer-ts_incoming), loop_ms1_diff, target_position_updated_counter);

  //-------------- NEW -------------------------------------  
  //-------------- OLD, but good enough --------------------

  if((int)loop_ms1_diff < tick_loop_time_min){
    if(DEBUG_OUTPUT)Serial.print("tick loop timer protection! to short! time:");
    if(DEBUG_OUTPUT)Serial.println(loop_ms1_diff);
    speedFac = 0.0;
  }

  if((int)loop_ms1_diff > tick_loop_time_max){
    if(DEBUG_OUTPUT)Serial.print("tick loop timer protection! to long! time:");
    if(DEBUG_OUTPUT)Serial.println(loop_ms1_diff);
    speedFac = 0.0;
    loop_ms1_diff = tick_loop_time_max;
  }
  t_current_speedFactor = speedFac;

  controller->overrideSpeed(speedFac);             // set new speed

  if(!controller->isRunning() && speedFac != 0.0){
    controller->rotateAsync(stepper);	  
  }

  return loop_ms1_diff;
}

unsigned long NailStepper::tick(){
  loop_ms1_old = loop_ms1;
  loop_ms1 = micros();
  loop_ms1_diff = loop_ms1-loop_ms1_old;

  t_current_pos_mm = steps_to_mm( stepper.getPosition());

  //watch out - dmx_step_size is now in mm not steps
  t_target_pos_mm = target_pos * dmx_step_size;

  t_target_pos_mm_buffer = target_pos_buffer * dmx_step_size;
  t_target_pos_mm_buffer_diff = t_target_pos_mm - t_target_pos_mm_buffer;
  
  t_distance_mm = t_target_pos_mm - t_current_pos_mm; 
  
  t_current_speed_mm = t_current_speedFactor * motor_speed_mm;
  //-------------- OLD, but good enough --------------------
  //-------------- NEW -------------------------------------

  speedFac = th.tick(t_current_speed_mm, t_current_pos_mm, t_target_pos_mm, t_target_pos_mm_buffer, (ts_incoming_buffer-ts_incoming), loop_ms1_diff, target_position_updated_counter);

  //-------------- NEW -------------------------------------  
  //-------------- OLD, but good enough --------------------

  if((int)loop_ms1_diff < tick_loop_time_min){
    if(DEBUG_OUTPUT)Serial.print("tick loop timer protection! to short! time:");
    if(DEBUG_OUTPUT)Serial.println(loop_ms1_diff);
    speedFac = 0.0;
  }

  if((int)loop_ms1_diff > tick_loop_time_max){
    if(DEBUG_OUTPUT)Serial.print("tick loop timer protection! to long! time:");
    if(DEBUG_OUTPUT)Serial.println(loop_ms1_diff);
    speedFac = 0.0;
    loop_ms1_diff = tick_loop_time_max;
  }
  t_current_speedFactor = speedFac;

  controller->overrideSpeed(speedFac);             // set new speed

  if(!controller->isRunning() && speedFac != 0.0){
    controller->rotateAsync(stepper);	  
  }

  return loop_ms1_diff;
}

void NailStepper::adjustMicrostepping(int ms_new){
  if(ms_new != run_microsteps){
    double fx = ms_new / run_microsteps;
    stepper.setPosition(int32_t((double)stepper.getPosition()*fx));
    microsteps(ms_new);
    
    run_microsteps = ms_new;
  }
}

double NailStepper::calculate_dmx_speed_ratio(){
  return 1.0/get_dmx_resolution();
}

double NailStepper::get_dmx_resolution(){
    double step_devider = 256.0;
    for(int i=1;i<artnet_resolution;i++){
      step_devider *= 256.0;
    }
    step_devider -= 1;    

    return step_devider;
}

void NailStepper::calculateSizesForMicrostepping(int ms){

    max_pos = getAvailableStepsForFullDistance(run_microsteps);
    
    
    //dmx_step_size = (double)max_pos/step_devider; //should it be 256 or 255? ,...hmmm
    dmx_step_size = (double)full_distance/get_dmx_resolution(); //should it be 256 or 255? ,...hmmm
    //delay(5000);
    if(DEBUG_OUTPUT)Serial.print("MaxPos: ");
    if(DEBUG_OUTPUT)Serial.println(max_pos);
    if(DEBUG_OUTPUT)Serial.print("StepSize: ");
    if(DEBUG_OUTPUT)Serial.println(dmx_step_size);
    //delay(5000);
}

float NailStepper::getRPM(){
	int step_freq = init_hzbase/init_timerSpeed;
	float rps = float(step_freq)/float(total_steps_per_revolution)/2;
	int _rpm = rps * 6000;
	float rpm = _rpm/100.0;

	return rpm;
};

void NailStepper::setDistance(int gear_count, int belt_pit = 2, int fs_per_rev = 200 ){
  gear_tooth_count = gear_count;
  belt_pitch = belt_pit;
  fullsteps_per_revolution = fs_per_rev;

  one_revolution_distance = gear_tooth_count * belt_pitch;
  total_steps_per_revolution = fullsteps_per_revolution * init_microsteps;    
  one_step_distance_in_mm = (long double) one_revolution_distance / (long double) total_steps_per_revolution;
} 

void NailStepper::setState(int s){
  state = s;
}


void NailStepper::scanInit_old(){
  begin(); 
  toff(4);
  blank_time(24);
  rms_current(500); // mA 400
  microsteps(init_microsteps);
  TCOOLTHRS(0xFFFFF); // 20bit max
  THIGH(0);
  semin(5);
  semax(2);
  sedn(0b01);
  sgt(STALL_VALUE); 
  delay(500); // just to be sure giving the spi same time to digest stuff
}

void NailStepper::scanInit(){
  if(driver_init == DRIVER_INIT_MINIMAL){ //66
    
    begin();             // Initiate pins and registeries
    toff(5);
    blank_time(36);
    rms_current(RMS_MINIMAL, 0.4);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    //en_pwm_mode(1);      // Enable extremely quiet stepping
    //pwm_autoscale(1);
    microsteps(run_microsteps);
  }else if(scan_init_old){ //33
    scanInit_old();
  }else if(driver_init == DRIVER_INIT_CONF){ //111
    tmc_driver_conf_init *c = new tmc_driver_conf_init;

    init_microsteps = NS_INIT_MICROSTEPS;
    //init_microsteps = 32;
    begin(); 
    toff(c->toff);
    blank_time(c->blank_time);
    rms_current(c->rms_current, c->rms_idle); // mA 400
    microsteps(init_microsteps);
    TCOOLTHRS(c->tcoolthrs);   // with TCOOLTHRS = max AND THIGH = 0 we are making sure COOLSTEP is ENABLED
    THIGH(c->thigh);             // with TCOOLTHRS = max AND THIGH = 0 we are making sure COOLSTEP is ENABLED
    semin(c->semin);       // 1-15 | 0 = COOLSTEP is DISABLED | if stallguard2 res <   semin * 32         = motor current INcreases to reduce step angle
    semax(c->semax);       // 0-15 |                            if stallguard2 res >= (SEMIN+SEMAX+1)*32  = motor current DEcreases to safe energy
    sedn(c->sedn);     // 0-3  | current down step speed  | 0 = each 32 sg2 values decrease by one | 1 = each 8 sg2 values decrease by one | 2 = each 2 sg2 values decrease by one | 3 = each 1 sg2 values decrease by one | 
    sgt(c->sgt);
  }else{ //DRIVER_INIT_NORMAL 99

    init_microsteps = NS_INIT_MICROSTEPS;
    //init_microsteps = 32;
    begin(); 
    toff(5);
    blank_time(36);
    rms_current(500, 0.4); // mA 400
    microsteps(init_microsteps);
    TCOOLTHRS(0xFFFFF);   // with TCOOLTHRS = max AND THIGH = 0 we are making sure COOLSTEP is ENABLED
    THIGH(0);             // with TCOOLTHRS = max AND THIGH = 0 we are making sure COOLSTEP is ENABLED
    semin(5);       // 1-15 | 0 = COOLSTEP is DISABLED | if stallguard2 res <   semin * 32         = motor current INcreases to reduce step angle
    semax(2);       // 0-15 |                            if stallguard2 res >= (SEMIN+SEMAX+1)*32  = motor current DEcreases to safe energy
    sedn(0b01);     // 0-3  | current down step speed  | 0 = each 32 sg2 values decrease by one | 1 = each 8 sg2 values decrease by one | 2 = each 2 sg2 values decrease by one | 3 = each 1 sg2 values decrease by one | 
    sgt(3); //TMC_STALL_VALUE); // -64 - +63 | lower value = higher sensitivity
    
    /* Old steppers:
    begin(); 
    toff(4);
    blank_time(24);
    rms_current(500); // mA 400
    microsteps(init_microsteps);
    TCOOLTHRS(0xFFFFF); // 20bit max
    THIGH(0);
    semin(5);
    semax(2);
    sedn(0b01);
    sgt(TMC_STALL_VALUE); 
    */
    delay(500); // just to be sure giving the spi same time to digest stuff
  }
}

void NailStepper::offsetEndstops(int mm, int dir){
  if(scanType == SCAN_TYPE_FULL_SCAN){
    full_distance = full_distance - ((double)mm*2.0);
  }
  if(mm != 0.0){
    move_distance_mm(mm, dir);
  }

}

int NailStepper::getMs(){
  if(state == NS_STATE_INIT){
    return init_microsteps;
  }else if(state == NS_STATE_RUN){
    return run_microsteps;
  }else{
    return 0;
  }
}

int NailStepper::getStepsPerRevolution(){
  return fullsteps_per_revolution*getMs();
}

void NailStepper::changeMicrosteps(int ms){
  microsteps(ms);
}

int NailStepper::mm_to_steps(double mm){
  double steps_per_mm = getStepsPerRevolution()/one_revolution_distance;
  return (int)(steps_per_mm * mm);
// motor_speed = 800 (in mm/sec)
// one_rev_mm = gear_teeth * pitch
// steps_one_rev = fstep * ms
// mm_per_steps = one_rev_mm/steps_on_rev
// steps_per_mm = steps_on_rev/one_rev_mm
// motor_speed_steps = steps_per_mm * motor_speed_mm
}

double NailStepper::steps_to_mm(int st){
  double mm_per_steps = (double)one_revolution_distance/(double)getStepsPerRevolution();
  return mm_per_steps*(double)st;
}

int NailStepper::getAvailableStepsForFullDistance(int ms){
  
  one_revolution_distance = gear_tooth_count * belt_pitch;
  total_steps_per_revolution = fullsteps_per_revolution * ms;    

  // full_distance is in mm

  //one_step_distance_in_mm = (double) one_revolution_distance / (double) total_steps_per_revolution;
  int one_mm_contains_steps = total_steps_per_revolution / one_revolution_distance;
  double full_distance_steps = (double)one_mm_contains_steps * full_distance;
  //Serial.print("OneStep_GetFull:");
  //Serial.println(double(one_step_distance_in_mm*100.0));
  //double full_distance_steps = full_distance/one_step_distance_in_mm;
  return  int(full_distance_steps);
}

void NailStepper::dmxSettingsUpdate(){
  //Serial.println("dmxSettingsUpdate!");
  if(dmx->settings_active_always){
    setInvert();
    setGear();
    setFullDistance();
    setDriver();
  }
}

int NailStepper::getStepperSetting(const char* option0,const char* option1,const char* option2,const char* option3){

  int set=0;
  switch(this->id){
    case 0:
      set = dmx->settings_get(option0);
      break;
    case 1:
      set = dmx->settings_get(option1);
      break;
    case 2:
      set = dmx->settings_get(option2);
      break;
    case 3:
      set = dmx->settings_get(option3);
      break;
  }

  return set;
}

int NailStepper::setMode(){
  int tmp = dmx->settings_get(DMX_SETTING_DRIVE_TYPE);
  //bool validInput = false;

  if(tmp == MODE_RAIN){
    if(DEBUG_OUTPUT)Serial.printf("[DMX-Setting] [MODE] [RAIN] for complete Controller is now %d \n", MODE_NAIL);    
    return MODE_RAIN;

  }else{
    if(DEBUG_OUTPUT)Serial.printf("[DMX-Setting] [MODE] [NAIL] for complete Controller is now %d \n", MODE_NAIL);    
    return MODE_NAIL;

  }
}

void NailStepper::setDriver(){
  int tmp = dmx->settings_get(DMX_SETTING_TMC_SETTINGS);
  bool validInput = false;

  if(tmp != 0 && tmp != driver_dmx_setting){
    driver_dmx_setting = tmp;
    if(driver_dmx_setting == DMX_SETTING_TMC_MINIMAL){ //66
      driver_init = DRIVER_INIT_MINIMAL;
      scan_init_old = 0;
      finish_init_old = 0;
      validInput = true;
    }else if(driver_dmx_setting == DMX_SETTING_TMC_OLD){ //33
      driver_init = DRIVER_INIT_NORMAL;
      scan_init_old = 1;
      finish_init_old = 1;
      validInput = true;
    }else if(driver_dmx_setting == DMX_SETTING_TMC_NEW){ //99
      driver_init = DRIVER_INIT_NORMAL;
      scan_init_old = 0;
      finish_init_old = 0;
      validInput = true;
    }else if(driver_dmx_setting == DMX_SETTING_TMC_CONF){ //111
      driver_init = DRIVER_INIT_CONF;
      scan_init_old = 0;
      finish_init_old = 0;
      validInput = true;
    }
  }

  if(validInput){
    if(state == NS_STATE_INIT){
      if(DEBUG_OUTPUT)Serial.printf("[DMX-Setting] [DRIVER] [INIT] for Stepper: %d is now %d \n", this->id, driver_dmx_setting);    
      //delay(1000);
      //scanInit();
      //delay(1000);
      if(DEBUG_OUTPUT)Serial.printf("[DMX-Setting] [DRIVER] [INIT] - Done \n");

    }else if(state == NS_STATE_RUN){
      delay(200);      
      uint32_t tmp_gstat1 = GSTAT();
      delay(200);      
      bool tmp_reset = reset();
      delay(200);
      uint32_t tmp_gstat2 = GSTAT();

      if(DEBUG_OUTPUT)Serial.printf("[DMX-Setting] [DRIVER] [RUN] Stepper: %d, GSTAT1 was: %s, result reset: %d \n", this->id, byte_to_binary(tmp_gstat1), tmp_reset);   
      if(DEBUG_OUTPUT)Serial.printf("[DMX-Setting] [DRIVER] [RUN] Stepper: %d, GSTAT2 was: %s, result reset: %d \n", this->id, byte_to_binary(tmp_gstat2), tmp_reset);   
      if(DEBUG_OUTPUT)Serial.printf("[DMX-Setting] [DRIVER] [RUN] for Stepper: %d is now %d \n", this->id, driver_dmx_setting);   
      delay(100);
      scanInit();
      delay(100);
      finishInit();
      delay(100);
      if(DEBUG_OUTPUT)Serial.printf("[DMX-Setting] [DRIVER] [RUN] - Done \n");
    }
  }
}

void NailStepper::setFullDistance(){
  
  int set = getStepperSetting(
    DMX_SETTING_TRAVEL_DISTANCE_ADJUST_0,
    DMX_SETTING_TRAVEL_DISTANCE_ADJUST_1,
    DMX_SETTING_TRAVEL_DISTANCE_ADJUST_2,
    DMX_SETTING_TRAVEL_DISTANCE_ADJUST_3
  );

  if(set != full_distance && set > 0){
    if(DEBUG_OUTPUT)Serial.printf("[DMX-Setting] [DISTANCE] for Stepper: %d is now %f got new:%d \n", this->id, full_distance, set);    
    full_distance = set;

    //setDistance(gear_tooth_count, belt_pitch, fullsteps_per_revolution);
    
    if(state == NS_STATE_RUN){
      calculateSizesForMicrostepping(run_microsteps);
    }
  }
}

void NailStepper::setGear(){
  
  int set = getStepperSetting(
    DMX_SETTING_GEAR_COUNT_0,
    DMX_SETTING_GEAR_COUNT_1,
    DMX_SETTING_GEAR_COUNT_2,
    DMX_SETTING_GEAR_COUNT_3
  );

  if(set != gear_tooth_count && set > 0){
    if(DEBUG_OUTPUT)Serial.printf("[DMX-Setting] [GEAR] for Stepper: %d is now %d got new:%d \n", this->id, gear_tooth_count, set);    
    gear_tooth_count = set;

    setDistance(gear_tooth_count, belt_pitch, fullsteps_per_revolution);
    
    if(state == NS_STATE_RUN){
      calculateSizesForMicrostepping(run_microsteps);
    }
  }
}

void NailStepper::setInvert(){

  int set = getStepperSetting(
    DMX_SETTING_INVERT_DIR_0,
    DMX_SETTING_INVERT_DIR_1,
    DMX_SETTING_INVERT_DIR_2,
    DMX_SETTING_INVERT_DIR_3
  );
    

  if(set > 0 && stepperInvert == 0){
    stepperInvert = 1;
    stepper.setInverseRotation(stepperInvert);
    if(DEBUG_OUTPUT)Serial.printf("[DMX-Setting] [DIRECTION] for Stepper: %d is now %d got set:%d \n", this->id, stepperInvert, set);
  }else if(set == 0 && stepperInvert == 1){
    stepperInvert = 0;
    stepper.setInverseRotation(stepperInvert);
    if(DEBUG_OUTPUT)Serial.printf("[DMX-Setting] [DIRECTION] for Stepper: %d is now %d got set:%d \n", this->id, stepperInvert, set);
  }
  
}

void NailStepper::prepareRun(double m_sp_mm, double m_acc_mm, double m_pull_sp_mm, int invertDir){
    motor_speed_mm = m_sp_mm;
    motor_speed_steps = mm_to_steps(m_sp_mm);

    motor_acc_mm = m_acc_mm;    
    motor_acc_steps = mm_to_steps(m_acc_mm);
    t_acc_mm = motor_acc_mm;
    t_acc_step = motor_acc_steps;

    motor_pullin_speed_mm = m_pull_sp_mm;
    motor_pullin_speed_steps = mm_to_steps(m_pull_sp_mm);

    stepper.setPosition(0);
    
    //DMX settings activated, then take that. otherwise use config settings
    if(dmx->settings_active_init || dmx->settings_active_always){
      setInvert(); //Dmx Invert Setting
    }else{
      stepper.setInverseRotation(invertDir);
    }
    
		stepper.setAcceleration(motor_acc_steps);
		stepper.setMaxSpeed(motor_speed_steps);	
		stepper.setPullInSpeed(motor_pullin_speed_steps);

		controller->rotateAsync(stepper);
		controller->overrideSpeed(0); // start with stopped slide
		stepper.setPosition(0);
}

// OLD!!!!
void NailStepper::handle_dmx_settings(double devider = 1.0){
    //Serial.print("TSTEP:");
    //Serial.println(TSTEP());
    

    for(int i=0;i<dmx_settings_count;i++){
      if(dmx_settings_channels_old[i][1] != dmx_settings_channels[i][1]){
        //setting has updated -> so update functions
        //10, 11, 12, 13
        double value = (double)dmx_settings_channels[i][1];
        value = value / 255.0;
        double max = 0.0;
        switch(dmx_settings_channels[i][0]){
          //pwm_freq()
          case 9:
            // 1 - 255
            max = 255.0;
            value *= max;
            Serial.print("Nail:");
            Serial.print(id);
            Serial.print(" - Old - pwm_ampl(): ");
            Serial.print(pwm_ampl());
            
            pwm_ampl((u_int8_t)value);
            
            Serial.print(" - New - pwm_ampl(): ");
            Serial.print(pwm_ampl());
            Serial.print(" value:");
            Serial.println(value);
            break;
          case 10:
            // 1 - 15
            max = 15.0;
            value *= max;
            Serial.print("Nail:");
            Serial.print(id);
            Serial.print(" - Old - pwm_grad(): ");
            Serial.print(pwm_grad());
            
            pwm_grad((u_int8_t)value);
            
            Serial.print(" - New - pwm_grad(): ");
            Serial.print(pwm_grad());
            Serial.print(" value:");
            Serial.println(value);
            break;
          case 11:
            max = 1048575;
            value *= max;
            Serial.print("Nail:");
            Serial.print(id);
            Serial.print(" - Old - TPWMTHRS: ");
            Serial.print(TPWMTHRS());
            
            TPWMTHRS((u_int32_t)value);
            
            Serial.print(" - New - TPWMTHRS: ");
            Serial.print(TPWMTHRS());
            Serial.print(" value:");
            Serial.println(value);
            break;
          case 12:
            max = 1048575;
            value *= max;
            Serial.print("Nail:");
            Serial.print(id);
            Serial.print(" - Old - TCOOLTHRS: ");
            Serial.print(TCOOLTHRS());
            
            //TCOOLTHRS() //1048575
            TCOOLTHRS((u_int32_t)value);
            
            Serial.print(" - New - TCOOLTHRS: ");
            Serial.print(TCOOLTHRS());
            Serial.print(" value:");
            Serial.println(value);
            break;
          case 13:
           max = 1048575;
            value *= max;
            Serial.print("Nail:");
            Serial.print(id);
            Serial.print(" - Old - THIGH: ");
            Serial.print(THIGH());
            
            //TCOOLTHRS() //1048575
            THIGH((u_int32_t)value);
            
            Serial.print(" - New - THIGH: ");
            Serial.print(THIGH());
            Serial.print(" value:");
            Serial.println(value);
            break;
          case 14:
    hstrt(8);// 4 - 7(exl)
    hend(4); // 0 - 3(exl)

            max = 8;
            value *= max;
            Serial.print("Nail:");
            Serial.print(id);
            Serial.print(" - Old - hstrt: ");
            Serial.print(hstrt());
            
            //ffective HEND+HSTRT ≤16
            hstrt((u_int8_t)value);
            
            Serial.print(" - New - hstrt: ");
            Serial.print(hstrt());
            Serial.print(" value:");
            Serial.println((u_int8_t)value);
            break;
          case 15:
            max = 16;
            value *= max;
            value -= 3;
            Serial.print("Nail:");
            Serial.print(id);
            Serial.print(" - Old - hend: ");
            Serial.print(hend());
            
            //TCOOLTHRS() //1048575
            hend((u_int8_t)value);
            
            Serial.print(" - New - hend: ");
            Serial.print(hend());
            Serial.print(" value:");
            Serial.println((u_int8_t)value);
            break;
          default:
            Serial.println("Unknown DMX-Setting");            
        }
      }
    }
}

void NailStepper::finishInit_old(){
    setState(NS_STATE_RUN);
    
    //Init settings ------------
    begin(); 
    //toff(4);
    //blank_time(24);
    //rms_current(500); // mA 400
    //microsteps(init_microsteps);
    //TCOOLTHRS(0xFFFFF); // 20bit max
    //THIGH(0);
    //semin(5);
    //semax(2);
    //sedn(0b01);
    //sgt(STALL_VALUE); 
    ///-----------init config
    delay(100);

    rms_current(800);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    
    en_pwm_mode(1);      // Enable extremely quiet stepping
    pwm_autoscale(1);
    //pwm_ampl();
    //pwm_grad();
    //pwm_freq(); // default is 1   - was 2 most of sunday
        //delay(5000);
    //run_microsteps = 32;
    changeMicrosteps(run_microsteps);
    toff(2);
    THIGH(0); // guessing

    if(DEBUG_OUTPUT)Serial.print("PWM_FREQ: ");
    if(DEBUG_OUTPUT)Serial.println(pwm_freq());
    
    //min 1 - max 15 - standard 4
    //8 - higher pitch sound
    if(DEBUG_OUTPUT)Serial.print("PWM_GRAD: ");
    if(DEBUG_OUTPUT)Serial.println(pwm_grad());
    
    //min 64 - max 255 - standard - 128
    if(DEBUG_OUTPUT)Serial.print("PWM_AMP: ");
    if(DEBUG_OUTPUT)Serial.println(pwm_ampl());

    /*
    Specifies  the  upper  velocity  for  operation  in stealthChop  voltage  PWM  mode.  
    Entry  the TSTEP reading  (time  between  two  microsteps)  when operating at the desired threshold velocity.
    0 ... 1048575
    */
    // default atm is 0
    //TPWMTHRS(1048575); //no noticebal diff

    //TPWMTHRS(1000);
    if(DEBUG_OUTPUT)Serial.print("TPWMTHRS: ");
    if(DEBUG_OUTPUT)Serial.println(TPWMTHRS());

    //default atm is 1048575 (hex 0xFFFFF)
    //100 - 1048575
    //50  - 524287
    //25  - 262143
    // 12 - 131071
    // 1  - 10500
    // 0.1- 1000

    //TCOOLTHRS 131071 & THIGH 1000 seems to kinda work @32ms for 20t gears but not for 16t
    //TCOOLTHRS(10500);
    if(DEBUG_OUTPUT)Serial.print("TCOOLTHRS: ");
    if(DEBUG_OUTPUT)Serial.println(TCOOLTHRS());

    //default atm is 0 or 32 depends how you see it
    //THIGH(1000);
    if(DEBUG_OUTPUT)Serial.print("THIGH: ");
    if(DEBUG_OUTPUT)Serial.println(THIGH());

    if(DEBUG_OUTPUT)Serial.print("IRUN:");
    if(DEBUG_OUTPUT)Serial.print(irun());
    if(DEBUG_OUTPUT)Serial.print(" IHOLD:");
    if(DEBUG_OUTPUT)Serial.print(ihold());
    if(DEBUG_OUTPUT)Serial.print(" IHOLD_DELAY:");
    if(DEBUG_OUTPUT)Serial.println(iholddelay());
    if(DEBUG_OUTPUT)Serial.print("v_sense:");
    if(DEBUG_OUTPUT)Serial.println(vsense());
    if(DEBUG_OUTPUT)Serial.print("tbl:");
    if(DEBUG_OUTPUT)Serial.print(tbl());
    if(DEBUG_OUTPUT)Serial.print(" toff:");
    if(DEBUG_OUTPUT)Serial.print(toff());
    if(DEBUG_OUTPUT)Serial.print(" HSTRT:");
    if(DEBUG_OUTPUT)Serial.print(hstrt());
    if(DEBUG_OUTPUT)Serial.print(" HEND:");
    if(DEBUG_OUTPUT)Serial.println(hend());
    if(DEBUG_OUTPUT)Serial.print(" chm:");
    if(DEBUG_OUTPUT)Serial.println(chm());
    if(DEBUG_OUTPUT)Serial.print(" intpol:");
    if(DEBUG_OUTPUT)Serial.println(intpol());
    if(DEBUG_OUTPUT)Serial.print(" small_hys:");
    if(DEBUG_OUTPUT)Serial.println(small_hysteresis());
    //small_hysteresis(1);
    //Serial.print(" small_hys:");
    //erial.println(small_hysteresis());

    ihold(17); //should be about 70% of irun
    iholddelay(2); // 1 - 15 so 8 is middle_ish
    TPOWERDOWN(64); //roughing it
    
    //overshooting before TPWMTHRS? Increase this (max 15)
    pwm_grad(1); // was 4 but 1 recommended to start with

    //overshooting past TPWMTHRS? Set this lower (keep above ~50)
    pwm_ampl(255); // 255 recommended to start with
    pwm_freq(0); // was 1 but 0 seems better freq * i think
    //pwm_ampl();
            // Recommended start values
    toff(4); // 4 - 5(exl)
    tbl(2);  // 2 - 2(exl)
    hstrt(4);// 4 - 7(exl)
    hend(0); // 0 - 3(exl)

//TPWMTHRS: 0
//TCOOLTHRS: 1048575
//THIGH: 0
//TCOOLTHRS ≥ TSTEP ≥ THIGH = coolStep is enabled, if configured & stealthChopvoltage PWM    mode    is disabled!!!!
    TCOOLTHRS(0); // was 
    
    //80 seems like goodish value, but doesnt fix the issue
    TPWMTHRS(0); //was zero, worked with 16ms but now with 32ms not anymore
    
    //TSTEP(); //at 800mm/sec setting fastest speed for nail is TSTEP:9 @ ms 8

    

    if(DEBUG_OUTPUT)Serial.print(" GCONF:"); //100 means everything left of 1 is 0 (yes that is correct!)
    u_int32_t tmp = GCONF();
    if(DEBUG_OUTPUT)Serial.println(tmp, BIN); //100?
    tmp = tmp >> 1;
    if(DEBUG_OUTPUT)Serial.println(tmp, BIN); //100?
    tmp = tmp >> 1;
    if(DEBUG_OUTPUT)Serial.println(tmp, BIN); //100?
    
    if(DEBUG_OUTPUT)Serial.println();
    
    if(DEBUG_OUTPUT)Serial.println();
    if(DEBUG_OUTPUT)Serial.println();

    delay(100);

    
    calculateSizesForMicrostepping(run_microsteps);
}

void NailStepper::finishInit(){
  dmx_speed_factor = calculate_dmx_speed_ratio();
  if(driver_init == DRIVER_INIT_MINIMAL){ //66
    
    setState(NS_STATE_RUN);
    calculateSizesForMicrostepping(run_microsteps);
    return;
  }
  if(finish_init_old == 1){ //33
    finishInit_old();
  }else if(driver_init == DRIVER_INIT_CONF){ //111
    tmc_driver_conf_run *c = new tmc_driver_conf_run;
    setState(NS_STATE_RUN);

    run_microsteps = NS_RUN_MICROSTEPS;
    
    //Init settings ------------
    begin(); 
    delay(100);
    //trying what is on the paper
    toff(c->toff);
    blank_time(c->blank_time);
    rms_current(c->rms_current,c->rms_idle);
    en_pwm_mode(c->en_pwm_mode);      // Enable extremely quiet stepping
    pwm_autoscale(c->pwm_autoscale);
    changeMicrosteps(run_microsteps);
    THIGH(c->thigh); // guessing
    //ihold(17); //should be about 70% of irun
    //iholddelay(2); // 1 - 15 so 8 is middle_ish
    //TPOWERDOWN(64); //roughing it
    //overshooting before TPWMTHRS? Increase this (max 15)
    pwm_grad(c->pwm_grad); // was 4 but 1 recommended to start with
    //overshooting past TPWMTHRS? Set this lower (keep above ~50)
    pwm_ampl(c->pwm_ampl); // 255 recommended to start with
    pwm_freq(c->pwm_freq); // based on table in documentation might make sense
    //pwm_ampl();
            // Recommended start values
    hstrt(c->hstrt);// 4 - 7(exl)
    hend(c->hend); // 0 - 3(exl)
    TCOOLTHRS(c->tcoolthrs); // was 
    TPWMTHRS(c->tpwmthrs); //was zero, worked with 16ms but now with 32ms not anymore
    

    delay(100);
    calculateSizesForMicrostepping(run_microsteps);

  }else{ //DMX DRIVER == 99(?)
    setState(NS_STATE_RUN);

    run_microsteps = NS_RUN_MICROSTEPS;
    
    //Init settings ------------
    begin(); 
    delay(100);

    //trying what is on the paper
    toff(5);
    blank_time(36);
    rms_current(800,0.4);
    
    en_pwm_mode(1);      // Enable extremely quiet stepping
    pwm_autoscale(1);
   
    changeMicrosteps(run_microsteps);
    
    THIGH(0); // guessing

    //ihold(17); //should be about 70% of irun
    //iholddelay(2); // 1 - 15 so 8 is middle_ish
    
    //TPOWERDOWN(64); //roughing it
    
    //overshooting before TPWMTHRS? Increase this (max 15)
    pwm_grad(4); // was 4 but 1 recommended to start with

    //overshooting past TPWMTHRS? Set this lower (keep above ~50)
    pwm_ampl(200); // 255 recommended to start with
    pwm_freq(1); // based on table in documentation might make sense

    //pwm_ampl();
            // Recommended start values
    
    hstrt(3);// 4 - 7(exl)
    hend(0); // 0 - 3(exl)

    TCOOLTHRS(0); // was 
    
    TPWMTHRS(0); //was zero, worked with 16ms but now with 32ms not anymore
    
    if(DEBUG_OUTPUT)Serial.print(" GCONF:"); //100 means everything left of 1 is 0 (yes that is correct!)
    u_int32_t tmp = GCONF();
    if(DEBUG_OUTPUT)Serial.println(tmp, BIN); //100?
    tmp = tmp >> 1;
    if(DEBUG_OUTPUT)Serial.println(tmp, BIN); //100?
    tmp = tmp >> 1;
    if(DEBUG_OUTPUT)Serial.println(tmp, BIN); //100?
    
    if(DEBUG_OUTPUT)Serial.println();
    
    if(DEBUG_OUTPUT)Serial.println();
    if(DEBUG_OUTPUT)Serial.println();

    delay(100);
    calculateSizesForMicrostepping(run_microsteps);
  }
}


void NailStepper::move_steps(int steps, int dir=0){

  if(DEBUG_OUTPUT)Serial.print("move_steps: ");
  if(DEBUG_OUTPUT)Serial.println(steps);
  if(DEBUG_OUTPUT)Serial.print("move_dir: ");
  if(DEBUG_OUTPUT)Serial.println(dir);

  if(dir == 0){
    digitalWrite( _pinDir,  LOW); 
  }else if(dir == 1){
    digitalWrite( _pinDir,  HIGH); 
  }

  for(int i=0;i<(steps*2);i++){
    digitalWrite(_pinStep, !digitalRead(_pinStep));
    delayMicroseconds(500);
  }
  
  
}



void NailStepper::move_distance_mm(int mm, int dir){
  if(run_steps_per_mm == 0){
    init_run_steps_per_mm();
  }

  if(DEBUG_OUTPUT)Serial.print("move_distance_mm: ");
  if(DEBUG_OUTPUT)Serial.println(mm);
  if(DEBUG_OUTPUT)Serial.print("move_dir: ");
  if(DEBUG_OUTPUT)Serial.println(dir);

  move_steps((mm*run_steps_per_mm), dir);
}

void NailStepper::init_run_steps_per_mm(){

  one_revolution_distance = gear_tooth_count * belt_pitch;
  run_total_steps_per_revolution = fullsteps_per_revolution * init_microsteps;    
  run_steps_per_mm = run_total_steps_per_revolution/one_revolution_distance;

}

void NailStepper::scanDistance(){
  scanInit();
  stall_counter = 0;
  digitalWrite( _pinDir,  LOW); 

  global_step = _pinStep;
  global_step_counter = step_counter;

  timer.begin(initStepTimer, init_timerSpeed); //150000 = every 0.15 seconds => 1.000.000 = 1sec (??)
  state_active = true;
  
  if(scanType == SCAN_TYPE_HOME){
    full_distance = default_distance_mm;
  }
  

  while(!distanceKnown || drive_to_start_position){
    TMC2130_n::DRV_STATUS_t drv_status{0};
    drv_status.sr = DRV_STATUS();

    if(!state_stall && drv_status.sg_result <= 1 && global_step_counter >= 250){
      timer.end();
      state_stall = true; 

      if(scanType == SCAN_TYPE_HOME){
        return;
      }
    }

    if(state_stall && state_active){
			state_active = false;

			current_dir = digitalRead(_pinDir);
			
			digitalWrite( _pinDir,  !current_dir); 
			digitalWrite(_pinStep, LOW); 
			
			stall_counter++;

			global_step_counter = global_step_counter / 2; // dev 2 because counting each flank of step
			if(stall_counter % 2 == 0){
				//even numbers
				steps_dir_A = global_step_counter;
			}else{
				steps_dir_B = global_step_counter;
			}

			distance_dir_A = int(float(steps_dir_A) * one_step_distance_in_mm * 100)/100.0;
			distance_dir_B = int(float(steps_dir_B) * one_step_distance_in_mm * 100)/100.0;
			
			printStallInfo();
			
			distance_diff = distance_dir_A - distance_dir_B;
			if(distance_diff < 0.0){
				distance_diff = distance_diff * -1;
			}
			if(distance_diff <= scan_distance_offset && stall_counter >= 2 && distance_dir_A >= scan_distance_min_distance){
				if(DEBUG_OUTPUT)Serial.println("Finished Calibration!");
				if(DEBUG_OUTPUT)Serial.print("Diff:");
				if(DEBUG_OUTPUT)Serial.println(distance_diff);
        if(DEBUG_OUTPUT)Serial.print("CurrentDir: ");
        if(DEBUG_OUTPUT)Serial.println(current_dir);
        if(current_dir == 1){
          drive_to_start_position = true;
          state_stall = false;
				  state_active = true;
				  timer.begin(initStepTimer, init_timerSpeed);				
        }
        
				distanceKnown=true;
        if (distance_dir_A >= distance_dir_B){
          full_distance = distance_dir_B;
        }else{
          full_distance = distance_dir_A;
        }
				delay(1000);

			}else if(stall_counter <= 10 && !drive_to_start_position){ //again
				state_stall = false;
				state_active = true;
				global_step_counter = 0;
        delay(500); 
				timer.begin(initStepTimer, init_timerSpeed);				
        digitalWrite( _pinEn,  LOW ); 
			}else if(drive_to_start_position){
        if(DEBUG_OUTPUT)Serial.println("Start position has been reached.");
        drive_to_start_position = false;
      }else{
				if(DEBUG_OUTPUT)Serial.println("ERROR! Can't get conclusive result");
				
				digitalWrite( _pinEn, HIGH ); 
				delay(10000);
			}

		}else if(!state_stall){
			//printDebugInfo(drv_status);
		}
  }

  if(DEBUG_OUTPUT)Serial.println("End - Scan Distance!");
}




void NailStepper::printStallInfo(){
    if(DEBUG_OUTPUT){
      Serial.print("Current Direction: ");
      Serial.println(current_dir);
      Serial.println("stall detected! Press 1 to start again, changing direction as well");
      Serial.print("Step Counter: ");
      Serial.println(global_step_counter);
      Serial.print("Dir1: ");
      Serial.print(steps_dir_A);
      Serial.print(" -Distance Dir1: ");
      Serial.println(distance_dir_A);
      Serial.print("Dir2: ");
      Serial.print(steps_dir_B);
      Serial.print(" -Distance Dir2: ");
      Serial.println(distance_dir_B);
      Serial.print("Stall Counter: ");
      Serial.println(stall_counter);
      Serial.print("one_step_distance:(*100) ");
      Serial.println(double(one_step_distance_in_mm*100.0));
      Serial.print("one_rev_distance: ");
      Serial.println(one_revolution_distance);
      Serial.print("total_steps_per_rev: ");
      Serial.println(total_steps_per_revolution);
    }
    
	
}

void NailStepper::printDriverStatus(TMC2130_n::DRV_STATUS_t drv_status){
    //Serial.println(drv_status);

    //Driver Status output, might be pushing the loop limit,...
    //TMC2130_n::DRV_STATUS_t drv_status{0};
    //drv_status.sr = DRV_STATUS();
    //printDriverStatus(drv_status);

    Serial.printf("id: %d, sr: %s\n",id, byte32_to_binary(drv_status.sr));
    Serial.printf("id: %d, adr: %d, cs_a: %d, fsact: %d, ola: %d, olb %d, ot: %d, otpw: %d, s2ga: %d, s2gb: %d, sg_res: %d, sr: %d, stall: %d, stst: %d\n",
    id,
    drv_status.address,
    drv_status.cs_actual,
    drv_status.fsactive,
    drv_status.ola,
    drv_status.olb,
    drv_status.ot,
    drv_status.otpw,
    drv_status.s2ga,
    drv_status.s2gb,
    drv_status.sg_result,
    drv_status.sr,
    drv_status.stallGuard,
    drv_status.stst
    );
    
    
}

void NailStepper::printDebugInfo(TMC2130_n::DRV_STATUS_t drv_status){
    if(DEBUG_OUTPUT){
      Serial.print("Active:");
      Serial.print(state_active, DEC);
      Serial.print(" SG:");
      Serial.print(drv_status.sg_result, DEC);
      Serial.print(" CS:");
      Serial.print(drv_status.cs_actual, DEC);
      Serial.print(" CS2RMS:");
      Serial.print(cs2rms(drv_status.cs_actual), DEC);
      Serial.print(" RPM:");
      Serial.print(getRPM());
      Serial.print(" Timer:");
      Serial.print(init_timerSpeed);	
      Serial.print(" Coil A:");
      Serial.print(coil_A());	
      Serial.print(" Coil B:");
      Serial.println(coil_B());
    }
}


void NailStepper::testQueue(){
    std::queue<int> myqueue;
    int myint = 9; 
    
    for(int i=0; i<myint; i++){
      myqueue.push(i);

    }
    if(DEBUG_OUTPUT)Serial.println("Queue Test Pop: ");
    while(!myqueue.empty()){
      if(DEBUG_OUTPUT)Serial.print(", ");
      if(DEBUG_OUTPUT)Serial.print(myqueue.front());
      myqueue.pop();

    }
    if(DEBUG_OUTPUT)Serial.println("");


}



void NailStepper::testVector(){
    std::vector<int> my_vector;
    int myint = 5000; 
    
    for(int i=0; i<myint; i++){
      my_vector.push_back(i);

    }

    if(DEBUG_OUTPUT)Serial.println("Vector Test: ");
    for(auto i = my_vector.begin(); i != my_vector.end(); i++){
      if(DEBUG_OUTPUT)Serial.print(*i);
      if(DEBUG_OUTPUT)Serial.print(" ");
    }
}