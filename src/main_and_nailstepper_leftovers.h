
//---------Nail Stepper Pins ----------
#define EN1_PIN           8 // Enable
#define DIR1_PIN          7 // Direction
#define STEP1_PIN         6 // Step
#define CS1_PIN           31 // Chip select

#define EN2_PIN           27 // Enable
#define DIR2_PIN          25 // Direction
#define STEP2_PIN         26 // Step
#define CS2_PIN           24 // Chip select

#define EN3_PIN           15 // Enable
#define DIR3_PIN          17 // Direction
#define STEP3_PIN         16 // Step
#define CS3_PIN           14 // Chip select

#define EN4_PIN           20 // Enable
#define DIR4_PIN          22 // Direction
#define STEP4_PIN         21 // Step
#define CS4_PIN           19 // Chip select

//int gears[] = {16,20,20,20};
int gears[] = {16,20,20,20};


//in theorie
//v = 800 mm/sec 
//a = 3200 mm/sec²
//
// microsteps 4
// gear 20t, pitch 2mm = 40mm / rev
// steps 200 
// 40mm/200stp = 0.2mm per f-step
// 0.05mm per step @ 4ms*20
// -> 16.000 steps/sec
//
// dist 100mm
// a = 64.000 stps/sec

// motor_speed = 800 (in mm/sec)
// one_rev_mm = gear_teeth * pitch
// steps_one_rev = fstep * ms
// steps_per_mm = steps_on_rev/one_rev_mm
// motor_speed_steps = steps_per_mm * motor_speed_mm


//int motor_speed = 16000; //steps per second (@4ms)
//int motor_speed = 40000; //steps per second
//int motor_acc = motor_speed * 4; // reach max speed in 0.25sec -> *2
//int motor_acc = 100000;
//int motor_pull_in_speed = 2000;

//32mm rev
//4 * 200 = 800 

IntervalTimer tickTimer;

void timerSetup(){
 	tick();
	
	// use a timer to periodically calculate new targets for the slide
	tickTimer.priority(TICK_TIMER_PRIORITY); // lowest priority, potentially long caclulations need to be interruptable by TeensyStep
	Serial.println("[MAIN] - TickTimerSetup:");
	Serial.println(tickTimer.begin(tick, recalcPeriod));
}



//from stepper setup:


	//nails[1]->setDistance(30, BELT_PITCH, full_steps_per_revolution);
	//nails[1]->scanDistance();
	//nails[1]->scanInit();
	//nails[1]->full_distance = 170;
	//nails[1]->offsetEndstops(offset_endstop_mm, offset_dir);
	//nails[1]->finishInit(microStepz);

	//nails[2]->setDistance(30, BELT_PITCH, full_steps_per_revolution);
	//nails[2]->scanDistance();
	//nails[2]->scanInit();
	//nails[2]->full_distance = 170;
	//nails[2]->offsetEndstops(offset_endstop_mm, offset_dir);
	//nails[2]->finishInit(microStepz);

	//nails[3]->setDistance(20, BELT_PITCH, full_steps_per_revolution);
	//nails[3]->scanDistance();
	//nails[3]->scanInit();
	//nails[3]->full_distance = 170;
	//nails[3]->offsetEndstops(offset_endstop_mm, offset_dir);
	//nails[3]->finishInit(microStepz);

	//for(int i=0;i<CONNECTED_STEPPERS;i++){
	//	nails[i]->stepper.setPosition(0);
		//nails[i]->stepper.setInverseRotation(1);
	//	nails[i]->stepper.setAcceleration(motor_acc);
	//	nails[i]->stepper.setMaxSpeed(motor_speed);	
	//	nails[i]->stepper.setPullInSpeed(motor_pull_in_speed);

	//	nails[i]->controller->rotateAsync(nails[i]->stepper);
	//	nails[i]->controller->overrideSpeed(0); // start with stopped slide
	//	nails[i]->stepper.setPosition(0);
	//}
	//Initialize Rotational Timer Setup 
	//timerSetup();




	//--------------------------------------------------------------------------
	//--------------------------------------------------------------------------
	//--------------------------------------------------------------------------
	//--------------------------------------------------------------------------
	//--------------------------NAILSTEPPER>CPP---------------------------------
	//--------------------------------------------------------------------------
	//--------------------------------------------------------------------------
	//--------------------------------------------------------------------------
	//--------------------------------------------------------------------------
	
/*
void NailStepper::walkTest_setup(){
    delay(500);
    rms_current(600);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    en_pwm_mode(1);      // Enable extremely quiet stepping
    pwm_autoscale(1);
    microsteps(4);

    delay(500);

    float acceleration = 30000; //steps/sec²
    float max_speed = 2000; // if ramp up = 0.25sec then max acc * 4 right (?)

    stepper.setMaxSpeed(max_speed); // 200mm/s @ 80 steps/mm
    stepper.setAcceleration(acceleration); // 20000steps/sec² with 4000mm/sec² and 20gears at 2mm pitch (?)
    
    this->stepper.setEnablePin(_pinEn);
    this->stepper.setPinsInverted(false, false, true);
    this->stepper.enableOutputs();

    if(current_dir == 0){
      walk_dir = -1;
    }else{
      walk_dir = 1;
    }
    
    Serial.println("Moving Stepper down:");
    if (this->stepper.distanceToGo() == 0) {
			this->stepper.disableOutputs();
			delay(10);
			walk_dir = walk_dir * -1;
			int toMove = walk_dir*50;
      //int toMove = walk_dir*1600; //should end up somewhere in the center
			if(walk_dir > 0){
				toMove += 3340;
				walk_dir = walk_dir * -1;
			}
			Serial.print("Move:");
			Serial.println(toMove);
			this->stepper.move(toMove); // 5000 = 4 microstep res , 250mm dist @ 40mm gear, 200 full steps
			this->stepper.enableOutputs();
      Serial.println("Moving Stepper down (inside)");
		}
		this->stepper.run();
    Serial.println("Moving Stepper down, done!");
		
		walk_dir = walk_dir * -1;
}

void NailStepper::walkTest(){
  

 	while(Serial.available() > 0) {
		int8_t read_byte = Serial.read();
	
		if (read_byte == '0' && digitalRead(_pinEn) == LOW)      { 
			state_active = false;
			//distance_left = stepper.distanceToGo();
			this->stepper.disableOutputs();
		
		}else if (read_byte == '1' ) {        // && digitalRead(EN_PIN) == HIGH)
			state_active = true;
			
			//stepper.move(distance_left); // 5000 = 4 microstep res , 250mm dist @ 40mm gear, 200 full steps
			//distance_left = 0;
			this->stepper.enableOutputs();

		}
	}

	if (this->stepper.distanceToGo() == 0 && state_active) {
    this->stepper.disableOutputs();
    delay(10);
		walk_dir = walk_dir * -1;
		int toMove = walk_dir*3340; //3440 would be full distance at 4micro but keeping some distance to the edges
		Serial.print("Move:");
		Serial.println(toMove);
		//Serial.println(ms);
    this->stepper.move(toMove); // 5000 = 4 microstep res , 250mm dist @ 40mm gear, 200 full steps
    this->stepper.enableOutputs();
  }
  this->stepper.run();
}*/



//finish init before mod for new stepper! 25.04
void NailStepper::finishInit(){
    this->setState(NS_STATE_RUN);
    
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
    //sgt(TMC_STALL_VALUE); 
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

    Serial.print("PWM_FREQ: ");
    Serial.println(pwm_freq());
    
    //min 1 - max 15 - standard 4
    //8 - higher pitch sound
    Serial.print("PWM_GRAD: ");
    Serial.println(pwm_grad());
    
    //min 64 - max 255 - standard - 128
    Serial.print("PWM_AMP: ");
    Serial.println(pwm_ampl());

    /*
    Specifies  the  upper  velocity  for  operation  in stealthChop  voltage  PWM  mode.  
    Entry  the TSTEP reading  (time  between  two  microsteps)  when operating at the desired threshold velocity.
    0 ... 1048575
    */
    // default atm is 0
    //TPWMTHRS(1048575); //no noticebal diff

    //TPWMTHRS(1000);
    Serial.print("TPWMTHRS: ");
    Serial.println(TPWMTHRS());

    //default atm is 1048575 (hex 0xFFFFF)
    //100 - 1048575
    //50  - 524287
    //25  - 262143
    // 12 - 131071
    // 1  - 10500
    // 0.1- 1000

    //TCOOLTHRS 131071 & THIGH 1000 seems to kinda work @32ms for 20t gears but not for 16t
    //TCOOLTHRS(10500);
    Serial.print("TCOOLTHRS: ");
    Serial.println(TCOOLTHRS());

    //default atm is 0 or 32 depends how you see it
    //THIGH(1000);
    Serial.print("THIGH: ");
    Serial.println(THIGH());

    Serial.print("IRUN:");
    Serial.print(irun());
    Serial.print(" IHOLD:");
    Serial.print(ihold());
    Serial.print(" IHOLD_DELAY:");
    Serial.println(iholddelay());
    Serial.print("v_sense:");
    Serial.println(vsense());
    Serial.print("tbl:");
    Serial.print(tbl());
    Serial.print(" toff:");
    Serial.print(toff());
    Serial.print(" HSTRT:");
    Serial.print(hstrt());
    Serial.print(" HEND:");
    Serial.println(hend());
    Serial.print(" chm:");
    Serial.println(chm());
    Serial.print(" intpol:");
    Serial.println(intpol());
    Serial.print(" small_hys:");
    Serial.println(small_hysteresis());
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

    

    Serial.print(" GCONF:"); //100 means everything left of 1 is 0 (yes that is correct!)
    u_int32_t tmp = GCONF();
    Serial.println(tmp, BIN); //100?
    tmp = tmp >> 1;
    Serial.println(tmp, BIN); //100?
    tmp = tmp >> 1;
    Serial.println(tmp, BIN); //100?
    
    Serial.println();
    
    Serial.println();
    Serial.println();

    delay(100);

    
    calculateSizesForMicrostepping(run_microsteps);
}