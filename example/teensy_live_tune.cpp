/**
 * Author Teemu Mäntykallio
 * Initializes the library and runs the stepper motor.
 */

#include <TMCStepper.h>

#define EN_PIN           8 // Enable
#define DIR_PIN          7 // Direction
#define STEP_PIN         6 // Step
#define CS_PIN           10 // Chip select
#define SW_MOSI          11 // Software Master Out Slave In (MOSI)
#define SW_MISO          12 // Software Master In Slave Out (MISO)
#define SW_SCK           13 // Software Slave Clock (SCK)


#define R_SENSE 0.11f // Match to your driver

TMC2130Stepper myStepper = TMC2130Stepper(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); 

using namespace TMC2130_n;

int speed = 10;
bool running = false;
float Rsense = 0.11;
float hold_x = 0.5;
boolean toggle1 = 0;
uint8_t stall_value = 9;
volatile uint32_t step_counter = 0;

//uint8_t gear_tooths = 16;
uint8_t gear_tooths = 40; //that seems to be the setting of the org version
uint8_t tooth_pitch = 2; //in mm
uint8_t one_rev_distance = gear_tooths * tooth_pitch; // on rev in mm (32)mm
uint16_t microsteps = 256;
const uint16_t fsteps_per_rotation = 200;
uint16_t msteps_per_rotation = microsteps * fsteps_per_rotation;  //51200 @256msteps & 200fsteps

// 
uint32_t steps_per_mm = msteps_per_rotation/one_rev_distance; // 51200/32 = 1.600 steps/mm // my version of steps per mm

//const uint32_t steps_per_mm = 80 * 16; // @ 256 microsteps

const uint32_t MHz = 16000000>>8; // Scaled by 256
const uint16_t acceleration = 2000;

uint16_t timerInterval = 50; //50 micro seconds = 20kHz
IntervalTimer stepSpeedTimer;

void takeStep(){
  digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
  step_counter++;
}

void initTimer(){

}

uint16_t calculateMMSTimer(uint8_t mms) {
	uint32_t steps_per_second = mms * steps_per_mm;
	steps_per_second >>= 8;
	uint32_t tmr = MHz / steps_per_second - 1;
	return tmr;
}
uint16_t calculateFSPSTimer(uint16_t fsps) {
	return MHz / fsps - 1;
}
uint16_t calculateRPSTimer(uint8_t rps) {
	return MHz / (rps * fsteps_per_rotation) - 1;
}

void accelerationRamp(uint16_t maxMMS = 100) {
  stepSpeedTimer.end(); //TIMSK1 &= ~(1 << OCIE1A); //(?) disable timer
	while (myStepper.coil_A() != 0) { // Use coil_B if measuring from B coil
		digitalWrite(STEP_PIN, HIGH);
		digitalWrite(STEP_PIN, LOW);
		delay(1);
	}
	delay(100);
	digitalWrite(EN_PIN, HIGH);
	uint16_t mms = 2;
  	//uint16_t maxTimerInterval = calculateMMSTimer(maxMMS);  //uint16_t maxOCR1A = calculateMMSTimer(maxMMS);
  	uint16_t _timerInterval = calculateMMSTimer(mms);  //uint16_t _OCR1A = calculateMMSTimer(mms);
	digitalWrite(EN_PIN, LOW);
	timerInterval = _timerInterval;
  	//OCR1A = _OCR1A;
	//TIMSK1 |= 1 << OCIE1A;
  	stepSpeedTimer.begin(takeStep,timerInterval);
	for (; mms <= maxMMS; mms += acceleration) {
		delay(1);
    stepSpeedTimer.update(calculateMMSTimer(mms)); //OCR1A = calculateMMSTimer(mms);
		
	}
	
  	noInterrupts(); //cli();
	step_counter = 0;
  	interrupts(); //sei();
	
	while (step_counter <= 100*steps_per_mm);
	for (; mms >= 2; mms -= acceleration) {
		delay(1);
		stepSpeedTimer.update(calculateMMSTimer(mms)); //OCR1A = calculateMMSTimer(mms);
	}
}

void serialTuple(String cmd, int arg) {
	Serial.print("Received command: ");
	Serial.print(cmd);
	Serial.print("(");
	Serial.print(arg);
	Serial.println(")");
}

void setup() {
  	//stepSpeedTimer.begin(takeStep, timerInterval);
	Serial.begin(250000);
  	while(!Serial);    
  	Serial.println("Setup started:");

	pinMode(EN_PIN, OUTPUT);
	pinMode(DIR_PIN, OUTPUT);
	pinMode(STEP_PIN, OUTPUT);
	pinMode(CS_PIN, OUTPUT);
	digitalWrite(EN_PIN, HIGH); //deactivate driver (LOW active)
	digitalWrite(DIR_PIN, LOW); //LOW or HIGH
	digitalWrite(STEP_PIN, LOW);
	digitalWrite(CS_PIN, HIGH);
	SPI.begin();
	pinMode(MISO, INPUT_PULLUP);

  
	//myStepper.push();
	//myStepper.tbl(1); //blank_time(24);
	//myStepper.TPOWERDOWN(255);
	//myStepper.toff(4);

	myStepper.begin();

	Serial.print("Testing Connection: ");
	Serial.println(myStepper.test_connection());
	digitalWrite(EN_PIN, LOW);
	Serial.print("Is Enabled: ");
	Serial.println(myStepper.isEnabled());
	digitalWrite(EN_PIN, HIGH);
	Serial.print("Is Enabled: ");
	Serial.println(myStepper.isEnabled());



	// Effective hysteresis = 0
	myStepper.hstrt(0); // hysteresis start value added to HEND
	myStepper.hend(2);  // hysteresis low value OFFSET sine wave offset

	myStepper.en_pwm_mode(true);
	myStepper.pwm_freq(1);
	myStepper.pwm_autoscale(true);
	myStepper.pwm_ampl(180);
	myStepper.pwm_grad(1);

	myStepper.rms_current(500); // mA
	myStepper.microsteps(microsteps);
	myStepper.diag1_stall(1);
	myStepper.diag1_pushpull(1);
	digitalWrite(EN_PIN, LOW);

  	Serial.print("GCONF: 0b");
	Serial.println(myStepper.GCONF(), BIN);

  	//myStepper.direct_mode(1);

  	Serial.print("GCONF: 0b");
	Serial.println(myStepper.GCONF(), BIN);

	/*
	while(Serial.available() <= 0);
	Serial.println("Setup going into loop");
	//stepSpeedTimer.begin(takeStep, timerInterval);
		
	while (myStepper.coil_A() < 240) { // Use coil_B if measuring from B coil
			digitalWrite(STEP_PIN, HIGH);
			digitalWrite(STEP_PIN, LOW);
			delay(10);
    Serial.print("coil_A = ");
    Serial.print(myStepper.coil_A(), BIN);
    Serial.print("  coil_B = ");
    Serial.println(myStepper.coil_B(), BIN);
	}*/
	Serial.print("coil_A = ");
	Serial.print(myStepper.coil_A(), DEC);
	Serial.print("  coil_B = ");
	Serial.println(myStepper.coil_B(), DEC);
  
}

void loop() {
	if (Serial.available() > 0) {
		String cmd = Serial.readStringUntil(' ');
		String strArg = Serial.readStringUntil('\n');

		int arg = strArg.toInt();

		if (cmd == "run") {
			serialTuple("run", arg);
			running = arg;
			//arg ? TIMSK1 |= 1 << OCIE1A : TIMSK1 &= ~(1 << OCIE1A);
			//arg ? digitalWrite(EN_PIN, LOW) : digitalWrite(EN_PIN, HIGH);
      if(arg){
        stepSpeedTimer.begin(takeStep, timerInterval);
      }else{
        stepSpeedTimer.end();        
      }
		}

		else if (cmd == "mms") {
			serialTuple("mms", arg);
      timerInterval = calculateMMSTimer(arg);
      stepSpeedTimer.update(timerInterval);
      Serial.print("MMS  - Set timerInterval to ");
      Serial.println(timerInterval);

			/*cli();
			TCNT1 = 0;
			OCR1A = calculateMMSTimer(arg);
			Serial.print("Set OCR1A to ");
			Serial.println(OCR1A);
			sei();*/
		}
		else if (cmd == "fsps") {
			serialTuple("fsps", arg);
      timerInterval = calculateFSPSTimer(arg);
      stepSpeedTimer.update(timerInterval);
      Serial.print("FSPS - Set timerInterval to ");
      Serial.println(timerInterval);

      /*
			cli();
			OCR1A = calculateFSPSTimer(arg);
			Serial.print("Set OCR1A to ");
			Serial.println(OCR1A);
			sei();
      */
		}
		else if (cmd == "rps") {
			serialTuple("rps", arg);
      timerInterval = calculateRPSTimer(arg);
      stepSpeedTimer.update(timerInterval);
      Serial.print("RPS  - Set timerInterval to ");
      Serial.println(timerInterval);
			
      /*
      cli();
			OCR1A = calculateRPSTimer(arg);
			Serial.print("Set OCR1A to ");
			Serial.println(OCR1A);
			sei();
      */
		}
		else if (cmd == "rms_current") {
			serialTuple("rms_current", arg);
			myStepper.rms_current(arg, hold_x);
		}
		else if (cmd == "find_pos") {
			serialTuple("find_pos", arg);
			stepSpeedTimer.end(); //TIMSK1 &= ~(1 << OCIE1A);
			while (myStepper.coil_A() != arg) { // Use coil_B if measuring from B coil
				digitalWrite(STEP_PIN, HIGH);
				digitalWrite(STEP_PIN, LOW);
				delay(1);
			}
		}
		else if (cmd == "rampTo") {
			serialTuple("rampTo", arg);
			accelerationRamp(arg);
		}
		else if (cmd == "Rsense") {
			Serial.print("Setting R sense value to: ");
			Serial.println(arg);
			Rsense = arg;
		}
		else if (cmd == "hold_multiplier") {
			Serial.print("Setting hold multiplier to: ");
			Serial.println(arg);
			hold_x = arg;
		}
		else if (cmd == "GCONF") {
			Serial.print("GCONF: 0b");
			Serial.println(myStepper.GCONF(), BIN);
		}
		else if (cmd == "I_scale_analog") {
			serialTuple("I_scale_analog", arg);
			myStepper.I_scale_analog(arg);
		}
		else if (cmd == "internal_Rsense") {
			serialTuple("internal_Rsense", arg);
			myStepper.internal_Rsense(arg);
		}
		else if (cmd == "en_pwm_mode") {
			serialTuple("en_pwm_mode", arg);
			myStepper.en_pwm_mode(arg);
		}
		else if (cmd == "enc_commutation") {
			serialTuple("enc_commutation", arg);
			myStepper.enc_commutation(arg);
		}
		else if (cmd == "shaft") {
			serialTuple("shaft", arg);
			myStepper.shaft(arg);
		}
		else if (cmd == "diag0_error") {
			serialTuple("diag0_error", arg);
			myStepper.diag0_error(arg);
		}
		else if (cmd == "diag0_otpw") {
			serialTuple("diag0_otpw", arg);
			myStepper.diag0_otpw(arg);
		}
		else if (cmd == "diag0_stall") {
			serialTuple("diag0_stall", arg);
			myStepper.diag0_stall(arg);
		}
		else if (cmd == "diag1_stall") {
			serialTuple("diag1_stall", arg);
			myStepper.diag1_stall(arg);
		}
		else if (cmd == "diag1_index") {
			serialTuple("diag1_index", arg);
			myStepper.diag1_index(arg);
		}
		else if (cmd == "diag1_onstate") {
			serialTuple("diag1_onstate", arg);
			myStepper.diag1_onstate(arg);
		}
		else if (cmd == "diag0_int_pushpull") {
			serialTuple("diag0_int_pushpull", arg);
			myStepper.diag0_int_pushpull(arg);
		}
		else if (cmd == "diag1_pushpull") {
			serialTuple("diag1_pushpull", arg);
			myStepper.diag1_pushpull(arg);
		}
		else if (cmd == "small_hysteresis") {
			serialTuple("small_hysteresis", arg);
			myStepper.small_hysteresis(arg);
		}
		else if (cmd == "stop_enable") {
			serialTuple("stop_enable", arg);
			myStepper.stop_enable(arg);
		}
		else if (cmd == "direct_mode") {
			serialTuple("direct_mode", arg);
			myStepper.direct_mode(arg);
		}
		// IHOLD_IRUN
		else if (cmd == "ihold") {
			serialTuple("ihold", arg);
			myStepper.ihold(arg);
		}
		else if (cmd == "irun") {
			serialTuple("irun", arg);
			myStepper.irun(arg);
		}
		else if (cmd == "iholddelay") {
			serialTuple("iholddelay", arg);
			myStepper.iholddelay(arg);
		}

		else if (cmd == "TSTEP") {
			Serial.print("TSTEP: ");
			Serial.println(myStepper.TSTEP());
		}

		else if (cmd == "TPWMTHRS") {
			serialTuple("TPWMTHRS", arg);
			myStepper.TPWMTHRS(arg);
		}
		else if (cmd == "TCOOLTHRS") {
			serialTuple("TCOOLTHRS", arg);
			myStepper.TCOOLTHRS(arg);
		}
		else if (cmd == "THIGH") {
			serialTuple("THIGH", arg);
			myStepper.THIGH(arg);
		}
		// XDIRECT
		else if (cmd == "coil_A") {
			serialTuple("coil_A", arg);
			myStepper.coil_A(arg);
		}
		else if (cmd == "coil_B") {
			serialTuple("coil_B", arg);
			myStepper.coil_B(arg);
		}

		else if (cmd == "VDCMIN") {
			serialTuple("VDCMIN", arg);
			myStepper.VDCMIN(arg);
		}

		else if (cmd == "CHOPCONF") {
			Serial.print("CHOPCONF: 0b");
			Serial.println(myStepper.CHOPCONF(), BIN);
		}
		else if (cmd == "toff") {
			serialTuple("toff", arg);
			myStepper.toff(arg);
		}
		else if (cmd == "hstrt") {
			serialTuple("hstrt", arg);
			myStepper.hstrt(arg);
		}
		else if (cmd == "hend") {
			serialTuple("hend", arg);
			myStepper.hend(arg);
		}
		/*
		else if (cmd == "fd") {
			serialTuple("fd", arg);
			myStepper.fd(arg);
		}
		*/
		else if (cmd == "disfdcc") {
			serialTuple("disfdcc", arg);
			myStepper.disfdcc(arg);
		}
		else if (cmd == "rndtf") {
			serialTuple("rndtf", arg);
			myStepper.rndtf(arg);
		}
		else if (cmd == "chm") {
			serialTuple("chm", arg);
			myStepper.chm(arg);
		}
		else if (cmd == "tbl") {
			serialTuple("tbl", arg);
			myStepper.tbl(arg);
		}
		else if (cmd == "vsense") {
			serialTuple("vsense", arg);
			myStepper.vsense(arg);
		}
		else if (cmd == "vhighfs") {
			serialTuple("vhighfs", arg);
			myStepper.vhighfs(arg);
		}
		else if (cmd == "vhighchm") {
			serialTuple("vhighchm", arg);
			myStepper.vhighchm(arg);
		}
		else if (cmd == "sync") {
			serialTuple("sync", arg);
			myStepper.sync(arg);
		}
		else if (cmd == "mres") {
			serialTuple("mres", arg);
			myStepper.mres(arg);
		}
		else if (cmd == "intpol") {
			serialTuple("intpol", arg);
			myStepper.intpol(arg);
		}
		else if (cmd == "dedge") {
			serialTuple("dedge", arg);
			myStepper.dedge(arg);
		}
		else if (cmd == "diss2g") {
			serialTuple("diss2g", arg);
			myStepper.diss2g(arg);
		}
		// COOLCONF
		else if (cmd == "semin") {
			serialTuple("semin", arg);
			myStepper.semin(arg);
		}
		else if (cmd == "seup") {
			serialTuple("seup", arg);
			myStepper.seup(arg);
		}
		else if (cmd == "semax") {
			serialTuple("semax", arg);
			myStepper.semax(arg);
		}
		else if (cmd == "sedn") {
			serialTuple("sedn", arg);
			myStepper.sedn(arg);
		}
		else if (cmd == "seimin") {
			serialTuple("seimin", arg);
			myStepper.seimin(arg);
		}
		else if (cmd == "sgt") {
			serialTuple("sgt", arg);
			myStepper.sgt(arg);
		}
		else if (cmd == "sfilt") {
			serialTuple("sfilt", arg);
			myStepper.sfilt(arg);
		}
		// PWMCONF
		else if (cmd == "pwm_ampl") {
			serialTuple("pwm_ampl", arg);
			myStepper.pwm_ampl(arg);
		}
		else if (cmd == "pwm_grad") {
			serialTuple("pwm_grad", arg);
			myStepper.pwm_grad(arg);
		}
		else if (cmd == "pwm_freq") {
			serialTuple("pwm_freq", arg);
			myStepper.pwm_freq(arg);
		}
		else if (cmd == "pwm_autoscale") {
			serialTuple("pwm_autoscale", arg);
			myStepper.pwm_autoscale(arg);
		}
		else if (cmd == "pwm_symmetric") {
			serialTuple("pwm_symmetric", arg);
			myStepper.pwm_symmetric(arg);
		}
		else if (cmd == "freewheel") {
			serialTuple("freewheel", arg);
			myStepper.freewheel(arg);
		}
		// ENCM_CTRL
		else if (cmd == "inv") {
			serialTuple("inv", arg);
			myStepper.inv(arg);
		}
		else if (cmd == "maxspeed") {
			serialTuple("maxspeed", arg);
			myStepper.maxspeed(arg);
		}

		else if (cmd == "DRVSTATUS") {
			Serial.print("DRVSTATUS: 0b");
			Serial.println(myStepper.DRV_STATUS(), BIN);
		}
		else if (cmd == "PWM_SCALE") {
			Serial.print("PWM_SCALE: 0b");
			Serial.println(myStepper.PWM_SCALE(), DEC);
		}
		else if (cmd == "LOST_STEPS") {
			Serial.print("LOST_STEPS: 0b");
			Serial.println(myStepper.LOST_STEPS(), DEC);
		}
		else {
			Serial.print("Invalid command! :#");
      Serial.print(cmd);
      Serial.println("#");
		}
	}
}
