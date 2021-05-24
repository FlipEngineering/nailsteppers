#include <Arduino.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <TMCStepper.h>

#define MAX_SPEED         1 // In timer value
#define MIN_SPEED         50000
#define SPEED_EFFECT_SIZE 1

int timer_speed = 44;
int microsteps = 32;
int hz_base = 1000000;
int rpm = 0;
bool stall_flag = false;
bool active = false;
int stall_counter = 0;
int step_counter = 0;

int steps_dir1 = 0;
int steps_dir2 = 0;

int current_direction = -1;

int gear_tooth_count = 20;
int belt_pitch = 2; //in mm
int one_rev_distance = gear_tooth_count*belt_pitch; //40mm = one rev
int fstep_one_rev = 200;
int total_steps_per_rev = fstep_one_rev*microsteps;
//long double one_step_distance = (long double)one_rev_distance/(long double)total_steps_per_rev;
long double one_step_distance = 40.0/6400.0;

double dist1 = 0.0;
double dist2 = 0.0;

bool init_done = false;
bool first_main = true;
float available_distance = 0;

#define STALL_VALUE      2 // [-64..63] (init was 15)

#define EN_PIN           8 // Enable
#define DIR_PIN          7 // Direction
#define STEP_PIN         6 // Step
#define CS_PIN           10 // Chip select
#define SW_MOSI          11 // Software Master Out Slave In (MOSI)
#define SW_MISO          12 // Software Master In Slave Out (MISO)
#define SW_SCK           13 // Software Slave Clock (SCK)

#define R_SENSE 0.11f // Match to your driver

TMC2130Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI

using namespace TMC2130_n;

// Using direct register manipulation can reach faster stepping times
#define STEP_PORT     PORTF // Match with STEP_PIN
#define STEP_BIT_POS      0 // Match with STEP_PIN

#include <AccelStepper.h>
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);
int dir = 1;
int distance_left = 0;

IntervalTimer myTimer;

void timerFunction(){
  digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
  step_counter++;
}


float getRPM(){
	int step_freq = hz_base/timer_speed;
	float rps = float(step_freq)/float(total_steps_per_rev)/2;
	int _rpm = rps * 6000;
	float rpm = _rpm/100.0;

	return rpm;
}

void printStallInfo(){
	Serial.print("Current Direction: ");
	Serial.println(current_direction);
	Serial.println("stall detected! Press 1 to start again, changing direction as well");
	Serial.print("Step Counter: ");
	Serial.println(step_counter);
	Serial.print("Dir1: ");
	Serial.print(steps_dir1);
	Serial.print(" -Distance Dir1: ");
	Serial.println(dist1);
	Serial.print("Dir2: ");
	Serial.print(steps_dir2);
	Serial.print(" -Distance Dir2: ");
	Serial.println(dist2);
	Serial.print("Stall Counter: ");
	Serial.println(stall_counter);
	Serial.print("one_step_distance: ");
	Serial.println(double(one_step_distance*100.0));
	Serial.print("one_rev_distance: ");
	Serial.println(one_rev_distance);
	Serial.print("total_steps_per_rev: ");
	Serial.println(total_steps_per_rev);
	
	
}

void printDebugInfo(DRV_STATUS_t drv_status){
	Serial.print("Active:");
	Serial.print(active, DEC);
	Serial.print(" SG:");
	Serial.print(drv_status.sg_result, DEC);
	Serial.print(" CS:");
	Serial.print(drv_status.cs_actual, DEC);
	Serial.print(" CS2RMS:");
	Serial.print(driver.cs2rms(drv_status.cs_actual), DEC);
	Serial.print(" RPM:");
	Serial.print(getRPM());
	Serial.print(" Timer:");
	Serial.print(timer_speed);	
	Serial.print(" Coil A:");
	Serial.print(driver.coil_A());	
	Serial.print(" Coil B:");
	Serial.println(driver.coil_B());
}

void init_loop(){
	
  static uint32_t last_time=0;
  uint32_t ms = millis();


  while(Serial.available() > 0) {
    int8_t read_byte = Serial.read();
  
    if (read_byte == '0' && digitalRead(EN_PIN) == LOW)      { 
      myTimer.end();
	  active = false;
      digitalWrite( EN_PIN, HIGH ); 
    }else if (read_byte == '1' ) {        // && digitalRead(EN_PIN) == HIGH)
	  step_counter = 0;
      myTimer.begin(timerFunction, timer_speed);
	  stall_flag = false;
	  active = true;
      digitalWrite( EN_PIN,  LOW ); 
    }else if (read_byte == '+') {
      if (timer_speed > MAX_SPEED) timer_speed -= SPEED_EFFECT_SIZE; 
      myTimer.update(timer_speed);
    }else if (read_byte == '-') { 
      if (timer_speed < MIN_SPEED) timer_speed += SPEED_EFFECT_SIZE; 
      myTimer.update(timer_speed);
    }
  }

  if((ms-last_time) > 10){  //run every 0.1s
    //last_time = ms;

    DRV_STATUS_t drv_status{0};
    drv_status.sr = driver.DRV_STATUS();

	if(!stall_flag && drv_status.sg_result <= 1 && step_counter >= 140){
		myTimer.end();
		stall_flag = true;
	}

	if((ms-last_time) > 200){
		last_time = ms;

		if(stall_flag && active){
			active = false;

			current_direction = digitalRead(DIR_PIN);
			
			digitalWrite( DIR_PIN,  !current_direction); 
			digitalWrite(STEP_PIN, LOW); 
			
			stall_counter++;

			step_counter = step_counter / 2; // dev 2 because counting each flank of step
			if(stall_counter % 2 == 0){
				//even numbers
				steps_dir1 = step_counter;
			}else{
				steps_dir2 = step_counter;
			}

			dist1 = int(float(steps_dir1) * one_step_distance * 100)/100.0;
			dist2 = int(float(steps_dir2) * one_step_distance * 100)/100.0;
			
			printStallInfo();
			
			float diff = dist1 - dist2;
			if(diff < 0.0){
				diff = diff * -1;
			}
			if(diff <= 0.5 && stall_counter >= 2){
				

				Serial.println("Finished Calibration!");
				Serial.print("Diff:");
				Serial.println(diff);
				init_done=true;
				delay(1000);

			}else if(stall_counter <= 6){ //again
				stall_flag = false;
				active = true;
				step_counter = 0;
				myTimer.begin(timerFunction, timer_speed);				
      			digitalWrite( EN_PIN,  LOW ); 
			}else{
				Serial.println("ERROR! Can't get conclusive result");
				
				digitalWrite( EN_PIN, HIGH ); 
				delay(10000);
			}

			
			
			
		}else if(!stall_flag){
			printDebugInfo(drv_status);
		}
  	}
  }
}

void main_setup(){
	driver.rms_current(600);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    driver.en_pwm_mode(1);      // Enable extremely quiet stepping
    driver.pwm_autoscale(1);
    driver.microsteps(4);

	float acceleration = 300000;
	float max_speed = 20000; // if ramp up = 0.25sec then max acc * 4 right (?)

    stepper.setMaxSpeed(max_speed); // 200mm/s @ 80 steps/mm
    stepper.setAcceleration(acceleration); // 20000steps/sec² with 4000mm/sec² and 20gears at 2mm pitch (?)
	
    stepper.setEnablePin(EN_PIN);
    stepper.setPinsInverted(false, false, true);
    stepper.enableOutputs();

	if(current_direction == 0){
		dir = -1;
	}else{
		dir = 1;
	}
	
}


void main_loop(){
	//static uint32_t last_time=0;
  	uint32_t ms = millis();

	if(first_main){
		first_main = false;
		main_setup();

		if (stepper.distanceToGo() == 0) {
			stepper.disableOutputs();
			delay(10);
			dir = dir * -1;
			int toMove = dir*50;
			if(dir > 0){
				toMove += 3340;
				dir = dir * -1;
			}
			Serial.print("Move:");
			Serial.println(toMove);
			stepper.move(toMove); // 5000 = 4 microstep res , 250mm dist @ 40mm gear, 200 full steps
			stepper.enableOutputs();
		}
		stepper.run();
		
		dir = dir * -1;
		//active = true;
	}

	while(Serial.available() > 0) {
		int8_t read_byte = Serial.read();
	
		if (read_byte == '0' && digitalRead(EN_PIN) == LOW)      { 
			active = false;
			//distance_left = stepper.distanceToGo();
			stepper.disableOutputs();
		
		}else if (read_byte == '1' ) {        // && digitalRead(EN_PIN) == HIGH)
			active = true;
			
			//stepper.move(distance_left); // 5000 = 4 microstep res , 250mm dist @ 40mm gear, 200 full steps
			//distance_left = 0;
			stepper.enableOutputs();

		}
	}


	//(ms-last_time) > 200){
	//last_time = ms;

	if (stepper.distanceToGo() == 0 && active) {
        stepper.disableOutputs();
        delay(10);
		dir = dir * -1;
		int toMove = dir*3340; //3440 would be full distance at 4micro but keeping some distance to the edges
		Serial.print("Move:");
		Serial.println(toMove);
		Serial.println(ms);
        stepper.move(toMove); // 5000 = 4 microstep res , 250mm dist @ 40mm gear, 200 full steps
        stepper.enableOutputs();
    }
    stepper.run();
}

void setup() {
  SPI.begin();
  Serial.begin(250000);         // Init serial port and set baudrate
  while(!Serial);               // Wait for serial port to connect
  Serial.println("\nStart...");

  pinMode(EN_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(CS_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(MISO, INPUT_PULLUP);
  digitalWrite(EN_PIN, LOW);

  driver.begin();
  driver.toff(4);
  driver.blank_time(24);
  driver.rms_current(400); // mA 400
  driver.microsteps(microsteps);
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  driver.THIGH(0);
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.sgt(STALL_VALUE);

  myTimer.begin(timerFunction, timer_speed); //150000 = every 0.15 seconds => 1.000.000 = 1sec (??)
  active = true;
}

void loop() {
	if(!init_done){
		init_loop();
	}else{
		main_loop();
	}
}

