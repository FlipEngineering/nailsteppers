/**
 * Author Teemu Mäntykallio
 *
 * Plot TMC2130 or TMC2660 motor load using the stallGuard value.
 * You can finetune the reading by changing the STALL_VALUE.
 * This will let you control at which load the value will read 0
 * and the stall flag will be triggered. This will also set pin DIAG1 high.
 * A higher STALL_VALUE will make the reading less sensitive and
 * a lower STALL_VALUE will make it more sensitive.
 *
 * You can control the rotation speed with
 * 0 Stop
 * 1 Resume
 * + Speed up
 * - Slow down
 */
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
int one_rev_distance = gear_tooth_count*belt_pitch;
int fstep_one_rev = 200;
int total_steps_per_rev = fstep_one_rev*microsteps;
float one_step_distance = float(one_rev_distance)/float(total_steps_per_rev);

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

/*	
	Serial.print("total_steps_per_rev: fstep_one_rev*microsteps 200*16: ");
	Serial.println(total_steps_per_rev);
	Serial.print("step_freq = hz/timer: ");
	Serial.println(step_freq);
	Serial.print("rps tot/step_freq: ");
	Serial.println(rps);
	Serial.print("_rpm:");
	Serial.println(_rpm);
	Serial.print("rpm:");
	Serial.println(rpm);
*/
	return rpm;
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

  //myTimer.begin(timerFunction, timer_speed); //150000 = every 0.15 seconds => 1.000.000 = 1sec (??)

}

void loop() {
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

  if((ms-last_time) > 50){  //run every 0.1s
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
			Serial.print("Current Direction: ");
			Serial.println(current_direction);
			digitalWrite( DIR_PIN,  !current_direction); 
			digitalWrite(STEP_PIN, LOW); 
			
			stall_counter++;

			if(stall_counter % 2 == 0){
				//even numbers
				steps_dir1 = step_counter;
			}else{
				steps_dir2 = step_counter;
			}

			float dist1 = int(float(steps_dir1) * one_step_distance * 100)/100.0;
			float dist2 = int(float(steps_dir2) * one_step_distance * 100)/100.0;

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
			Serial.println(one_step_distance);
			
			
		}else if(!stall_flag){
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
  	}
  }
}

