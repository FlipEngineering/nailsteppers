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

#define MAX_SPEED         1000 // In timer value
#define MIN_SPEED         50000
#define SPEED_EFFECT_SIZE 1000
int timer_speed = 6000;

#define STALL_VALUE      11 // [-64..63] (init was 15)

#define EN_PIN           8 // Enable
#define DIR_PIN          7 // Direction
#define STEP_PIN         6 // Step
#define CS_PIN           10 // Chip select
#define SW_MOSI          11 // Software Master Out Slave In (MOSI)
#define SW_MISO          12 // Software Master In Slave Out (MISO)
#define SW_SCK           13 // Software Slave Clock (SCK)

#define R_SENSE 0.11f // Match to your driver
                      // SilentStepStick series use 0.11
                      // UltiMachine Einsy and Archim2 boards use 0.2
                      // Panucatt BSD2660 uses 0.1
                      // Watterott TMC5160 uses 0.075

// Select your stepper driver type
//TMC2130Stepper driver(CS_PIN, R_SENSE);                           // Hardware SPI
TMC2130Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI
//TMC5160Stepper driver(CS_PIN, R_SENSE);
//TMC5160Stepper driver(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);

using namespace TMC2130_n;

// Using direct register manipulation can reach faster stepping times
#define STEP_PORT     PORTF // Match with STEP_PIN
#define STEP_BIT_POS      0 // Match with STEP_PIN

IntervalTimer myTimer;

void timerFunction(){
  digitalWrite(STEP_PIN, !digitalRead(STEP_PIN));
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
  driver.rms_current(500); // mA
  driver.microsteps(16);
  driver.TCOOLTHRS(0xFFFFF); // 20bit max
  driver.THIGH(0);
  driver.semin(5);
  driver.semax(2);
  driver.sedn(0b01);
  driver.sgt(STALL_VALUE);

  myTimer.begin(timerFunction, timer_speed); //150000 = every 0.15 seconds => 1.000.000 = 1sec (??)

}

void loop() {
  static uint32_t last_time=0;
  uint32_t ms = millis();

  while(Serial.available() > 0) {
    int8_t read_byte = Serial.read();
  
    if (read_byte == '0' && digitalRead(EN_PIN) == LOW)      { 
      myTimer.end();
      digitalWrite( EN_PIN, HIGH ); 
    }else if (read_byte == '1' && digitalRead(EN_PIN) == HIGH) {         
      myTimer.begin(timerFunction, timer_speed);
      digitalWrite( EN_PIN,  LOW ); 
    }else if (read_byte == '+') {
      if (timer_speed > MAX_SPEED) timer_speed -= SPEED_EFFECT_SIZE; 
      myTimer.update(timer_speed);
    }else if (read_byte == '-') { 
      if (timer_speed < MIN_SPEED) timer_speed += SPEED_EFFECT_SIZE; 
      myTimer.update(timer_speed);
    }
  }

  if((ms-last_time) > 100) { //run every 0.1s
    last_time = ms;

    DRV_STATUS_t drv_status{0};
    drv_status.sr = driver.DRV_STATUS();

    Serial.print("0 ");
    Serial.print(drv_status.sg_result, DEC);
    Serial.print(" ");
    Serial.print(driver.cs2rms(drv_status.cs_actual), DEC);
    Serial.print(" ");
    Serial.println(timer_speed);
  }
}

