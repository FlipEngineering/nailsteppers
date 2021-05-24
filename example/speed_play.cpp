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
                     // SilentStepStick series use 0.11
                     // UltiMachine Einsy and Archim2 boards use 0.2
                     // Panucatt BSD2660 uses 0.1
                     // Watterott TMC5160 uses 0.075

// Select your stepper driver type
//TMC2130Stepper driver = TMC2130Stepper(CS_PIN, R_SENSE); // Hardware SPI
TMC2130Stepper driver = TMC2130Stepper(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI
//TMC2208Stepper driver = TMC2208Stepper(&SERIAL_PORT, R_SENSE); // Hardware Serial0
//TMC2208Stepper driver = TMC2208Stepper(SW_RX, SW_TX, R_SENSE); // Software serial
//TMC2660Stepper driver = TMC2660Stepper(CS_PIN, R_SENSE); // Hardware SPI
//TMC2660Stepper driver = TMC2660Stepper(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);
//TMC5160Stepper driver = TMC5160Stepper(CS_PIN, R_SENSE);
//TMC5160Stepper driver = TMC5160Stepper(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK);

int beltPitch = 2; // 2mm belt pitch
int gearToothCount = 16;

u_int16_t microsteps = 16;

int distancePerRev = beltPitch*gearToothCount; // 32mm per rev
int baseSteps = 200;
int fullStepsPerRev = baseSteps*microsteps;

u_int32_t steps_per_mm = fullStepsPerRev/distancePerRev;

int speedMultiplier = 100;
int speedSteps = 10;
float speed = speedMultiplier*steps_per_mm;
int dir = 1;

int acc_step_size = 500;
int acceleration = 200*steps_per_mm;

#include <AccelStepper.h>
AccelStepper stepper = AccelStepper(stepper.DRIVER, STEP_PIN, DIR_PIN);

void setup() {
    SPI.begin();
    Serial.begin(9600);
    while(!Serial);
    Serial.println("Start...");
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    driver.begin();             // Initiate pins and registeries
    driver.rms_current(600);    // Set stepper current to 600mA. The command is the same as command TMC2130.setCurrent(600, 0.11, 0.5);
    driver.en_pwm_mode(1);      // Enable extremely quiet stepping
    driver.pwm_autoscale(1);
    driver.microsteps(microsteps);

    stepper.setMaxSpeed(speed); // 200mm/s @ 80 steps/mm
    stepper.setAcceleration(acceleration); // 4000mm/s^2
    stepper.setEnablePin(EN_PIN);
    stepper.setPinsInverted(false, false, true);
    stepper.enableOutputs();
}


void printSettings(){
  Serial.print("MaxSpeed: ");
  Serial.print(speed);
  Serial.print(" Acceleration: ");
  Serial.print(acceleration);
  Serial.print(" Steps per mm: ");
  Serial.print(steps_per_mm);
  Serial.print(" FullStepsPerRev: ");
  Serial.print(fullStepsPerRev);
  Serial.print(" RPM: ");
  float rpm = speed/fullStepsPerRev*60.0;
  Serial.println(rpm);

}

void changeMaxSpeed(int toAdd){
  stepper.disableOutputs();
  speedMultiplier += toAdd;
  speed = speedMultiplier*steps_per_mm;
  stepper.setMaxSpeed(speed);
  stepper.enableOutputs();
  printSettings();
}

void changeAcceleration(int toAdd){
  stepper.disableOutputs();  
  acceleration += toAdd;
  stepper.setAcceleration(acceleration);
  stepper.enableOutputs();
  printSettings();
}


void loop() {
    static uint32_t last_time=0;
    uint32_t ms = millis();

    while(Serial.available() > 0) {
      int8_t read_byte = Serial.read();
    
      //if (read_byte == '0' && digitalRead(EN_PIN) == LOW)      { 
      //  myTimer.end();
      //  digitalWrite( EN_PIN, HIGH ); 
      //}else if (read_byte == '1' && digitalRead(EN_PIN) == HIGH) {         
      //  myTimer.begin(timerFunction, timer_speed);
      //  digitalWrite( EN_PIN,  LOW ); 
      //}else 
      if (read_byte == '+') {
        changeMaxSpeed(speedSteps);
      }else if (read_byte == '-') { 
        changeMaxSpeed(speedSteps*-1);
      }else if (read_byte == '/') {
        changeAcceleration(acc_step_size);
      }else if (read_byte == '*') { 
        changeAcceleration(acc_step_size*-1);
      }
    }

    if (stepper.distanceToGo() == 0) {
        stepper.disableOutputs();
        delay(100);
        stepper.move(1000*steps_per_mm*dir); // Move 100mm
        stepper.enableOutputs();
        dir = dir*-1;
    }
    stepper.run();
}
