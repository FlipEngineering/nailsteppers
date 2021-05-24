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

constexpr uint32_t steps_per_mm = 80;

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
    driver.microsteps(16);

    stepper.setMaxSpeed(100*steps_per_mm); // 200mm/s @ 80 steps/mm
    stepper.setAcceleration(2000*steps_per_mm); // 4000mm/s^2
    stepper.setEnablePin(EN_PIN);
    stepper.setPinsInverted(false, false, true);
    stepper.enableOutputs();
}

void loop() {
    if (stepper.distanceToGo() == 0) {
        stepper.disableOutputs();
        delay(100);
        stepper.move(100*steps_per_mm); // Move 100mm
        stepper.enableOutputs();
    }
    stepper.run();
}
