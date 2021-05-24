#include <TMCStepper.h>

#define EN_PIN           8 // Enable
#define DIR_PIN          7 // Direction
#define STEP_PIN         6 // Step
#define CS_PIN           10 // Chip select
#define SW_MOSI          11 // Software Master Out Slave In (MOSI)
#define SW_MISO          12 // Software Master In Slave Out (MISO)
#define SW_SCK           13 // Software Slave Clock (SCK)


#define R_SENSE 0.11f // Match to your driver
TMC2130Stepper driver = TMC2130Stepper(CS_PIN, R_SENSE, SW_MOSI, SW_MISO, SW_SCK); // Software SPI

constexpr uint32_t steps_per_mm = 80;
int dir = 1;

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
    driver.microsteps(4);

	float acceleration = 150000;
	float max_speed = 20000; // if ramp up = 0.25sec then max acc * 4 right (?)

    stepper.setMaxSpeed(max_speed); // 200mm/s @ 80 steps/mm
    stepper.setAcceleration(acceleration); // 20000steps/sec² with 4000mm/sec² and 20gears at 2mm pitch (?)
	
    stepper.setEnablePin(EN_PIN);
    stepper.setPinsInverted(false, false, true);
    stepper.enableOutputs();
}

void loop() {
	while(!Serial);
    if (stepper.distanceToGo() == 0) {
        stepper.disableOutputs();
        delay(10);
		dir = dir * -1;
        stepper.move(dir*4000); // 5000 = 4 microstep res , 250mm dist @ 40mm gear, 200 full steps
        stepper.enableOutputs();
    }
    stepper.run();
}
