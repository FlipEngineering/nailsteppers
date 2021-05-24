#include <Arduino.h>

void test(){
    Serial.println("Test Called!");
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

/*

- Init SPI
- Check available distance 

*/