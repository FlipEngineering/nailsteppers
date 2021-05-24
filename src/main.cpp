#ifndef CONFIG_FILE_INCLUDED
#include <config.h>
#endif 

#ifndef COMM_FILE_INCLUDED
#include <Comm.h>
#endif 

#include "NailStepper.h"
#include "dip.h"

unsigned long current_time = micros();
unsigned long loop_time = micros();
unsigned long base_loop_time = micros();
unsigned long artnet_loop_time_start_time = micros();
unsigned long art_sleep_start = micros();
unsigned long art_sleep_now = micros();
unsigned long art_sleep_goal = micros();

unsigned long recalc_loop_time = micros();

unsigned long artnet_loop_time = micros();
unsigned long step_loop_time = micros();
long double time_per_step = 0.0;
unsigned long artnet_frame_rate = micros();
unsigned long artnet_frame_rate_diff = micros();
int last_no_artnet_iterations = 0;
int no_artnet_iterations = 0;


unsigned long main_loop_ms1 = millis();
unsigned long main_loop_ms2 = millis();
unsigned long main_loop_diff = 0;
long double diff_collect = 0;
int loop_counter_1 = 0;
int loop_counter_2 = 0;
unsigned long max_loops[4] = {TICK_TIMER_MAX_INIT,TICK_TIMER_MAX_INIT,TICK_TIMER_MAX_INIT,TICK_TIMER_MAX_INIT};
unsigned long min_loops[4] = {TICK_TIMER_MIN_INIT,TICK_TIMER_MIN_INIT,TICK_TIMER_MIN_INIT,TICK_TIMER_MIN_INIT};
unsigned long current_loops[4] = {0,0,0,0};
unsigned long loop_diffX = 0;

bool error_stop = false;
uint16_t artnet_read;

int step_mode = STEP_MODE;

NailStepper *nails[CONNECTED_STEPPERS];

Dmx *dmx = new Dmx();

void onDmxFrame(uint16_t universe, uint16_t length, uint8_t sequence, uint8_t* data, IPAddress remoteIP)
{
	if(universe == dmx->dmxAdd.active_universe){   
		dmx->last_ms = dmx->ms;  
		dmx->ms = millis(); 
		dmx->diff_ms = dmx->ms - dmx->last_ms; 
		for(int i=0; i<ARTNET_DATA_LENGTH/ARTNET_RESOLUTION; i++){
			if(ARTNET_RESOLUTION == RES_DOUBLE){
			int a = data[dmx->dmxAdd.channel_start+(i*ARTNET_RESOLUTION)]; 
			int b = data[dmx->dmxAdd.channel_start+(i*ARTNET_RESOLUTION)+1];
			dmx->received_data[i] = a << 8;    
			dmx->received_data[i] = dmx->received_data[i] | b;
			}else if(ARTNET_RESOLUTION == RES_SINGLE){
			dmx->received_data[i] = data[dmx->dmxAdd.channel_start+(i*ARTNET_RESOLUTION)];
			}else{
				if(DEBUG_OUTPUT){
					if(DEBUG_OUTPUT)Serial.println("------------Artnet-Error--------");
					if(DEBUG_OUTPUT)Serial.println("Artnet Resolution not supported!");
					if(DEBUG_OUTPUT)Serial.println("------------Artnet-Error--------");
				}
			}
		}
		
		dmx->settings_update(data);

		for(int i=0;i<dmx->settings_count;i++){
		dmx->settings_channels[i][1] = data[dmx->dmxAdd.channel_start+dmx->settings_channels[i][0]];
		}

	}

}

void tick(){
	for(int i=0;i<CONNECTED_STEPPERS;i++){		
		
		if(step_mode == MODE_RAIN){
				current_loops[i] = nails[i]->tick_continues_dir();
		
		}else if(ACTIVE_TICK == TICK_MOTION_GEN){		
			 current_loops[i] = nails[i]->tick_motion_gen();	
		}else if(ACTIVE_TICK == TICK_NEW){
			current_loops[i] = nails[i]->tick();			
		}else if (ACTIVE_TICK == TICK_OLD){
			current_loops[i] = nails[i]->tick_old();			
		}
		
		if(current_loops[i] > max_loops[i] && current_loops[i] < TICK_TIMER_UPPER_LIMIT){
			max_loops[i] = current_loops[i];
		}
		if(current_loops[i] < min_loops[i]){
			min_loops[i] = current_loops[i];
		}
	}

	if(step_mode == MODE_RAIN && 1==0){
		const char *spacer = ",";
		if(DEBUG_OUTPUT)Serial.print(" avg:");
		if(DEBUG_OUTPUT)Serial.print(nails[0]->avg_step_time);Serial.print(spacer);Serial.print(nails[1]->avg_step_time);Serial.print(spacer);Serial.print(nails[2]->avg_step_time);Serial.print(spacer);Serial.print(nails[3]->avg_step_time);
		if(DEBUG_OUTPUT)Serial.print(" st:");
		if(DEBUG_OUTPUT)Serial.print(nails[0]->t_current_pos_step_diff);Serial.print(spacer);Serial.print(nails[1]->t_current_pos_step_diff);Serial.print(spacer);Serial.print(nails[2]->t_current_pos_step_diff);Serial.print(spacer);Serial.print(nails[3]->t_current_pos_step_diff);
		if(DEBUG_OUTPUT)Serial.print(" dt:");
		if(DEBUG_OUTPUT)Serial.print(nails[0]->loop_ms1_diff);Serial.print(spacer);Serial.print(nails[1]->loop_ms1_diff);Serial.print(spacer);Serial.print(nails[2]->loop_ms1_diff);Serial.print(spacer);Serial.print(nails[3]->loop_ms1_diff);
		if(DEBUG_OUTPUT)Serial.print(" RPM:");
		if(DEBUG_OUTPUT)Serial.print(nails[0]->current_rpm);Serial.print(spacer);Serial.print(nails[1]->current_rpm);Serial.print(spacer);Serial.print(nails[2]->current_rpm);Serial.print(spacer);Serial.print(nails[3]->current_rpm);
		if(DEBUG_OUTPUT)Serial.print(" NoArt:"); Serial.println(no_artnet_iterations);
	}

	if(DISPLAY_TICK_TIMER && DEBUG_OUTPUT){
		Serial.printf("%lu",max_loops[0]); Serial.print(" - ");
		Serial.printf("%lu",max_loops[1]); Serial.print(" - ");
		Serial.printf("%lu",max_loops[2]); Serial.print(" - ");
		Serial.printf("%lu",max_loops[3]); Serial.print(" -----");
		Serial.printf("%lu",current_loops[0]); Serial.print(" - ");
		Serial.printf("%lu",current_loops[1]); Serial.print(" - ");
		Serial.printf("%lu",current_loops[2]); Serial.print(" - ");
		Serial.printf("%lu",current_loops[3]); Serial.print(" -----");
		Serial.printf("%lu",min_loops[0]); Serial.print(" - ");
		Serial.printf("%lu",min_loops[1]); Serial.print(" - ");
		Serial.printf("%lu",min_loops[2]); Serial.print(" - ");
		Serial.printf("%lu",min_loops[3]); Serial.print(" -----");
		Serial.println(" ");
	}
}

void stepper_setup(){
	int steppers[CONNECTED_STEPPERS][STEPPER_CONFIG_COUNT] = STEPPERS;

	for(int i=0;i<CONNECTED_STEPPERS;i++){
		nails[i] =  new NailStepper(steppers[i][CS_IDX], SW_MOSI, SW_MISO, SW_SCK, steppers[i][STEP_IDX], steppers[i][DIR_IDX], steppers[i][EN_IDX], MOTOR_MAX_SPEED, MOTOR_MAX_ACC);
		nails[i]->dmx = dmx;
		nails[i]->id=i;
		nails[i]->artnet_resolution = ARTNET_RESOLUTION;
		nails[i]->default_distance_mm = STEPPER_DEFAULT_DISTANCE_MM;

		nails[i]->setDistance(steppers[i][GEAR_IDX], BELT_PITCH, STEPPER_FULL_STEPS_PER_REV);
		if(dmx->settings_active_init){
			nails[i]->setGear();
			nails[i]->setDriver();
			step_mode = nails[i]->setMode();
		}
		
		if(STEPPER_SCAN && step_mode == MODE_NAIL){
			nails[i]->scanDistance();
		}else{
		
			nails[i]->scanInit();
			nails[i]->full_distance = STEPPER_DEFAULT_DISTANCE_MM;
		}
		if(dmx->settings_active_init){
			nails[i]->setFullDistance();
		}

		if(STEPPER_SCAN_OFFSET && STEPPER_SCAN && step_mode == MODE_NAIL){
			nails[i]->offsetEndstops(STEPPER_SCAN_OFFSET_DISTANCE, STEPPER_SCAN_OFFSET_DIRECTION);
		}
		nails[i]->finishInit();		
		nails[i]->prepareRun(MOTOR_MAX_SPEED, MOTOR_MAX_ACC, PULL_IN_SPEED_MM, steppers[i][INV_DIR_IDX]);
		
	}
	
}


void setup(){
    if(DEBUG_OUTPUT)Serial.begin(SERIAL_BAUD);         // Init serial port and set baudrate
    //while(!Serial);               // Wait for serial port to connect
    if(DEBUG_OUTPUT)Serial.println("\nStart...");
    
	SPI.begin();
    pinMode(MISO, INPUT_PULLUP); 


	dmx->dmxAdd.channel_start = 0;

	if(!ETHERNET_EVAL){
		//Initialize Steppers and scan distance
		delay(SETUP_INIT_DELAY);
	
		//Initialize Artnet Communication
		//artnet_data_length = CONNECTED_STEPPERS*ARTNET_RESOLUTION; //Comm.h data length	
		dmx->ethernet_setup();
    	dmx->artnet->setArtDmxCallback(onDmxFrame);
		
		if(DIP_ENABLED){
			if(dip_setup()){
				dmx->dmxAdd.channel_start = dip_loop();
			}
		}else{
			dmx->dmxAdd.channel_start = (dmx->controller_id * ARTNET_DATA_LENGTH) + (dmx->controller_id * DMX_ADDITIONAL_SETTINGS) + (dmx->controller_id * DMX_EMPTY_BUFFER);
			if(DEBUG_OUTPUT)Serial.printf("Channel Start: %d \n", dmx->dmxAdd.channel_start);
		}
		dmx->settings_init(); //initAdditionalSettings();

		//Read at least one time dmx settings
		if(DEBUG_OUTPUT)Serial.println("Starting looking for a DMX input:");
		while(artnet_read == 0){
			artnet_read = dmx->ethernet_loop();

			if(artnet_read != 0){
				if(dmx->settings_get(DMX_SETTINGS_ACTIVE_INIT) == DMX_SETTINGS_ACTIVE_INIT_VALUE){				
					dmx->settings_active_init = true;
				}
				if(DEBUG_OUTPUT)Serial.printf("DMX Settings Init: %d \n", dmx->settings_active_init);
				
				if(dmx->settings_get(DMX_SETTINGS_ACTIVE_ALWAYS) == DMX_SETTINGS_ACTIVE_ALWAYS_VALUE){
					dmx->settings_active_always = true;
				}
				if(DEBUG_OUTPUT)Serial.printf("DMX Settings Always: %d \n", dmx->settings_active_always);
			}
		}
		if(DEBUG_OUTPUT)Serial.println("Fetched at least one dmx signal!");
		if(DEBUG_OUTPUT)Serial.println("Fetched at least one dmx signal!");
		if(DEBUG_OUTPUT)Serial.println("Fetched at least one dmx signal!");
		
		
		if(DEBUG_OUTPUT)Serial.println("Stepper Setup Begin");
		stepper_setup();		
		if(DEBUG_OUTPUT)Serial.println("Stepper setup end");

	}else{
		// ETHERNET_EVAL ONLY ---- fake data, stuff doesn't really matter
		//Initialize Artnet Communication
		//artnet_data_length = 1*ARTNET_RESOLUTION; //Comm.h data length	
		dmx->ethernet_setup();
    	dmx->artnet->setArtDmxCallback(onDmxFrame);

	}
	if(DEBUG_OUTPUT){
		Serial.print("Channel Start DMX:");
		Serial.println(dmx->dmxAdd.channel_start);
	}

	//addSettingsChannel(DMX_SETTING_GEAR_COUNT);
	//addSettingsChannel(9);
	//addSettingsChannel(10);
	//addSettingsChannel(11);
	//addSettingsChannel(12);
	//addSettingsChannel(13);
	//addSettingsChannel(14);
	//addSettingsChannel(15);
	//addSettingsChannel(16);
}

void loop_monitor(){
	main_loop_ms2 = main_loop_ms1;
	main_loop_ms1 = micros();
	main_loop_diff = main_loop_ms1 - main_loop_ms2;
	
	if(loop_counter_1 == 0){ //avg 10
		loop_counter_2++;
		diff_collect += main_loop_diff;
		if(loop_counter_2 == 1000){
			diff_collect = diff_collect / float(loop_counter_2);
			if(DEBUG_OUTPUT){
				Serial.println(int(diff_collect));
			}	
			loop_counter_1 = 1;
			loop_counter_2 = 0;
			diff_collect = 0;
		}
	}else if(loop_counter_1 == 1){ //avg 100
		loop_counter_2++;
		diff_collect += main_loop_diff;
		if(loop_counter_2 == 1000){
			diff_collect = diff_collect / float(loop_counter_2);
			if(DEBUG_OUTPUT){
				Serial.println(int(diff_collect));
			}		
			loop_counter_1 = 2;
			loop_counter_2 = 0;
			diff_collect = 0;
		}
	}else if(loop_counter_1 == 2){ //avg 1000
		loop_counter_2++;
		diff_collect += main_loop_diff;
		if(loop_counter_2 == 1000){
			diff_collect = diff_collect / float(loop_counter_2);
			if(DEBUG_OUTPUT){
				Serial.println(int(diff_collect));
			}		
			loop_counter_1 = 0;
			loop_counter_2 = 0;
			diff_collect = 0;
		}
	}
}

void loop(){
	//nails[0]->testQueue();
	//nails[0]->testVector();
	//return;
	//delay(10000);

	//while(!error_stop){
		if(MONITOR_LOOP_TIME){
			loop_monitor();
		}

		int all_loop_steps = 0;
		
		//Read incoming Artnet Data
		//reading into received_data[];

		artnet_loop_time_start_time = micros();
		if(DEBUG_OUTPUT)Serial.println("Art1");
		artnet_read = dmx->ethernet_loop();
		if(DEBUG_OUTPUT)Serial.println("Art2");
		if( artnet_read != 0){
			
			if(DEBUG_OUTPUT)Serial.println("L1");
			
			last_no_artnet_iterations = no_artnet_iterations;
			no_artnet_iterations = 0;
			artnet_frame_rate_diff = micros() - artnet_frame_rate;
			artnet_frame_rate = micros();
		//Update Position information for each nail
			for(int i=0;i<CONNECTED_STEPPERS;i++){
				//if(received_data[i] > 255){
				//	error_stop = true;
				//	Serial.println("Received invalid data!");
				//	Serial.println(received_data[i]);
				//}
				//Serial.println(received_data[i]);
				nails[i]->ts_incoming = nails[i]->ts_incoming_buffer;
				nails[i]->target_pos = nails[i]->target_pos_buffer;
				nails[i]->ts_incoming_buffer = micros();
				nails[i]->dmx_raw = (double)dmx->received_data[i];
				nails[i]->target_pos_buffer = (double)dmx->received_data[i];
				nails[i]->target_position_updated = true;
				nails[i]->target_position_updated_counter++;
				
				//nails[i]->dmx_settings_count = dmx->settings_count;
				//for(int j=0;j<dmx->settings_count;j++){
				//	nails[i]->dmx_settings_channels_old[j][0] = nails[i]->dmx_settings_channels[j][0]; 
				//	nails[i]->dmx_settings_channels_old[j][1] = nails[i]->dmx_settings_channels[j][1]; 
				//	nails[i]->dmx_settings_channels[j][0] = dmx->settings_channels[j][0];
				//	nails[i]->dmx_settings_channels[j][1] = dmx->settings_channels[j][1];
				//}

				//if(i == 0){
				//	nails[i]->handle_dmx_settings();
				//}
				
				//Serial.print(" R:");
				//Serial.print(received_data[i]);
				//Serial.print(" ");
				//Serial.println(nails[i]->dmx_step_size);
				if(DEBUG_OUTPUT)Serial.printf("Gp%d s\n",i);
				nails[i]->current_pos = nails[i]->stepper.getPosition();
				if(DEBUG_OUTPUT)Serial.printf("Gp%d e\n",i);

				all_loop_steps += nails[i]->last_loop_steps;

			}

			//Only run DMX Stepper update Config check if all steppers are standing still
			if(all_loop_steps == 0){
				for(int i=0; i<CONNECTED_STEPPERS;i++){
					nails[i]->dmxSettingsUpdate();
				}
			}

		}else{
			
			if(DEBUG_OUTPUT)Serial.println("L2");

			no_artnet_iterations++;
			art_sleep_now = micros() - current_time;
			art_sleep_goal = recalc_loop_time;
			while(art_sleep_now < art_sleep_goal){
				delayMicroseconds(10);
				//Serial.print(".");
				art_sleep_now = micros() - current_time;
				//delayMicroseconds(art_sleep_now);
				//if(art_sleep_now > 12){
					
				//}
			}

			//delayMicroseconds(MAIN_LOOP_NO_ARTNET_READ_DELAY);
		}
		artnet_loop_time = micros()-artnet_loop_time_start_time;

		loop_time = micros() - current_time;
		if(no_artnet_iterations == 0){
			recalc_loop_time = loop_time;
		}

		current_time = micros();

		if(all_loop_steps == 0 ){
			//not moving loop time (calculating out the artnet loop time)
			base_loop_time = loop_time-artnet_loop_time;
		}else{
			//with moving loop time
			int tmp = loop_time-base_loop_time-artnet_loop_time;
			if(tmp < 0){
				tmp = 0;
			}
			step_loop_time = tmp;
			time_per_step = step_loop_time / all_loop_steps;

			if(artnet_read == 0){
				// with step loop time - without artnet
			}else{
				// with step loop time - with artnet
			}
			
		}
/*
Full Loop Time:452 Base:49 Step:0 Time Per Step:0.00 Artnet:404 STEPS:1 ARTNET FPS:16158 No Artnet Iters:34
Full Loop Time:459 Base:49 Step:2 Time Per Step:1.00 Artnet:408 STEPS:2 ARTNET FPS:15385 No Artnet Iters:32
Full Loop Time:453 Base:49 Step:0 Time Per Step:0.00 Artnet:407 STEPS:4 ARTNET FPS:31428 No Artnet Iters:66
Full Loop Time:454 Base:48 Step:0 Time Per Step:0.00 Artnet:406 STEPS:5 ARTNET FPS:15589 No Artnet Iters:32
Full Loop Time:456 Base:51 Step:1 Time Per Step:0.00 Artnet:404 STEPS:6 ARTNET FPS:16603 No Artnet Iters:34
Full Loop Time:458 Base:47 Step:5 Time Per Step:0.00 Artnet:406 STEPS:7 ARTNET FPS:29518 No Artnet Iters:61
Full Loop Time:454 Base:49 Step:0 Time Per Step:0.00 Artnet:406 STEPS:7 ARTNET FPS:15716 No Artnet Iters:32
Full Loop Time:453 Base:47 Step:1 Time Per Step:0.00 Artnet:405 STEPS:7 ARTNET FPS:16193 No Artnet Iters:33
Full Loop Time:459 Base:49 Step:2 Time Per Step:0.00 Artnet:408 STEPS:7 ARTNET FPS:14774 No Artnet Iters:30
Full Loop Time:450 Base:51 Step:0 Time Per Step:0.00 Artnet:405 STEPS:7 ARTNET FPS:31847 No Artnet Iters:66
Full Loop Time:451 Base:48 Step:0 Time Per Step:0.00 Artnet:405 STEPS:7 ARTNET FPS:15225 No Artnet Iters:31
Full Loop Time:453 Base:49 Step:0 Time Per Step:0.00 Artnet:406 STEPS:6 ARTNET FPS:15670 No Artnet Iters:32
Full Loop Time:453 Base:47 Step:0 Time Per Step:0.00 Artnet:406 STEPS:4 ARTNET FPS:31612 No Artnet Iters:66
Full Loop Time:454 Base:48 Step:0 Time Per Step:0.00 Artnet:407 STEPS:4 ARTNET FPS:15993 No Artnet Iters:33
Full Loop Time:455 Base:48 Step:2 Time Per Step:0.00 Artnet:405 STEPS:3 ARTNET FPS:15504 No Artnet Iters:32
Full Loop Time:452 Base:46 Step:0 Time Per Step:0.00 Artnet:407 STEPS:3 ARTNET FPS:14502 No Artnet Iters:30
Full Loop Time:451 Base:48 Step:0 Time Per Step:0.00 Artnet:406 STEPS:2 ARTNET FPS:31131 No Artnet Iters:66
*/

		if(all_loop_steps != 0 && 1 == 0 && DEBUG_OUTPUT){
			Serial.print("Full Loop Time:");
			Serial.print(loop_time);
			Serial.print(" Base:");
			Serial.print(base_loop_time);
			Serial.print(" Step:");
			Serial.print(step_loop_time);
			Serial.print(" Time Per Step:");
			Serial.print((double)time_per_step);
			Serial.print(" Artnet:");
			Serial.print(artnet_loop_time);
			Serial.print(" STEPS:");
			Serial.print(all_loop_steps);
			Serial.print(" ARTNET FPS:");
			Serial.print(artnet_frame_rate_diff);
			Serial.print(" No Artnet Iters:");
			Serial.println(last_no_artnet_iterations);
		}

		step_loop_time = 0;
		time_per_step = 0;
		artnet_loop_time = 0;

		if(DEBUG_OUTPUT)Serial.println("T1");
		tick();
		if(DEBUG_OUTPUT)Serial.println("T2");
		

		if(ETHERNET_EVAL){
			if(DEBUG_OUTPUT){
				Serial.println(dmx->received_data[0]);
				Serial.println("#####");
			}
			delay(1000);
			
		}
	//}
}

