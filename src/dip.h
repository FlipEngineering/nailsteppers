
#include "Grove_Multi_Switch.h"

#ifndef CONFIG_FILE_INCLUDED
#include <config.h>
#endif 

GroveMultiSwitch mswitch[1];
const char* grove_5way_tactile_keys[] = {
    "KEY A",
    "KEY B",
    "KEY C",
    "KEY D",
    "KEY E",
};
const char* grove_6pos_dip_switch_keys[] = {
    "POS 1",
    "POS 2",
    "POS 3",
    "POS 4",
    "POS 5",
    "POS 6",
};

const char** key_names;

int deviceDetect(void) {
    if (!mswitch->begin()) {
        if(DEBUG_OUTPUT)Serial.println("***** Device probe failed *****");
        return -1;
    }

    Serial.println("***** Device probe OK *****");
    if (PID_VAL(mswitch->getDevID()) == PID_5_WAY_TACTILE_SWITCH) {
        if(DEBUG_OUTPUT)Serial.println("Grove 5-Way Tactile Switch Inserted!");
        key_names = grove_5way_tactile_keys;
    } else if (PID_VAL(mswitch->getDevID()) == PID_6_POS_DIP_SWITCH) {
        if(DEBUG_OUTPUT)Serial.println("Grove 6-Position DIP Switch Inserted!");
        key_names = grove_6pos_dip_switch_keys;
    }

    // enable event detection
    mswitch->setEventMode(true);

    // report device model
    if(DEBUG_OUTPUT)Serial.print("A ");
    if(DEBUG_OUTPUT)Serial.print(mswitch->getSwitchCount());
    if(DEBUG_OUTPUT)Serial.print(" Button/Switch Device ");
    if(DEBUG_OUTPUT)Serial.println(mswitch->getDevVer());
    return 0;
}

bool dip_setup() {
    //Serial.begin(115200);
    if(DEBUG_OUTPUT)Serial.println("Grove Multi Switch - Setup - begin");

    // Initial device probe
    if (deviceDetect() < 0) {
        if(DEBUG_OUTPUT)Serial.println("No DIP Connected!");
        return false;
    }
    if(DEBUG_OUTPUT)Serial.println("Grove Multi Switch - Setup - done");
    return true;
}

int dip_loop() {
    GroveMultiSwitch::ButtonEvent_t* evt;

    //delay(1);

    evt = mswitch->getEvent();
    if (!evt) {
        // dynamic device probe
        deviceDetect();
        //delay(1000);
        if(DEBUG_OUTPUT)Serial.println("[DIP] no evt.");
        return 0;
    }

    int comp = 0b000001;
    int count = 0;
    for (int i = 0; i < mswitch->getSwitchCount(); i++) {

        if(!(evt->button[i] & GroveMultiSwitch::BTN_EV_RAW_STATUS)){
            count = count | comp;
        }
        comp = comp << 1;
        
    }
    //Serial.println(count);

    return count;
}
