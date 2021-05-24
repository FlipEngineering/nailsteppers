#include "Comm.h"


int Dmx::settings_getChannel(const char *p){
    return dmxAdd.settings[p].first;
}

int Dmx::settings_get(const char *p){
    return dmxAdd.settings[p].second;
}

void Dmx::set(const char *p, int val){
    dmxAdd.settings[p].second = val;
}

void Dmx::settings_update(uint8_t* data){
    for (auto itr = dmxAdd.settings.begin(); itr != dmxAdd.settings.end(); ++itr) {
        int tmpChan = dmxAdd.settings[itr->first].first;
        int tmpVal  = data[tmpChan];
        //Serial.printf("Updating %s on Channel %d with value %d \n", itr->first, tmpChan, tmpVal);
        dmxAdd.settings[itr->first].second = tmpVal;
    }
}

void Dmx::settings_init(){


    //Indexing the Settings Channel according to the Controller channel_start and stepper data length
    int channel_offset = dmxAdd.channel_start + ARTNET_DATA_LENGTH;
    for (auto itr = dmxAdd.settings.begin(); itr != dmxAdd.settings.end(); ++itr) {
        if(DEBUG_OUTPUT)Serial.print(dmxAdd.settings[itr->first].first);
        if(DEBUG_OUTPUT)Serial.print(" - ");
        dmxAdd.settings[itr->first].first += channel_offset;

        if(DEBUG_OUTPUT)Serial.println(dmxAdd.settings[itr->first].first);
        //dmxAdd.settings[itr->first].second = itr->second.second+5;
    }
}


void Dmx::addSettingsChannel(int channel){

    // dmx_settings[DMX_SETTING_GEAR_COUNT] = 4;
    settings_channels[settings_count][0] = channel;
    settings_count++;
    if(DEBUG_OUTPUT)Serial.print("Added Channel:");
    if(DEBUG_OUTPUT)Serial.print(channel);
    if(DEBUG_OUTPUT)Serial.print(" to Settings Channels");
}


void Dmx::ethernet_setup() {
    // Get Teensy MAC address: 04-E9-E5-xx-xx-xx for PJRC.com
    teensyMAC(mac);

    //not nice, but will work ish
    int tmpM = mac[5];
    if(tmpM == 0 || tmpM == 1){
        tmpM = mac[4];
    }else if(tmpM == TEENSY_0_MAC_END){
        dmxAdd.ip[3] = TEENSY_0_IP_END; // Controller 0
    }else if(tmpM == TEENSY_1_MAC_END){
        dmxAdd.ip[3] = TEENSY_1_IP_END; // Controller 0
    }else if(tmpM == TEENSY_2_MAC_END){
        dmxAdd.ip[3] = TEENSY_2_IP_END; // Controller 0
    }else if(tmpM == TEENSY_3_MAC_END){
        dmxAdd.ip[3] = TEENSY_3_IP_END; // Controller 0
    }else if(tmpM == TEENSY_4_MAC_END){
        dmxAdd.ip[3] = TEENSY_4_IP_END; // Controller 0
    }else if(tmpM == TEENSY_5_MAC_END){
        dmxAdd.ip[3] = TEENSY_5_IP_END; // Controller 0
    }else if(tmpM == TEENSY_6_MAC_END){
        dmxAdd.ip[3] = TEENSY_6_IP_END; // Controller 0
    }else{
        dmxAdd.ip[3] = tmpM;
    }

    if(dmxAdd.ip[3] >= 10){
        controller_id = (dmxAdd.ip[3]-10);
        if(DEBUG_OUTPUT)Serial.printf("Controller ID: %d \n", controller_id);
    }



    if(DEBUG_OUTPUT)Serial.println("Initalize Ethernet"); // with static IP derived from MAC address:");
    Ethernet.init(10);  // Most Arduino shields
    Ethernet.begin(mac, dmxAdd.ip, dmxAdd.dns, dmxAdd.gateway, dmxAdd.subnet);
    delay(3000);
    if(DEBUG_OUTPUT)Serial.printf("Channel Start: %d", dmxAdd.channel_start);
    if(DEBUG_OUTPUT)Serial.printf("\tMAC Address: %02X:%02X:%02X:%02X:%02X:%02X \n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    if(DEBUG_OUTPUT)Serial.printf("\tMAC Address int: %d:%d:%d:%d:%d:%d \n", (int)mac[0], (int)mac[1], (int)mac[2], (int)mac[3], (int)mac[4], (int)mac[5]);
    //Serial.printf("\tIP Address: %d.%d.%d.%d \n", (int)ip[0], (int)ip[1], (int)ip[2], (int)ip[3]);

    if(DEBUG_OUTPUT)Serial.printf("\tIP Address: %d.%d.%d.%d \n", (int)dmxAdd.ip[0], (int)dmxAdd.ip[1], (int)dmxAdd.ip[2], (int)dmxAdd.ip[3]);
    //Serial.print("\tIP address: ");
    //Serial.println(ip);
    if(DEBUG_OUTPUT)Serial.println("Beginning Artnet 1");  

    artnet->begin(mac, dmxAdd.ip);

    if(DEBUG_OUTPUT)Serial.println("Beginning Artnet 2");



    if(DEBUG_OUTPUT)Serial.println("Beginning Artnet 3");
    if(DEBUG_OUTPUT)Serial.println("Ethernet Setup done");
}

uint16_t Dmx::ethernet_loop() {
    return artnet->read();
}
