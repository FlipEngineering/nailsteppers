#include <Ethernet.h>
#include <EthernetUdp.h>

#include <TeensyID.h>
#include <Artnet.h>
#include <map>
#include <String>

#ifndef CONFIG_FILE_INCLUDED
#include <config.h>
#endif 

#ifndef COMM_FILE_INCLUDED
#define COMM_FILE_INCLUDED 1

class Dmx{
  public:
  
  dmxAdditional dmxAdd;
  int controller_id = 0;
  
  uint32_t ms = millis();
  uint32_t last_ms;
  uint32_t diff_ms;
  byte mac[6];
  Artnet *artnet = new Artnet();
  
  int received_position = 0;
  int received_data[CONNECTED_STEPPERS];
  int settings_channels[DMX_MAX_SETTINGS_CHANNELS][2];
  int settings_count = 0;

  bool settings_active_init = false;
  bool settings_active_always = false;

  unsigned int localPort = COM_PORT;
  char packetBuffer[UDP_TX_PACKET_MAX_SIZE]; 
  EthernetUDP Udp;

  int settings_getChannel(const char *p);
  int settings_get(const char *p);
  void set(const char *p, int val);
  void settings_update(uint8_t* data);
  void settings_init();
  void addSettingsChannel(int channel);
  void ethernet_setup();
  uint16_t ethernet_loop();
};

#endif 


