// #include "FastGPIO.h"
// #include "Wire.h"
#include <Telemetry.h>
#include <Arduino.h>

namespace Telemetry{
  // Initialize the logger module
  void backendInit() {}
  void backendUpdateBegin() {}
  void backendUpdateEnd() {}

  unsigned int available() { 
    unsigned int test = Serial.available();
    // Serial.print(test);
    // Serial.print(" test\n");
    return test; 
  }
  unsigned int availableForWrite() { return Serial.availableForWrite(); }

  void read(uint8_t *buffer, unsigned int size) {  
    Serial.readBytes(buffer, size); 
    // for(int i = 0; i < size; i ++) {
    //   Serial.print(buffer[i]);
    //   Serial.print(" ");
    // }
    // Serial.print(" | ");
  }
  void writePacket(packetHeader header, const uint8_t *buffer, unsigned int size) {
    Serial.write((uint8_t*)&header, sizeof(packetHeader));
    Serial.write(buffer, size);
  }

  bool getNextHeader(packetHeader *header) {
    static packetHeader lastHeader; 
    static bool splitLastPacket;

    
    if(!splitLastPacket) {
      if(available() < sizeof(packetHeader)) {
        return false;
      }

      Serial.readBytes((uint8_t*)&lastHeader, sizeof(packetHeader));
    }
    
    if(available() < dataSize(lastHeader.id)){
      splitLastPacket = true;
      return false;
    }

    splitLastPacket = false;
    *header = lastHeader;
    return true;
  }

  void* allocate(packet_id id ) { return malloc(dataSize(id)); }
  void deallocate(packet_id id ) { return free(data_values[id]); }

  void debug(const char* string) {/* Serial.print(string); */}
}
