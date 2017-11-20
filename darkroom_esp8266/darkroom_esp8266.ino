
/*
    DarkRoom SPI slave
    Connect the SPI Master device to the following pins on the esp8266:
    GPIO    NodeMCU   Name  |   Uno
  ===================================
     15       D8       SS   |   D10
     13       D7      MOSI  |   D11
     12       D6      MISO  |   D12
     14       D5      SCK   |   D13
    Note: If the ESP is booting at a moment when the SPI Master has the Select line HIGH (deselected)
    the ESP8266 WILL FAIL to boot!
    See SPISlave_SafeMaster example for possible workaround
*/

#include "SPISlave.h"
#include "wirelessLove.hpp"

WirelessLove *wifi;

volatile int sensor_value_counter = 0;

void printBits(uint32_t val){
  int i=0;
 for(uint32_t mask = 0x80000000; mask; mask >>= 1){
    if(i%8==0)
         Serial.print(' ');
    i++;
    if(mask  & val)
       Serial.print('1');
    else
       Serial.print('0');
 }
}

void setup() {
  Serial.begin(115200);

  Serial.println("");
  Serial.println("------------------------------------------------------");
  Serial.println("                    DARKROOM ESP");
  Serial.println("------------------------------------------------------");

  IPAddress hostIP(129,187,142,26); // UDP packets will be sent to this IP (you can use a broadcast IP aswell)
  wifi = new WirelessLove("roboy","wiihackroboy", hostIP); 
  wifi->initUDPSockets();

  /************** SET UP SPI SLAVE OF FPGA*****************/
  SPISlave.onData([](uint8_t * data, size_t len) {
        if(sensor_value_counter%1000==0){
          Serial.printf("received %d sensor frames, this frame:\n", sensor_value_counter);
          char str[33];
          for(uint i=0;i<len;i+=4){
            uint32_t val = uint32_t(data[i+3]<<24|data[i+2]<<16|data[i+1]<<8|data[i]);
            Serial.printf("%d:\t",i);
            printBits(val);
            Serial.println();
            Serial.printf("   \t%d\t%d\t%d\t%d\t\n", data[i+3], data[i+2], data[i+1], data[i] );
          }
      }
      wifi->broadcast_send(data, len);
      sensor_value_counter++;
  });
  SPISlave.begin();
}

void loop() {
  delay(1000);
  Serial.print("i am groot:\t");
  Serial.print(sensor_value_counter);
  Serial.println("\t spi frames processed, so far");
}
