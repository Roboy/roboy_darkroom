/*
    BSD 3-Clause License

    Copyright (c) 2018, Roboy
            All rights reserved.

    Redistribution and use in source and binary forms, with or without
            modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice, this
    list of conditions and the following disclaimer.

    * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

    * Neither the name of the copyright holder nor the names of its
    contributors may be used to endorse or promote products derived from
    this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
            IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
            FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
            DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
            SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
            CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    author: Simon Trendel ( simon.trendel@tum.de ), 2017
    description: arduino sketch showing how to receive the darkroom frames via spi and send them via UDP to host.

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

  IPAddress hostIP(192,168,255,255); // UDP packets will be sent to this IP (you can use a broadcast IP aswell)
  wifi = new WirelessLove("roboy","wiihackroboy", hostIP); 
  wifi->initUDPSockets();

  /************** SET UP SPI SLAVE OF FPGA*****************/
  SPISlave.onData([](uint8_t * data, size_t len) {
        if(sensor_value_counter%500==0){
          Serial.printf("received %d sensor frames, this frame:\n", sensor_value_counter);
          char str[33];
          for(uint i=0;i<len;i+=4){
            uint32_t val = uint32_t(data[i+3]<<24|data[i+2]<<16|data[i+1]<<8|data[i]);
            Serial.printf("sensor %d",(val>>19) & 0x3FF);
            printBits(val);
            Serial.println();            
            Serial.printf("   \tlighthouse %d\t rotor %d\t valid %d\t duration %d\t\t angle ", (val >> 31) & 0x1, (val >> 31) & 0x1, (val >> 29) & 0x1, val & 0x7FFFF);
            Serial.println( (uint32_t)(val & 0x7FFFF)* 0.021600864 / 16.0);
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
