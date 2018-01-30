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
    description: example sketch for decoding sensor signal on th mkr1000
*/

#include "sensor.hpp"
#include "wirelessLove.hpp"


WirelessLove *wifi;
 
void setup() {
  Serial.begin(115200);      // initialize serial communication
  Serial.println("");
  Serial.println("------------------------------------------------------");
  Serial.println("                    DARKROOM MKR");
  Serial.println("------------------------------------------------------");
  initCounter();
  pinMode(6,INPUT); 
  pinMode(7,INPUT); 
  attachInterrupt(6, rising_IRQ_S1, RISING); 
  attachInterrupt(7, falling_IRQ_S1, FALLING); 

  IPAddress hostIP(129,187,142,26); // UDP packets will be sent to this IP (you can use a broadcast IP aswell)
  wifi = new WirelessLove("roboy","wiihackroboy", hostIP); 
  wifi->initUDPSockets();
}

void loop() {
  Sweep * detectedSweep = FIFO128_read(sweepFIFO); 
  while( NULL != detectedSweep){
      Serial.print("sweepDuration ");
      Serial.print(detectedSweep->sweepDuration);
      Serial.print(" lighthouse ");
      Serial.print(detectedSweep->lighthouse);
      Serial.print(" rotor ");
      Serial.println(detectedSweep->rotor);
      wifi->broadcast_send((uint8_t*)detectedSweep, sizeof(Sweep));
      free(detectedSweep); 
      detectedSweep = FIFO128_read(sweepFIFO); 
  }
}
