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
