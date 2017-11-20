#include "wirelessLove.hpp"

const char  TestBuffer[] ="Hello World";
const int    timestampSize = 2;

WirelessLove::WirelessLove(const char* ssid, const char* passwd, IPAddress &broadcastIP):
broadcastIP(broadcastIP){
  Serial.println();
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  
  /* Explicitly set the ESP8266 to be a WiFi-client, otherwise, it by default,
  would try to act as both a client and an access-point and could cause
  network-issues with your other WiFi-devices on your WiFi-network. */
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, passwd);
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  
  Serial.println("");
  Serial.println("WiFi connected");  
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  initUDPSockets();
}

uint32_t WirelessLove::getLocalIP()
{
    return  WiFi.localIP();
}

void WirelessLove::printWifiStatus(void)
{
    Serial.print("SSID: ");
    Serial.println(WiFi.SSID());

    IPAddress ip = WiFi.localIP();
    Serial.print("IP Address: ");
    Serial.println(ip);

    long rssi = WiFi.RSSI();
    Serial.print("signal strength (RSSI):");
    Serial.print(rssi);
    Serial.println(" dBm");

}


bool WirelessLove::initUDPSockets(void)
{
    return UDPbroadcast.begin(broadcast_port);
}

bool WirelessLove::broadcast_send(const uint8_t * buffer, size_t size)
{
    if(0 == UDPbroadcast.beginPacket(broadcastIP, broadcast_port))
    {
        Serial.println("Can not connect to the supplied IP or PORT");
        return  false;
    }

    if(size != UDPbroadcast.write(buffer, size)){
        Serial.println("Size of the UDP Package to big! Truncated overlapping data");
    }
    UDPbroadcast.endPacket();
    return true;
}


