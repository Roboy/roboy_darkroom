#include "wirelessLove.hpp"


 const char  TestBuffer[] ="Hello World";
 const int    timestampSize = 2;

WirelessLove::WirelessLove(const char* SSID, const char* PASSWD, IPAddress &broadcastIP):
broadcastIP(broadcastIP){
    uint32_t timoutCounter = 0;

    // check for presence of the WiFiShield:
    if(WiFi.status() == WL_NO_SHIELD) {
        Serial.println("WiFi shield not present");
    }

    // wait max 10 seconds to connect to the provided Wifi
    while(WiFi.begin(SSID,PASSWD) != WL_CONNECTED){
        delay(1000);
        Serial.print("trying to connect to");
        Serial.println(SSID);
    }
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

    uint32_t UDPBufferSize = SOCKET_BUFFER_MTU;
    Serial.print("Buffer Size from UDP Socket: ");
    Serial.println(UDPBufferSize);
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


