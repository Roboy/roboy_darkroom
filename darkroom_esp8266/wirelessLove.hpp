#ifndef WIRELESSLOVE_H
#define WIRELESSLOVE_H

#include "ESP8266WiFi.h"
#include <WiFiUdp.h>

class WirelessLove{
public:
    /**
    * @param ssid name of wifi network
    * @param passwd the password
    * @param Bcast ip of your network (check with ifconfig)
    */
    WirelessLove(const char* ssid, const char* passwd, IPAddress &broadcastIP);
    /**
    * @return MKRs' IP
    */
    uint32_t getLocalIP();
    /**
     * initalizes udp broadcaster
     */
    bool initUDPSockets();
    /** prints information about the wifi network */
    void    printWifiStatus();
    /**
    * Broadcasts the config message
    * @param buffer the encoded protobuf buffer
    * @param size how big in bytes
    * @param success
    */
    bool     broadcast_send(const uint8_t * buffer, size_t size);

    WiFiUDP UDPbroadcast;
    IPAddress broadcastIP;
    int broadcast_port = 8000;
};


#endif
