/*
    BSD 3-Clause License

    Copyright (c) 2017, Roboy
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
    description: helper class for handling wifi related stuff
*/

#ifndef WIRELESSLOVE_H
#define WIRELESSLOVE_H

#include <WiFi101.h>
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
