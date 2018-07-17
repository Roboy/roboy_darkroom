### roboy_darkroom
* This repo contains everything you need for tracking our custom sensors or the original HTC sensors in space.
* The repo is structered like so:
* ![tree](https://github.com/Roboy/roboy_darkroom/blob/master/images/tree.png?raw=true "tree")


#### darkroom 
This is a ROS package containing all the code necessary for triangulation and pose estimation of the sensors. It's the heart of the tracking system. However, the body is roboy_rqt which uses the heart for meaningful stuff. The idea was, that users could integrate this ROS package into their own tracking system.

### darkroom_arduino
#### darkroom_esp8266
In order to receive the decoded sensor signal wirelessly via UDP, you will need an esp8266 (Nodemcu or similar are fine aswell). The sensor data is sent to the esp via SPI. The esp then sends a UDP to the host. This folder contains the arduino sketch for doing the above. You will most likely need to adapt the ssid/password/hostIP to your local lan.

#### darkroom_mkr
This is an arduino sketch actually decoding the sensor signals directly. This is kind of a relict from the early days of our endeavour. Be aware that the number of sensors you can decode is limited to approx 3. The results are also sent via UDP to the host. You will most likely need to adapt the ssid/password/hostIP to your local lan.

#### ootx_frame_decoder
This is an example arduino sketch for receiving the decoded ootx frames via Serial (uart). Depends on [SDFat](https://github.com/greiman/SdFat) and [CRC32](https://github.com/bakercp/CRC32).

### darkroom_rqt
This is the body of darkroom. It is a ROS package (and rqt plugin), using the darkroom code in a meaningful way. 

### roboy_de10_nano_soc
This is the fpga code decoding the sensor signals and sending the data via SPI to the esp. For building this you will need Quartus 16.0 or above. Checkout the README.md inside to get an idea what you need to do for building this.

### openvr
follow this tutorial http://help.triadsemi.com/steamvr-tracking/steamvr-tracking-without-an-hmd/

