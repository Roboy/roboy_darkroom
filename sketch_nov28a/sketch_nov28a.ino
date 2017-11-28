#include "SdFat.h"
#include "CRC32.h"

union OOTX{
  uint8_t data[39];
  struct {
    uint16_t fw_version;
    uint32_t ID;
    uint16_t fcal_0_phase;
    uint16_t fcal_1_phase;
    uint16_t fcal_0_tilt;
    uint16_t fcal_1_tilt;
    uint8_t unlock_count;
    uint8_t hw_version;
    uint16_t fcal_0_curve;
    uint16_t fcal_1_curve;
    int8_t accel_dir_x;
    int8_t accel_dir_y;
    int8_t accel_dir_z;
    uint16_t fcal_0_gibphase;
    uint16_t fcal_1_gibphase;
    uint16_t fcal_0_gibmag;
    uint16_t fcal_1_gibmag;
    uint8_t mode;
    uint8_t faults;
    uint16_t payload_length;
    uint32_t crc32;
  }frame;
}ootx;

int counter = 0;

ArduinoOutStream cout(Serial);

void setup() {
  // initialize both serial ports:
  Serial.begin(115200);
  Serial1.begin(115200);
  Serial.println("welcome");
  int counter = 0;
}

void loop() {
  // read from port 1, send to port 0:
  if (Serial1.available() == sizeof(ootx)) {
    CRC32 crc;
    while(counter<sizeof(ootx)){
      ootx.data[counter] = Serial1.read();
      if(counter<33)
        crc.update(ootx.data[counter]);
      counter++;
    }
    // Calculate the checksum.
    uint32_t checksum = crc.finalize();
    
    cout << "--------------------------------------" <<  endl;
    cout << "received ootx frame of length "  << (unsigned int)ootx.frame.payload_length << " with crc " << ootx.frame.crc32 << " calculated: " << checksum<< endl;
    cout << "fw_version:          " ; Serial.println(ootx.frame.fw_version,BIN);
    cout << "ID:                  " << ootx.frame.ID << endl;
    cout << "fcal_0_phase:        " << ootx.frame.fcal_0_phase << endl;
    cout << "fcal_1_phase:        " << ootx.frame.fcal_1_phase << endl;
    cout << "fcal_0_tilt:         " << ootx.frame.fcal_0_tilt << endl;
    cout << "fcal_1_tilt:         " << ootx.frame.fcal_1_tilt << endl;
    cout << "unlock_count:        " << (unsigned int)ootx.frame.unlock_count << endl;
    cout << "hw_version:          " << (unsigned int)ootx.frame.hw_version << endl;
    cout << "fcal_0_curve:        " << ootx.frame.fcal_0_curve << endl;
    cout << "fcal_1_curve:        " << ootx.frame.fcal_1_curve << endl;
    cout << "accel_dir_x:         " << (int)ootx.frame.accel_dir_x << endl;
    cout << "accel_dir_y:         " << (int)ootx.frame.accel_dir_y << endl;
    cout << "accel_dir_z:         " << (int)ootx.frame.accel_dir_z << endl;
    cout << "fcal_0_gibphase:     " << ootx.frame.fcal_0_gibphase << endl;
    cout << "fcal_1_gibphase:     " << ootx.frame.fcal_1_gibphase << endl;
    cout << "fcal_0_gibmag:       " << ootx.frame.fcal_0_gibmag << endl;
    cout << "fcal_1_gibmag:       " << ootx.frame.fcal_1_gibmag << endl;
    cout << "mode:                " << (unsigned int)ootx.frame.mode << endl;
    cout << "faults:              " << (unsigned int)ootx.frame.faults << endl;
    counter = 0;
  }
}
