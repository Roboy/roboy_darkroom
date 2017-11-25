#pragma once

#include <fstream>
#include "darkroom/Sensor.hpp"
#include "yaml-cpp/yaml.h"
#include <sys/stat.h>


using namespace std;

class Utilities{
public:
/**
     * Reads a yaml tracked object file
     * @param filepath to
     * @return success
     */
    bool readConfig(string filepath, int &objectID, string &name, string &mesh,
                           vector<int> &calibrated_sensors, map<int, Sensor> &sensors);

    /**
     * Writes a tracked object to file
     * @param filepath
     * @return success
     */
    bool writeConfig(string filepath, int &objectID, string &name, string &mesh,
                     vector<int> &calibrated_sensors, map<int, Sensor> &sensors);

    /**
     * Checks if a file exists
     * @param name
     * @return exists
     */
    inline bool exists (const std::string& name) {
        struct stat buffer;
        return (stat (name.c_str(), &buffer) == 0);
    }
};