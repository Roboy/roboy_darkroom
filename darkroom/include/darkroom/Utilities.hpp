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

    author: Simon Trendel ( simon.trendel@tum.de ), 2018
    description: helper class for reading and writing tracking object information from to file
*/

#pragma once

#include <fstream>
#include "darkroom/Triangulation.hpp"
#include "yaml-cpp/yaml.h"
#include <sys/stat.h>
#include <boost/filesystem.hpp>

using namespace std;
namespace fs = boost::filesystem;

struct ModelInformation{
    string name;
    fs::path rootDirectory;
    fs::path lighthouseDirectory;
    fs::path meshDirectory;
    fs::path modelSdf;
    vector<fs::path> lighthouseConfigFiles;
    vector<fs::path> meshes;
};

class Utilities{
public:
    /**
     * Scans the provided directory for a model and fills a structure with information about it
     * @param modelDirectory path to model
     * @param info filled with information about the model
     * @return success
     */
    bool readModelDirectory(string modelDirectory, ModelInformation &info);
    /**
     * Reads a yaml tracked object file
     * @param filepath to
     * @return success
     */
    bool readConfig(fs::path filepath, string &objectID, string &name, fs::path &mesh,
                           vector<int> &calibrated_sensors, map<int, Sensor> &sensors,
                    map<int,vector<double>> &calibrationAngles);

    /**
     * Writes a tracked object to file
     * @param filepath
     * @return success
     */
    bool writeConfig(fs::path filepath, string &objectID, string &name, fs::path &mesh,
                     vector<int> &calibrated_sensors, map<int, Sensor> &sensors);

    /**
     * Writes lighthouse calibration to file
     * @param filepath
     * @param lighthouse of this lighthouse
     * @param calib those values
     * @return success
     */
    bool writeCalibrationConfig(string filepath, int lighthouse, LighthouseCalibration *calib);

    /**
     * Reads lighthouse calibration from file
     * @param filepath
     * @param lighthouse of this lighthouse
     * @param calib will be filled with the data
     * @return success
     */
    bool readCalibrationConfig(fs::path filepath, int lighthouse, LighthouseCalibration *calib);

    /**
     * Checks if the provided directory exists and is actually a directory
     * @param directory_path
     * @return is directory
     */
    bool directory_exists(string directory_path);
    /**
     * Checks if the provided file exists and is actually a file
     * @param file_path
     * @return is file
     */
    bool file_exists(string file_path);
};