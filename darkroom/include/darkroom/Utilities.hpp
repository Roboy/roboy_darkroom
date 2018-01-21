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