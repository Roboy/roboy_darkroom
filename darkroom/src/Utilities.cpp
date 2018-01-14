#include "darkroom/Utilities.hpp"

bool Utilities::readModelDirectory(string modelDirectory, ModelInformation &info) {
    if(!directory_exists(modelDirectory))
        return false;

    info.modelSdf = fs::path(modelDirectory+"/model.sdf");

    info.rootDirectory = fs::path(modelDirectory);
    info.meshDirectory = fs::path(modelDirectory+"/meshes/CAD");
    if(directory_exists(info.meshDirectory.root_path().c_str())){
        fs::directory_iterator end_iter;
        for (fs::directory_iterator dir_itr(info.meshDirectory); dir_itr != end_iter; ++dir_itr) {
            try {
                if (fs::is_regular_file(dir_itr->status())) {
                    info.meshes.push_back(dir_itr->path());
                    ROS_DEBUG_STREAM(dir_itr->path());
                }
            }
            catch (const std::exception &ex) {
                ROS_ERROR_STREAM(dir_itr->path().filename() << " " << ex.what());
            }
        }
    }else{
        ROS_ERROR("mesh directory %s does not exist", info.meshDirectory.c_str());
        return false;
    }
    info.lighthouseDirectory = fs::path(modelDirectory+"/lighthouseSensors");
    if(directory_exists(info.lighthouseDirectory.root_path().c_str())){
        fs::directory_iterator end_iter;
        for (fs::directory_iterator dir_itr(info.lighthouseDirectory); dir_itr != end_iter; ++dir_itr) {
            try {
                if (fs::is_regular_file(dir_itr->status())) {
                    info.lighthouseConfigFiles.push_back(dir_itr->path());
                    ROS_DEBUG_STREAM(dir_itr->path());
                }
            }
            catch (const std::exception &ex) {
                ROS_ERROR_STREAM(dir_itr->path().filename() << " " << ex.what());
            }
        }
    }else{
        ROS_ERROR("lighthouse directory %s does not exist", info.lighthouseDirectory.c_str());
        return false;
    }
}

bool Utilities::readConfig(fs::path filepath, int &objectID, string &name, fs::path &mesh,
                           vector<int> &calibrated_sensors, map<int, Sensor> &sensors, map<int,vector<double>> &calibrationAngles) {
    if(!file_exists(filepath.c_str()))
        return false;
    try {
        ROS_INFO_STREAM("reading config " << filepath.filename());
        YAML::Node config = YAML::LoadFile(filepath.c_str());
        objectID = config["ObjectID"].as<int>();
        name = config["name"].as<string>();
        mesh = filepath.remove_filename().c_str();
        mesh += "/" + config["mesh"].as<string>();
        if(!file_exists(mesh.c_str())){
            ROS_WARN("mesh %s does not exist", mesh.c_str());
            mesh.clear();
        }
        vector<vector<float>> relative_locations =
                config["sensor_relative_locations"].as<vector<vector<float >>>();
        sensors.clear();
        calibrated_sensors.clear();
        cout << "using calibrated sensors: ";
        for (int i = 0; i < relative_locations.size(); i++) {
            Vector3d relLocation(relative_locations[i][1], relative_locations[i][2], relative_locations[i][3]);
            sensors[relative_locations[i][0]].setRelativeLocation(relLocation);
            calibrated_sensors.push_back((int) relative_locations[i][0]);
            cout << "\t" << calibrated_sensors.back();
        }
        cout << endl;
        if(config["calibration_angles"]){
            ROS_INFO("found calibration angles");
            vector<vector<float>> calAngles = config["calibration_angles"].as<vector<vector<float >>>();
            for(int i=0;i<calAngles.size();i++){
                calibrationAngles[calAngles[i][0]].push_back(calAngles[i][2]);
                calibrationAngles[calAngles[i][0]].push_back(calAngles[i][1]);
                ROS_INFO_STREAM(calibrationAngles[calAngles[i][0]][VERTICAL]<<" "<<calibrationAngles[calAngles[i][0]][HORIZONTAL]);
            }
        }
    } catch (YAML::Exception &e) {
        ROS_ERROR_STREAM(e.what());
        return false;
    }
    return true;
}

bool Utilities::writeConfig(fs::path filepath, int &objectID, string &name, fs::path &mesh,
                            vector<int> &calibrated_sensors, map<int, Sensor> &sensors) {
    std::ofstream fout(filepath.root_path().c_str());
    if (!fout.is_open()) {
        ROS_WARN_STREAM("Could not write config " << filepath);
        return false;
    }

    YAML::Node config;
    config["ObjectID"] = objectID;
    config["name"] = name;
    config["mesh"] = mesh.c_str();
    for (auto &sensor : sensors) {
        if (!sensor.second.isCalibrated())
            continue;
        YAML::Node node = YAML::Load("[0, 0, 0, 0]");
        Vector3d relative_location;
        sensor.second.getRelativeLocation(relative_location);
        // first number is the sensor id
        node[0] = sensor.first;
        for (int i = 1; i <= 3; i++) {
            node[i] = relative_location(i - 1);
        }
        config["sensor_relative_locations"].push_back(node);
    }

    fout << config;
    fout.close();
    return true;
}

bool Utilities::writeCalibrationConfig(string filepath, int lighthouse, LighthouseCalibration *calib){
    fstream fout;
    fout.open(filepath);
    if (!fout.is_open()) {
        ROS_WARN_STREAM("Could not write config " << filepath);
        return false;
    }

    YAML::Node config;

    YAML::Node node = YAML::Load("[0, 0, 0, 0, 0, 0, 0, 0, 0, 0]");
    node[0] = calib[VERTICAL].phase;
    node[1] = calib[HORIZONTAL].phase;
    node[2] = calib[VERTICAL].tilt;
    node[3] = calib[HORIZONTAL].tilt;
    node[4] = calib[VERTICAL].curve;
    node[5] = calib[HORIZONTAL].curve;
    node[6] = calib[VERTICAL].gibphase;
    node[7] = calib[HORIZONTAL].gibphase;
    node[8] = calib[VERTICAL].gibmag;
    node[9] = calib[HORIZONTAL].gibmag;
    char str[100];
    sprintf(str,"ligthhouse%d", lighthouse);
    config[str].push_back(node);

    fout << config;
    fout.close();
    return true;
}

bool Utilities::readCalibrationConfig(fs::path filepath, int lighthouse, LighthouseCalibration *calib){
    if(!file_exists(filepath.c_str()))
        return false;
    try {
        ROS_INFO_STREAM("reading calibration config " << filepath.filename() << " for lighthouse " << lighthouse);

        YAML::Node config = YAML::LoadFile(filepath.c_str());
        char str[100];
        sprintf(str,"ligthhouse%d", lighthouse);
        vector<float> calib_data = config[str].as<vector<float>>();
        calib[VERTICAL].phase = calib_data[0];
        calib[HORIZONTAL].phase = calib_data[1];
        calib[VERTICAL].tilt = calib_data[2];
        calib[HORIZONTAL].tilt = calib_data[3];
        calib[VERTICAL].curve = calib_data[4];
        calib[HORIZONTAL].curve = calib_data[5];
        calib[VERTICAL].gibphase = calib_data[6];
        calib[HORIZONTAL].gibphase = calib_data[7];
        calib[VERTICAL].gibmag = calib_data[8];
        calib[HORIZONTAL].gibmag = calib_data[9];

    } catch (YAML::Exception &e) {
        ROS_ERROR_STREAM(e.what());
        return false;
    }
    return true;
}

bool Utilities::directory_exists(string directory_path){
    if (!fs::exists(directory_path)) {
        ROS_ERROR("%s does not exist", directory_path.c_str());
        return false;
    }
    if (!fs::is_directory(directory_path)) {
        ROS_ERROR("%s id not a directory", directory_path.c_str());
        return false;
    }
    return true;
}

bool Utilities::file_exists(string file_path){
    if (!fs::exists(file_path)) {
        ROS_ERROR("%s does not exist", file_path.c_str());
        return false;
    }
    if (!fs::is_regular_file(file_path)) {
        ROS_ERROR("%s id not a file", file_path.c_str());
        return false;
    }
    return true;
}