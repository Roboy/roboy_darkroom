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
        ROS_ERROR("mesh directory %s does not exist", info.meshDirectory);
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
        ROS_ERROR("lighthouse directory %s does not exist", info.lighthouseDirectory);
        return false;
    }
}

bool Utilities::readConfig(fs::path filepath, int &objectID, string &name, fs::path &mesh,
                           vector<int> &calibrated_sensors, map<int, Sensor> &sensors) {
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
            ROS_ERROR("mesh %s does not exist", mesh.c_str());
            return false;
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