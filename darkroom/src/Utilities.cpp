#include "darkroom/Utilities.hpp"

bool Utilities::readConfig(string filepath, int &objectID, string &name, string &mesh,
                           vector<int> &calibrated_sensors, map<int, Sensor> &sensors) {
    try{
        YAML::Node config = YAML::LoadFile(filepath);
        objectID = config["ObjectID"].as<int>();
        name = config["name"].as<string>();
        mesh = config["mesh"].as<string>();
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
    }catch(YAML::Exception& e) {
        ROS_ERROR_STREAM(e.what());
        return false;
    }
    return true;
}

bool Utilities::writeConfig(string filepath, int &objectID, string &name, string &mesh,
                            vector<int> &calibrated_sensors, map<int, Sensor> &sensors) {
    std::ofstream fout(filepath);
    if (!fout.is_open()) {
        ROS_WARN_STREAM("Could not write config " << filepath);
        return false;
    }

    YAML::Node config;
    config["ObjectID"] = objectID;
    config["name"] = name;
    config["mesh"] = mesh;
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