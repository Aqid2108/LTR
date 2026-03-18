#pragma once

#include <string>

struct JointMapEntry {
    std::string name;
    int id;
};

struct JointCalibEntry {
    std::string name;
    int id;
    int min_pos;
    int max_pos;
    int rest_pos;
    bool invert;
};

bool calibrate_all_6_servos(const std::string& port, int baudrate,
                            const std::string& calib_yaml = "calibration.yaml");