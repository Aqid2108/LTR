#pragma once

#include <string>
#include <vector>

struct ServoConfig {
    std::string name;
    int id = 0;
    int min = 0;
    int max = 0;
    int rest = 0;
    bool invert = false;
};

struct CalibrationData {
    std::string port;
    int baudrate = 1000000;
    std::vector<ServoConfig> servos;
};

void run_calibration_menu();
void run_leader_mode();
void run_follower_mode();

bool load_calibration_yaml(const std::string& yaml_path, CalibrationData& data);
void print_calibration(const CalibrationData& data);