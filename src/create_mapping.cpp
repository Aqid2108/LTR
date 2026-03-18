#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <algorithm>

#include "common_servo.hpp"
#include "SCServo.h"
#include "SMS_STS.h"

static void flush_line() {
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

static void wait_for_enter(const std::string& msg) {
    std::cout << msg;
    std::cin.get();
}

static bool open_servo_port(SMS_STS& sm_st, const std::string& port, int baudrate) {
    if (sm_st.begin(baudrate, port.c_str())) {
        return true;
    }
    std::cerr << "Failed to open serial port: " << port << "\n";
    return false;
}

static int read_servo_position_retry(SMS_STS& sm_st, int id, int attempts = 8) {
    for (int i = 0; i < attempts; ++i) {
        int pos = sm_st.ReadPos(id);
        if (pos >= 0) return pos;
    }
    return -1;
}

static bool contains_id(const CalibrationData& cfg, int id) {
    return find_servo_by_id(cfg, id) != nullptr;
}

int run_create_mapping_mode() {
    std::string leader_yaml_path, follower_yaml_path, output_mapping_path;

    std::cout << "\n=== Create Leader-Follower Mapping/Alignment ===\n";
    std::cout << "Leader YAML path: ";
    std::cin >> leader_yaml_path;
    std::cout << "Follower YAML path: ";
    std::cin >> follower_yaml_path;
    std::cout << "Output mapping YAML file: ";
    std::cin >> output_mapping_path;
    flush_line();

    CalibrationData leader_cfg = load_calibration_yaml(leader_yaml_path);
    CalibrationData follower_cfg = load_calibration_yaml(follower_yaml_path);

    print_calibration("Leader calibration", leader_cfg);
    print_calibration("Follower calibration", follower_cfg);

    SMS_STS leader_bus;
    SMS_STS follower_bus;

    if (!open_servo_port(leader_bus, leader_cfg.port, leader_cfg.baudrate)) {
        return 1;
    }
    if (!open_servo_port(follower_bus, follower_cfg.port, follower_cfg.baudrate)) {
        return 1;
    }

    const std::vector<std::string> joint_names = {
        "base", "shoulder", "elbow", "wrist_pitch", "wrist_roll", "gripper"
    };

    MappingData map;
    map.leader_yaml = leader_yaml_path;
    map.follower_yaml = follower_yaml_path;
    map.pairs.clear();

    std::cout << "\nCreate one pair per physical joint.\n";
    std::cout << "Use IDs from the YAML files shown above.\n\n";

    for (const auto& joint : joint_names) {
        JointMap jm;
        jm.joint = joint;

        while (true) {
            std::cout << "Joint [" << joint << "] leader ID: ";
            std::cin >> jm.leader_id;
            if (!contains_id(leader_cfg, jm.leader_id)) {
                std::cout << "Leader ID not found in leader.yaml. Try again.\n";
                continue;
            }
            break;
        }

        while (true) {
            std::cout << "Joint [" << joint << "] follower ID: ";
            std::cin >> jm.follower_id;
            if (!contains_id(follower_cfg, jm.follower_id)) {
                std::cout << "Follower ID not found in follower.yaml. Try again.\n";
                continue;
            }
            break;
        }

        int invert_input = 0;
        std::cout << "Invert direction for joint [" << joint << "]? (0 = no, 1 = yes): ";
        std::cin >> invert_input;
        jm.invert = (invert_input != 0);

        std::cout << "Trim for joint [" << joint << "] (raw counts, default 0): ";
        std::cin >> jm.trim;

        map.pairs.push_back(jm);
        flush_line();
        std::cout << "\n";
    }

    std::cout << "Now place BOTH arms in the SAME physical reference pose.\n";
    std::cout << "Example:\n";
    std::cout << "  - base facing forward\n";
    std::cout << "  - shoulder level\n";
    std::cout << "  - elbow bent same amount\n";
    std::cout << "  - wrist straight\n";
    std::cout << "  - gripper neutral\n\n";

    wait_for_enter("Press Enter to capture reference raw values for all mapped joints...");

    for (auto& jm : map.pairs) {
        jm.leader_ref = read_servo_position_retry(leader_bus, jm.leader_id);
        jm.follower_ref = read_servo_position_retry(follower_bus, jm.follower_id);

        if (jm.leader_ref < 0) {
            std::cerr << "Failed reading leader reference for ID " << jm.leader_id << "\n";
            return 1;
        }
        if (jm.follower_ref < 0) {
            std::cerr << "Failed reading follower reference for ID " << jm.follower_id << "\n";
            return 1;
        }
    }

    std::cout << "\nCaptured reference values:\n";
    for (const auto& jm : map.pairs) {
        std::cout
            << "  " << jm.joint
            << " | leader_id=" << jm.leader_id
            << " leader_ref=" << jm.leader_ref
            << " | follower_id=" << jm.follower_id
            << " follower_ref=" << jm.follower_ref
            << "\n";
    }

    save_mapping_yaml(output_mapping_path, map);

    std::cout << "\nMapping saved to: " << output_mapping_path << "\n";
    print_mapping("Saved mapping", map);
    return 0;
}


void run_create_mapping() {
    // existing mapping code here
}