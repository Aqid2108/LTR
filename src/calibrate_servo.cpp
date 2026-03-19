#include <iostream>
#include <string>
#include <vector>
#include <limits>
#include <algorithm>
#include <filesystem>

#include "common_servo.hpp"
#include "SCServo.h"
#include "SMS_STS.h"

namespace fs = std::filesystem;

static void flush_line() {
    std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
}

static void wait_for_enter(const std::string& msg) {
    std::cout << msg;
    std::cin.get();
}

static std::vector<std::string> detect_candidate_ports() {
    std::vector<std::string> ports;

    const std::vector<std::string> prefixes = {
        "/dev/ttyACM",
        "/dev/ttyUSB",
        "/dev/ttyS"
    };

    for (const auto& entry : fs::directory_iterator("/dev")) {
        std::string path = entry.path().string();
        for (const auto& p : prefixes) {
            if (path.rfind(p, 0) == 0) {
                ports.push_back(path);
                break;
            }
        }
    }

    std::sort(ports.begin(), ports.end());
    return ports;
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

static std::vector<int> detect_servo_ids(SMS_STS& sm_st, int id_min = 1, int id_max = 20) {
    std::vector<int> ids;
    for (int id = id_min; id <= id_max; ++id) {
        int pos = read_servo_position_retry(sm_st, id, 3);
        if (pos >= 0) {
            ids.push_back(id);
            std::cout << "Detected servo ID " << id << " at raw position " << pos << "\n";
        }
    }
    return ids;
}

static std::vector<int> capture_positions(SMS_STS& sm_st, const std::vector<int>& ids) {
    std::vector<int> positions;
    positions.reserve(ids.size());

    for (int id : ids) {
        int pos = read_servo_position_retry(sm_st, id, 8);
        positions.push_back(pos);
    }

    return positions;
}

static bool has_duplicate_ids(const std::vector<int>& ids) {
    std::vector<int> sorted = ids;
    std::sort(sorted.begin(), sorted.end());
    return std::adjacent_find(sorted.begin(), sorted.end()) != sorted.end();
}

int run_calibrate_all_mode() {
    CalibrationData cfg;
    cfg.baudrate = 1000000;

    std::string output_yaml;
    std::cout << "\n=== Auto Calibration: All 6 Servos ===\n";
    std::cout << "This mode expects one full arm with servo IDs ["
              << expected_arm_servo_ids_text() << "].\n";
    std::cout << "Output YAML file: ";
    std::cin >> output_yaml;
    flush_line();

    auto ports = detect_candidate_ports();
    if (ports.empty()) {
        std::cerr << "No candidate serial ports found under /dev.\n";
        return 1;
    }

    std::cout << "\nDetected serial ports:\n";
    for (size_t i = 0; i < ports.size(); ++i) {
        std::cout << "  [" << i << "] " << ports[i] << "\n";
    }

    cfg.port = ports.front();
    std::cout << "\nSelected port: " << cfg.port << "\n";
    wait_for_enter("Press Enter to confirm and continue...");
    std::cout << "Using baudrate: " << cfg.baudrate << "\n";

    SMS_STS sm_st;
    if (!open_servo_port(sm_st, cfg.port, cfg.baudrate)) {
        return 1;
    }

    std::cout << "\nScanning servo IDs...\n";
    std::vector<int> ids = detect_servo_ids(sm_st, 1, 20);

    if (has_duplicate_ids(ids)) {
        std::cerr << "\nDetected duplicate servo IDs during scan: ["
                  << join_ints(ids) << "]\n";
        return 1;
    }

    std::sort(ids.begin(), ids.end());

    const std::vector<int> expected_ids = expected_arm_servo_ids();
    if (ids.size() != expected_ids.size() || ids != expected_ids) {
        std::cerr << "\nCalibration requires exactly 6 reachable servos with IDs ["
                  << expected_arm_servo_ids_text() << "].\n";
        std::cerr << "Detected IDs: [" << join_ints(ids) << "]\n";
        return 1;
    }

    std::cout << "\nDetected all 6 servo IDs:\n";
    for (int id : ids) {
        std::cout << "  ID " << id << " (" << joint_name_for_servo_id(id) << ")\n";
    }

    cfg.servos.clear();

    for (size_t i = 0; i < ids.size(); ++i) {
        int id = ids[i];

        std::cout << "\n====================================\n";
        std::cout << "Calibrating servo " << (i + 1) << " of " << ids.size() << "\n";
        std::cout << "Servo ID: " << id
                  << " (" << joint_name_for_servo_id(id) << ")\n";
        std::cout << "Only move this one joint while capturing min/max.\n";

        int current_pos = read_servo_position_retry(sm_st, id, 8);
        if (current_pos < 0) {
            std::cerr << "Failed to read current position for servo ID " << id << "\n";
            return 1;
        }

        std::cout << "Current raw position: " << current_pos << "\n";

        wait_for_enter(
            "Move ONLY this servo to its MIN physical limit, then press Enter to capture...");
        int min_pos = read_servo_position_retry(sm_st, id, 8);
        if (min_pos < 0) {
            std::cerr << "Failed reading MIN for servo ID " << id << "\n";
            return 1;
        }

        std::cout << "Captured MIN for ID " << id << ": " << min_pos << "\n";

        wait_for_enter(
            "Move ONLY this servo to its MAX physical limit, then press Enter to capture...");
        int max_pos = read_servo_position_retry(sm_st, id, 8);
        if (max_pos < 0) {
            std::cerr << "Failed reading MAX for servo ID " << id << "\n";
            return 1;
        }

        std::cout << "Captured MAX for ID " << id << ": " << max_pos << "\n";

        if (min_pos == max_pos) {
            std::cerr << "MIN and MAX are identical for servo ID " << id
                      << ". Calibration invalid.\n";
            return 1;
        }

        if (std::abs(max_pos - min_pos) < 32) {
            std::cerr << "MIN/MAX span too small for servo ID " << id
                      << " (" << min_pos << " to " << max_pos << ").\n";
            std::cerr << "Move the joint through a real range and calibrate again.\n";
            return 1;
        }

        ServoConfig s;
        s.name = joint_name_for_servo_id(id);
        s.id = id;
        s.min_pos = min_pos;
        s.max_pos = max_pos;
        s.invert = false;

        cfg.servos.push_back(s);

        std::cout << "Saved servo ID " << id
                  << " | min=" << min_pos
                  << " | max=" << max_pos << "\n";
        std::cout << "====================================\n";
    }

    save_calibration_yaml(output_yaml, cfg);

    std::cout << "\nCalibration saved to: " << output_yaml << "\n";
    print_calibration("Saved calibration", cfg);

    return 0;
}
void run_calibration();
