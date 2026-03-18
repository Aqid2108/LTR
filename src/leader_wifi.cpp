#include <iostream>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <sstream>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include "common_servo.hpp"
#include "SCServo.h"
#include "SMS_STS.h"

static bool open_servo_port(SMS_STS& sm_st, const std::string& port, int baudrate) {
    if (sm_st.begin(baudrate, port.c_str())) {
        return true;
    }
    std::cerr << "Failed to open serial port: " << port << "\n";
    return false;
}

static int read_servo_position(SMS_STS& sm_st, int id) {
    int pos = sm_st.ReadPos(id);
    if (pos < 0) {
        std::cerr << "Warning: failed to read servo ID " << id << "\n";
    }
    return pos;
}

static int compute_mapped_target(int leader_raw,
                                 const ServoConfig& leader_servo,
                                 const ServoConfig& follower_servo,
                                 const JointMap& pair) {
    const double leader_up_span =
        std::max(1, std::abs(leader_servo.max_pos - pair.leader_ref));
    const double leader_down_span =
        std::max(1, std::abs(pair.leader_ref - leader_servo.min_pos));

    double delta_norm = 0.0;
    if (leader_raw >= pair.leader_ref) {
        delta_norm = (leader_raw - pair.leader_ref) / leader_up_span;
    } else {
        delta_norm = (leader_raw - pair.leader_ref) / leader_down_span;
    }

    delta_norm = clamp_double(delta_norm, -1.0, 1.0);

    if (pair.invert) {
        delta_norm = -delta_norm;
    }

    const double follower_up_span =
        std::max(1, std::abs(follower_servo.max_pos - pair.follower_ref));
    const double follower_down_span =
        std::max(1, std::abs(pair.follower_ref - follower_servo.min_pos));

    double target = 0.0;
    if (delta_norm >= 0.0) {
        target = pair.follower_ref + delta_norm * follower_up_span;
    } else {
        target = pair.follower_ref + delta_norm * follower_down_span;
    }

    target += pair.trim;

    int out = static_cast<int>(std::round(target));
    int lo = std::min(follower_servo.min_pos, follower_servo.max_pos);
    int hi = std::max(follower_servo.min_pos, follower_servo.max_pos);
    return clamp_int(out, lo, hi);
}

int run_leader_mode(const std::string& leader_yaml,
                    const std::string& follower_yaml,
                    const std::string& mapping_yaml,
                    const std::string& follower_ip,
                    int follower_port) {
    CalibrationData leader_cfg = load_calibration_yaml(leader_yaml);
    CalibrationData follower_cfg = load_calibration_yaml(follower_yaml);
    MappingData map = load_mapping_yaml(mapping_yaml);

    print_calibration("Leader calibration", leader_cfg);
    print_calibration("Follower calibration", follower_cfg);
    print_mapping("Leader-follower mapping", map);

    SMS_STS sm_st;
    if (!open_servo_port(sm_st, leader_cfg.port, leader_cfg.baudrate)) {
        return 1;
    }

    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        std::cerr << "Socket creation failed.\n";
        return 1;
    }

    sockaddr_in server_addr{};
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(follower_port);
    if (inet_pton(AF_INET, follower_ip.c_str(), &server_addr.sin_addr) <= 0) {
        std::cerr << "Invalid follower IP address.\n";
        close(sock);
        return 1;
    }

    std::cout << "Connecting to follower " << follower_ip << ":" << follower_port << " ...\n";
    if (connect(sock, (sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Connection to follower failed.\n";
        close(sock);
        return 1;
    }

    std::cout << "Connected. Streaming mapped control.\n";
    std::cout << "Press Ctrl+C to stop.\n";

    while (true) {
        std::ostringstream msg;
        bool first = true;

        for (const auto& pair : map.pairs) {
            const ServoConfig* leader_servo = find_servo_by_id(leader_cfg, pair.leader_id);
            const ServoConfig* follower_servo = find_servo_by_id(follower_cfg, pair.follower_id);

            if (!leader_servo || !follower_servo) {
                std::cerr << "Mapping references missing servo IDs.\n";
                continue;
            }

            int leader_raw = read_servo_position(sm_st, pair.leader_id);
            if (leader_raw < 0) continue;

            int follower_target = compute_mapped_target(
                leader_raw, *leader_servo, *follower_servo, pair
            );

            if (!first) msg << ",";
            msg << pair.follower_id << ":" << follower_target;
            first = false;
        }

        msg << "\n";
        std::string out = msg.str();

        ssize_t sent = send(sock, out.c_str(), out.size(), 0);
        if (sent <= 0) {
            std::cerr << "Lost connection to follower.\n";
            break;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }

    close(sock);
    return 0;
}

void run_leader() {
    // existing leader code here
}