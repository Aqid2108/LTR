#include "lerobot_hybrid.hpp"
#include "common_servo.hpp"
#include "SCServo.h"
#include "SMS_STS.h"

#include <yaml-cpp/yaml.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <string>
#include <thread>
#include <vector>

namespace {

struct JointRef {
    int leader_start = 0;
    int follower_start = 0;
    int last_sent = std::numeric_limits<int>::min();
};

struct FollowPacket {
    int count = 0;
    int ids[16]{};
    int positions[16]{};
};

int clamp_int(int v, int lo, int hi) {
    return std::max(lo, std::min(hi, v));
}

double safe_range(int a, int b) {
    double r = std::abs(static_cast<double>(b - a));
    return (r < 1.0) ? 1.0 : r;
}

bool load_servo_list(const std::string& path,
                     std::string& port,
                     int& baudrate,
                     std::vector<ServoConfig>& servos) {
    try {
        YAML::Node root = YAML::LoadFile(path);

        port = root["port"].as<std::string>();
        baudrate = root["baudrate"].as<int>();

        YAML::Node arr = root["servos"];
        if (!arr || !arr.IsSequence()) {
            std::cerr << "Invalid servo YAML: missing servos in " << path << "\n";
            return false;
        }

        servos.clear();
        for (const auto& s : arr) {
            ServoConfig sc{};
            sc.name = s["name"].as<std::string>();
            sc.id = s["id"].as<int>();
            sc.min_pos = s["min"].as<int>();
            sc.max_pos = s["max"].as<int>();
            sc.invert = s["invert"] ? s["invert"].as<bool>() : false;
            servos.push_back(sc);
        }

        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to load servo YAML: " << path << "\n";
        std::cerr << e.what() << "\n";
        return false;
    }
}

bool open_servo_port(SMS_STS& sm_st, const std::string& port, int baudrate) {
    if (sm_st.begin(baudrate, port.c_str())) {
        return true;
    }
    std::cerr << "Failed to open serial port: " << port << "\n";
    return false;
}

int read_servo_position_retry(SMS_STS& sm_st, int id, int attempts = 8) {
    for (int i = 0; i < attempts; ++i) {
        int pos = sm_st.ReadPos(id);
        if (pos >= 0) return pos;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return -1;
}

int map_absolute_leader_to_follower(const ServoConfig& leader_servo,
                                    const ServoConfig& follower_servo,
                                    int leader_now) {
    double leader_span = safe_range(leader_servo.min_pos, leader_servo.max_pos);
    double norm =
        (static_cast<double>(leader_now) - static_cast<double>(leader_servo.min_pos)) / leader_span;

    norm = std::clamp(norm, 0.0, 1.0);

    bool effective_invert = leader_servo.invert ^ follower_servo.invert;
    if (effective_invert) {
        norm = 1.0 - norm;
    }

    double follower_span =
        static_cast<double>(follower_servo.max_pos - follower_servo.min_pos);

    int target = static_cast<int>(std::lround(
        static_cast<double>(follower_servo.min_pos) + norm * follower_span
    ));

    return clamp_int(target, follower_servo.min_pos, follower_servo.max_pos);
}

int compute_relative_target(const ServoConfig& leader_servo,
                            const ServoConfig& follower_servo,
                            int leader_now,
                            const JointRef& ref) {
    double leader_span = safe_range(leader_servo.min_pos, leader_servo.max_pos);
    double delta_norm =
        (static_cast<double>(leader_now) - static_cast<double>(ref.leader_start)) / leader_span;

    bool effective_invert = leader_servo.invert ^ follower_servo.invert;
    if (effective_invert) {
        delta_norm = -delta_norm;
    }

    double follower_span =
        static_cast<double>(follower_servo.max_pos - follower_servo.min_pos);

    int target = static_cast<int>(std::lround(
        static_cast<double>(ref.follower_start) + delta_norm * follower_span
    ));

    return clamp_int(target, follower_servo.min_pos, follower_servo.max_pos);
}

bool send_follow_packet(int sock,
                        const sockaddr_in& follower_addr,
                        const FollowPacket& pkt) {
    ssize_t sent = sendto(sock,
                          &pkt,
                          sizeof(pkt),
                          0,
                          reinterpret_cast<const sockaddr*>(&follower_addr),
                          sizeof(follower_addr));
    return sent >= 0;
}

} // namespace

int run_lerobot_hybrid_mode(const std::string& leader_yaml,
                            const std::string& follower_yaml,
                            const std::string& follower_ip,
                            int follower_port) {
    std::cout << "\n=== LeRobot Hybrid Follow Mode ===\n";

    std::string leader_port, follower_port_name;
    int leader_baudrate = 0;
    int follower_baudrate = 0;

    std::vector<ServoConfig> leader_servos;
    std::vector<ServoConfig> follower_servos;

    if (!load_servo_list(leader_yaml, leader_port, leader_baudrate, leader_servos)) {
        return 1;
    }

    if (!load_servo_list(follower_yaml, follower_port_name, follower_baudrate, follower_servos)) {
        return 1;
    }

    if (leader_servos.size() != follower_servos.size()) {
        std::cerr << "Leader/follower servo count mismatch.\n";
        std::cerr << "Leader servos: " << leader_servos.size() << "\n";
        std::cerr << "Follower servos: " << follower_servos.size() << "\n";
        return 1;
    }

    if (leader_servos.empty()) {
        std::cerr << "No servos found in YAML files.\n";
        return 1;
    }

    if (leader_servos.size() > 16) {
        std::cerr << "Too many servos. Max supported is 16.\n";
        return 1;
    }

    std::cout << "\nLeader YAML: " << leader_yaml << "\n";
    std::cout << "Follower YAML: " << follower_yaml << "\n";
    std::cout << "Leader port: " << leader_port << "\n";
    std::cout << "Leader baudrate: " << leader_baudrate << "\n";
    std::cout << "Follower IP: " << follower_ip << "\n";
    std::cout << "Follower port: " << follower_port << "\n";

    std::cout << "\nIMPORTANT:\n";
    std::cout << "This mode does NOT use a mapping YAML.\n";
    std::cout << "Leader and follower are matched by servo order in the YAML files.\n";
    std::cout << "Place both arms in IDENTICAL physical positions before continuing.\n";
    std::cout << "Ensure base, shoulder, elbow, wrist, and gripper all match.\n";
    std::cout << "Make sure the follower arm has enough room to move.\n";
    std::cout << "Press ENTER only when both arms are in the same pose...";
    std::cin.get();

    SMS_STS leader_io;
    if (!open_servo_port(leader_io, leader_port, leader_baudrate)) {
        return 1;
    }

    for (size_t i = 0; i < leader_servos.size(); ++i) {
        int pos = read_servo_position_retry(leader_io, leader_servos[i].id, 8);
        if (pos < 0) {
            std::cerr << "Failed to read leader servo ID " << leader_servos[i].id << "\n";
            leader_io.end();
            return 1;
        }
    }

    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) {
        std::cerr << "Failed to create UDP socket.\n";
        leader_io.end();
        return 1;
    }

    sockaddr_in follower_addr{};
    follower_addr.sin_family = AF_INET;
    follower_addr.sin_port = htons(follower_port);

    if (inet_pton(AF_INET, follower_ip.c_str(), &follower_addr.sin_addr) <= 0) {
        std::cerr << "Invalid follower IP: " << follower_ip << "\n";
        close(sock);
        leader_io.end();
        return 1;
    }

    std::vector<JointRef> refs(leader_servos.size());

    // Phase 1: absolute align
    std::cout << "\nAligning follower to leader...\n";
    std::cout << "Do NOT touch the arms.\n";

    FollowPacket align_pkt{};
    align_pkt.count = static_cast<int>(leader_servos.size());

    for (size_t i = 0; i < leader_servos.size(); ++i) {
        const ServoConfig& ls = leader_servos[i];
        const ServoConfig& fs = follower_servos[i];

        int leader_now = read_servo_position_retry(leader_io, ls.id, 8);
        if (leader_now < 0) {
            std::cerr << "Failed to read leader servo ID " << ls.id << " during alignment.\n";
            close(sock);
            leader_io.end();
            return 1;
        }

        int follower_target = map_absolute_leader_to_follower(ls, fs, leader_now);

        align_pkt.ids[i] = fs.id;
        align_pkt.positions[i] = follower_target;

        refs[i].follower_start = follower_target;
        refs[i].last_sent = follower_target;

        std::cout << "Joint " << (i + 1)
                  << " | leader_id=" << ls.id
                  << " raw=" << leader_now
                  << " -> follower_id=" << fs.id
                  << " target=" << follower_target
                  << "\n";
    }

    if (!send_follow_packet(sock, follower_addr, align_pkt)) {
        std::cerr << "Failed to send alignment packet.\n";
        close(sock);
        leader_io.end();
        return 1;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1200));

    // Phase 2: capture reference
    std::cout << "\nAlignment complete.\n";
    std::cout << "Capturing reference pose...\n";

    for (size_t i = 0; i < leader_servos.size(); ++i) {
        const ServoConfig& ls = leader_servos[i];
        int leader_now = read_servo_position_retry(leader_io, ls.id, 8);
        if (leader_now < 0) {
            std::cerr << "Failed to read leader servo ID " << ls.id << " during reference capture.\n";
            close(sock);
            leader_io.end();
            return 1;
        }

        refs[i].leader_start = leader_now;

        std::cout << "Joint " << (i + 1)
                  << " | leader_start=" << refs[i].leader_start
                  << " | follower_start=" << refs[i].follower_start
                  << "\n";
    }

    std::cout << "\nStarting smooth relative follow...\n";
    std::cout << "Move the leader arm by hand.\n";
    std::cout << "Press Ctrl+C to stop.\n";

    // Phase 3: relative follow
    while (true) {
        FollowPacket pkt{};
        pkt.count = static_cast<int>(leader_servos.size());

        for (size_t i = 0; i < leader_servos.size(); ++i) {
            const ServoConfig& ls = leader_servos[i];
            const ServoConfig& fs = follower_servos[i];

            int leader_now = read_servo_position_retry(leader_io, ls.id, 8);
            if (leader_now < 0) {
                std::cerr << "Failed to read leader servo ID " << ls.id << " during follow.\n";
                close(sock);
                leader_io.end();
                return 1;
            }

            int target = compute_relative_target(ls, fs, leader_now, refs[i]);

            if (refs[i].last_sent != std::numeric_limits<int>::min() &&
                std::abs(target - refs[i].last_sent) < 4) {
                target = refs[i].last_sent;
            } else {
                refs[i].last_sent = target;
            }

            pkt.ids[i] = fs.id;
            pkt.positions[i] = target;
        }

        if (!send_follow_packet(sock, follower_addr, pkt)) {
            std::cerr << "sendto failed while sending follow packet.\n";
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    close(sock);
    leader_io.end();
    return 0;
}