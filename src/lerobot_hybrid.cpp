#include "lerobot_hybrid.hpp"
#include "common_servo.hpp"

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
#include "SMS_STS.h"
namespace {

struct MappingEntry {
    int leader_id = 0;
    int follower_id = 0;
    bool invert = false;
    int trim = 0;
};

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
    double r = std::abs(double(b - a));
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

        servos.clear();
        for (const auto& s : root["servos"]) {
            ServoConfig sc{};
            sc.name = s["name"].as<std::string>();
            sc.id = s["id"].as<int>();
            sc.min_pos = s["min"].as<int>();
            sc.max_pos = s["max"].as<int>();

            // only set fields that definitely exist in your ServoConfig
            sc.invert = s["invert"].as<bool>();

            servos.push_back(sc);
        }
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to load servo YAML: " << path << "\n";
        std::cerr << e.what() << "\n";
        return false;
    }
}

bool load_mapping_config(const std::string& path,
                         std::vector<MappingEntry>& mappings) {
    try {
        YAML::Node root = YAML::LoadFile(path);
        YAML::Node maps = root["mappings"];
        if (!maps || !maps.IsSequence()) {
            std::cerr << "Invalid mapping YAML: missing mappings\n";
            return false;
        }

        mappings.clear();
        for (const auto& m : maps) {
            MappingEntry e{};
            e.leader_id = m["leader_id"].as<int>();
            e.follower_id = m["follower_id"].as<int>();
            e.invert = m["invert"] ? m["invert"].as<bool>() : false;
            e.trim = m["trim"] ? m["trim"].as<int>() : 0;
            mappings.push_back(e);
        }
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Failed to load mapping YAML: " << path << "\n";
        std::cerr << e.what() << "\n";
        return false;
    }
}

const ServoConfig* find_servo_by_id(const std::vector<ServoConfig>& servos, int id) {
    for (const auto& s : servos) {
        if (s.id == id) return &s;
    }
    return nullptr;
}

int map_absolute_leader_to_follower(const ServoConfig& leader_servo,
                                    const ServoConfig& follower_servo,
                                    const MappingEntry& mapping,
                                    int leader_now) {
    double leader_span = safe_range(leader_servo.min_pos, leader_servo.max_pos);
    double norm = (leader_now - leader_servo.min_pos) / leader_span;
    norm = std::clamp(norm, 0.0, 1.0);

    bool effective_invert = mapping.invert ^ leader_servo.invert ^ follower_servo.invert;
    if (effective_invert) {
        norm = 1.0 - norm;
    }

    double follower_span = double(follower_servo.max_pos - follower_servo.min_pos);
    int target = int(std::lround(follower_servo.min_pos + norm * follower_span));
    target += mapping.trim;

    return clamp_int(target, follower_servo.min_pos, follower_servo.max_pos);
}

int compute_relative_target(const ServoConfig& leader_servo,
                            const ServoConfig& follower_servo,
                            const MappingEntry& mapping,
                            int leader_now,
                            const JointRef& ref) {
    double leader_span = safe_range(leader_servo.min_pos, leader_servo.max_pos);
    double delta_norm = (leader_now - ref.leader_start) / leader_span;

    bool effective_invert = mapping.invert ^ leader_servo.invert ^ follower_servo.invert;
    if (effective_invert) {
        delta_norm = -delta_norm;
    }

    double follower_span = double(follower_servo.max_pos - follower_servo.min_pos);
    int target = int(std::lround(ref.follower_start + delta_norm * follower_span));
    target += mapping.trim;

    return clamp_int(target, follower_servo.min_pos, follower_servo.max_pos);
}

} // namespace

int run_lerobot_hybrid_mode(const std::string& leader_yaml,
                            const std::string& follower_yaml,
                            const std::string& mapping_yaml,
                            const std::string& follower_ip,
                            int follower_port) {
    std::cout << "\n=== LeRobot Hybrid Follow Mode ===\n";

    std::string leader_port, follower_port_name;
    int leader_baudrate = 0;
    int follower_baudrate = 0;

    std::vector<ServoConfig> leader_servos;
    std::vector<ServoConfig> follower_servos;
    std::vector<MappingEntry> mappings;

    if (!load_servo_list(leader_yaml, leader_port, leader_baudrate, leader_servos)) {
        return 1;
    }
    if (!load_servo_list(follower_yaml, follower_port_name, follower_baudrate, follower_servos)) {
        return 1;
    }
    if (!load_mapping_config(mapping_yaml, mappings)) {
        return 1;
    }

    if (mappings.empty()) {
        std::cerr << "No mappings found.\n";
        return 1;
    }

    // IMPORTANT:
    // Replace SMS_STS with whatever class your working files already use.
    SMS_STS leader_io;
    if (leader_io.begin(leader_baudrate, leader_port.c_str()) != 1) {
        std::cerr << "Failed to open leader port: " << leader_port << "\n";
        return 1;
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

    std::cout << "Leader port: " << leader_port << "\n";
    std::cout << "Leader baudrate: " << leader_baudrate << "\n";
    std::cout << "Follower target: " << follower_ip << ":" << follower_port << "\n";
    std::cout << "\nPlace both arms in a safe, roughly matching pose.\n";
    std::cout << "Press ENTER to align follower to leader...";
    std::cin.get();

    std::vector<JointRef> refs(mappings.size());

    // Phase 1: absolute align
    {
        FollowPacket pkt{};
        pkt.count = static_cast<int>(mappings.size());

        std::cout << "\nAligning follower to leader...\n";

        for (size_t i = 0; i < mappings.size(); ++i) {
            const auto& map = mappings[i];

            const ServoConfig* ls = find_servo_by_id(leader_servos, map.leader_id);
            const ServoConfig* fs = find_servo_by_id(follower_servos, map.follower_id);

            if (!ls || !fs) {
                std::cerr << "Mapping error: missing servo id.\n";
                close(sock);
                leader_io.end();
                return 1;
            }

            int leader_now = leader_io.ReadPos(ls->id);
            int follower_target_pos = map_absolute_leader_to_follower(*ls, *fs, map, leader_now);

            pkt.ids[i] = fs->id;
            pkt.positions[i] = follower_target_pos;

            refs[i].follower_start = follower_target_pos;
            refs[i].last_sent = follower_target_pos;

            std::cout << "Joint " << (i + 1)
                      << ": leader_id=" << ls->id
                      << " raw=" << leader_now
                      << " -> follower_id=" << fs->id
                      << " target=" << follower_target_pos
                      << "\n";
        }

        ssize_t sent = sendto(sock,
                              &pkt,
                              sizeof(pkt),
                              0,
                              reinterpret_cast<sockaddr*>(&follower_addr),
                              sizeof(follower_addr));

        if (sent < 0) {
            std::cerr << "Failed to send alignment packet.\n";
            close(sock);
            leader_io.end();
            return 1;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1200));
    }

    // Phase 2: capture reference
    std::cout << "Alignment complete.\n";
    std::cout << "Capturing reference pose...\n";

    for (size_t i = 0; i < mappings.size(); ++i) {
        const auto& map = mappings[i];
        const ServoConfig* ls = find_servo_by_id(leader_servos, map.leader_id);

        if (!ls) {
            std::cerr << "Reference capture failed: missing leader servo.\n";
            close(sock);
            leader_io.end();
            return 1;
        }

        refs[i].leader_start = leader_io.ReadPos(ls->id);

        std::cout << "Joint " << (i + 1)
                  << " leader_start=" << refs[i].leader_start
                  << " follower_start=" << refs[i].follower_start
                  << "\n";
    }

    std::cout << "\nStarting smooth relative follow...\n";
    std::cout << "Move the leader arm by hand.\n";
    std::cout << "Press Ctrl+C to stop.\n";

    while (true) {
        FollowPacket pkt{};
        pkt.count = static_cast<int>(mappings.size());

        for (size_t i = 0; i < mappings.size(); ++i) {
            const auto& map = mappings[i];

            const ServoConfig* ls = find_servo_by_id(leader_servos, map.leader_id);
            const ServoConfig* fs = find_servo_by_id(follower_servos, map.follower_id);

            if (!ls || !fs) {
                std::cerr << "Runtime mapping error.\n";
                close(sock);
                leader_io.end();
                return 1;
            }

            int leader_now = leader_io.ReadPos(ls->id);
            int target = compute_relative_target(*ls, *fs, map, leader_now, refs[i]);

            if (refs[i].last_sent != std::numeric_limits<int>::min() &&
                std::abs(target - refs[i].last_sent) < 4) {
                target = refs[i].last_sent;
            } else {
                refs[i].last_sent = target;
            }

            pkt.ids[i] = fs->id;
            pkt.positions[i] = target;
        }

        sendto(sock,
               &pkt,
               sizeof(pkt),
               0,
               reinterpret_cast<sockaddr*>(&follower_addr),
               sizeof(follower_addr));

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    close(sock);
    leader_io.end();
    return 0;
}