#include "lerobot_hybrid.hpp"
#include "common_servo.hpp"
#include "follow_protocol.hpp"
#include "SCServo.h"
#include "SMS_STS.h"

#include <yaml-cpp/yaml.h>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <limits>
#include <map>
#include <string>
#include <thread>
#include <vector>

namespace {

struct JointRef {
    int servo_id = 0;
    std::string joint_name;
    int leader_start = 0;
    int follower_start = 0;
    int last_sent = std::numeric_limits<int>::min();
};

int clamp_int(int v, int lo, int hi) {
    return std::max(lo, std::min(hi, v));
}

double signed_span(const ServoConfig& servo) {
    double span = static_cast<double>(servo.max_pos - servo.min_pos);
    if (std::abs(span) < 1.0) {
        return (span < 0.0) ? -1.0 : 1.0;
    }
    return span;
}

int clamp_to_servo_range(const ServoConfig& servo, int raw) {
    int lo = std::min(servo.min_pos, servo.max_pos);
    int hi = std::max(servo.min_pos, servo.max_pos);
    return clamp_int(raw, lo, hi);
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
    double leader_span = signed_span(leader_servo);
    double norm =
        (static_cast<double>(leader_now) - static_cast<double>(leader_servo.min_pos)) / leader_span;

    norm = std::clamp(norm, 0.0, 1.0);

    bool effective_invert = leader_servo.invert ^ follower_servo.invert;
    if (effective_invert) {
        norm = 1.0 - norm;
    }

    double follower_span = signed_span(follower_servo);

    int target = static_cast<int>(std::lround(
        static_cast<double>(follower_servo.min_pos) + norm * follower_span
    ));

    return clamp_to_servo_range(follower_servo, target);
}

int compute_relative_target(const ServoConfig& leader_servo,
                            const ServoConfig& follower_servo,
                            int leader_now,
                            const JointRef& ref) {
    double leader_span = signed_span(leader_servo);
    double delta_norm =
        (static_cast<double>(leader_now) - static_cast<double>(ref.leader_start)) / leader_span;

    bool effective_invert = leader_servo.invert ^ follower_servo.invert;
    if (effective_invert) {
        delta_norm = -delta_norm;
    }

    double follower_span = signed_span(follower_servo);

    int target = static_cast<int>(std::lround(
        static_cast<double>(ref.follower_start) + delta_norm * follower_span
    ));

    return clamp_to_servo_range(follower_servo, target);
}

bool send_follow_packet(int sock,
                        const sockaddr_in& follower_addr,
                        const FollowPacketData& pkt) {
    FollowWireMessage msg{};
    msg.type = FollowPacketType::Command;
    msg.data = pkt;

    std::array<unsigned char, kFollowWirePacketSize> buf{};
    serialize_follow_message(msg, buf);

    ssize_t sent = sendto(sock,
                          buf.data(),
                          buf.size(),
                          0,
                          reinterpret_cast<const sockaddr*>(&follower_addr),
                          sizeof(follower_addr));
    return sent >= 0;
}

bool wait_for_handshake_ack(int sock, const sockaddr_in& follower_addr) {
    FollowWireMessage hello{};
    hello.type = FollowPacketType::HandshakeRequest;

    std::array<unsigned char, kFollowWirePacketSize> hello_buf{};
    serialize_follow_message(hello, hello_buf);

    ssize_t sent = sendto(sock,
                          hello_buf.data(),
                          hello_buf.size(),
                          0,
                          reinterpret_cast<const sockaddr*>(&follower_addr),
                          sizeof(follower_addr));
    if (sent < 0) {
        return false;
    }

    timeval timeout{};
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    std::array<unsigned char, kFollowWirePacketSize> reply_buf{};
    sockaddr_in sender_addr{};
    socklen_t sender_len = sizeof(sender_addr);
    ssize_t bytes = recvfrom(sock,
                             reply_buf.data(),
                             reply_buf.size(),
                             0,
                             reinterpret_cast<sockaddr*>(&sender_addr),
                             &sender_len);

    timeout.tv_sec = 0;
    timeout.tv_usec = 0;
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    if (bytes < 0) {
        return false;
    }

    FollowWireMessage reply{};
    if (!deserialize_follow_message(reply_buf.data(), static_cast<std::size_t>(bytes), reply)) {
        return false;
    }

    return reply.type == FollowPacketType::HandshakeAck;
}

std::map<int, ServoConfig> build_servo_map(const std::vector<ServoConfig>& servos) {
    std::map<int, ServoConfig> out;
    for (const auto& servo : servos) {
        out[servo.id] = servo;
    }
    return out;
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

    CalibrationData leader_cfg{};
    leader_cfg.port = leader_port;
    leader_cfg.baudrate = leader_baudrate;
    leader_cfg.servos = leader_servos;

    CalibrationData follower_cfg{};
    follower_cfg.port = follower_port_name;
    follower_cfg.baudrate = follower_baudrate;
    follower_cfg.servos = follower_servos;

    std::string leader_validation_error;
    std::vector<int> leader_ids;
    if (!validate_expected_arm_servos(leader_cfg, leader_validation_error, &leader_ids)) {
        std::cerr << "Leader YAML validation failed: " << leader_validation_error << "\n";
        return 1;
    }

    std::string follower_validation_error;
    std::vector<int> follower_ids;
    if (!validate_expected_arm_servos(follower_cfg, follower_validation_error, &follower_ids)) {
        std::cerr << "Follower YAML validation failed: " << follower_validation_error << "\n";
        return 1;
    }

    if (leader_ids != follower_ids) {
        std::cerr << "Leader/follower servo ID mismatch.\n";
        std::cerr << "Leader IDs: [" << join_ints(leader_ids) << "]\n";
        std::cerr << "Follower IDs: [" << join_ints(follower_ids) << "]\n";
        return 1;
    }

    if (leader_servos.size() > 16) {
        std::cerr << "Too many servos. Max supported is 16.\n";
        return 1;
    }

    std::cout << "\n=== Hybrid Follow Preflight ===\n";
    std::cout << "Recommended workflow:\n";
    std::cout << "  1. Run option 1 on each arm to calibrate all 6 servos\n";
    std::cout << "  2. Run option 4 first on the follower laptop\n";
    std::cout << "  3. Run option 6 second on the leader laptop\n";
    std::cout << "\nLeader YAML: " << leader_yaml << "\n";
    std::cout << "Follower YAML: " << follower_yaml << "\n";
    std::cout << "Leader port: " << leader_port << "\n";
    std::cout << "Leader baudrate: " << leader_baudrate << "\n";
    std::cout << "Follower port from YAML: " << follower_port_name << "\n";
    std::cout << "Follower baudrate: " << follower_baudrate << "\n";
    std::cout << "Follower IP: " << follower_ip << "\n";
    std::cout << "Follower UDP port: " << follower_port << "\n";
    std::cout << "Hybrid pairing mode: match by servo ID\n";
    std::cout << "Hybrid mode does NOT use a mapping YAML.\n";

    const std::map<int, ServoConfig> leader_by_id = build_servo_map(leader_servos);
    const std::map<int, ServoConfig> follower_by_id = build_servo_map(follower_servos);

    std::vector<int> paired_ids = leader_ids;
    std::cout << "\nMatched joint table:\n";
    for (int id : paired_ids) {
        const ServoConfig& ls = leader_by_id.at(id);
        const ServoConfig& fs = follower_by_id.at(id);
        std::cout << "  ID " << id
                  << " (" << joint_name_for_servo_id(id) << ")"
                  << " | leader=" << ls.name
                  << " [" << ls.min_pos << "," << ls.max_pos << "]"
                  << " | follower=" << fs.name
                  << " [" << fs.min_pos << "," << fs.max_pos << "]\n";
    }

    std::cout << "\nIMPORTANT:\n";
    std::cout << "Start option 4 on the follower laptop before continuing.\n";
    std::cout << "Place both arms in IDENTICAL physical positions before continuing.\n";
    std::cout << "Match these joints: base, shoulder, elbow, wrist pitch, wrist roll, gripper.\n";
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

    std::cout << "\nChecking follower UDP link...\n";
    if (!wait_for_handshake_ack(sock, follower_addr)) {
        std::cerr << "Follower did not acknowledge the UDP handshake.\n";
        std::cerr << "Check follower IP/port, firewall rules, and ensure option 4 is already running.\n";
        close(sock);
        leader_io.end();
        return 1;
    }
    std::cout << "Follower handshake ACK received.\n";

    std::vector<JointRef> refs(paired_ids.size());

    // Phase 1: capture leader reference and compute initial follower alignment.
    std::cout << "\nCapturing leader reference pose and preparing initial follower alignment...\n";
    std::cout << "Do NOT touch the arms.\n";

    FollowPacketData align_pkt{};
    align_pkt.count = static_cast<int>(paired_ids.size());

    for (size_t i = 0; i < paired_ids.size(); ++i) {
        int id = paired_ids[i];
        const ServoConfig& ls = leader_by_id.at(id);
        const ServoConfig& fs = follower_by_id.at(id);

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

        refs[i].servo_id = id;
        refs[i].joint_name = joint_name_for_servo_id(id);
        refs[i].leader_start = leader_now;
        refs[i].follower_start = follower_target;
        refs[i].last_sent = follower_target;

        std::cout << "Joint " << (i + 1)
                  << " (" << refs[i].joint_name << ")"
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

    // Phase 2: report captured reference
    std::cout << "\nAlignment complete.\n";
    std::cout << "Captured reference pose:\n";

    for (size_t i = 0; i < refs.size(); ++i) {
        std::cout << "Joint " << (i + 1)
                  << " (" << refs[i].joint_name << ")"
                  << " | leader_start=" << refs[i].leader_start
                  << " | follower_start=" << refs[i].follower_start
                  << "\n";
    }

    std::cout << "\nStarting smooth relative follow...\n";
    std::cout << "Move the leader arm by hand.\n";
    std::cout << "Press Ctrl+C to stop.\n";

    // Phase 3: relative follow
    while (true) {
        FollowPacketData pkt{};
        pkt.count = static_cast<int>(paired_ids.size());

        for (size_t i = 0; i < paired_ids.size(); ++i) {
            int id = paired_ids[i];
            const ServoConfig& ls = leader_by_id.at(id);
            const ServoConfig& fs = follower_by_id.at(id);

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
