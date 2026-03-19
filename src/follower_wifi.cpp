#include <iostream>
#include <vector>
#include <string>
#include <cstring>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include "common_servo.hpp"
#include "SCServo.h"
#include "SMS_STS.h"

struct FollowPacket {
    int count;
    int ids[16];
    int positions[16];
};

static bool open_servo_port(SMS_STS& sm_st, const std::string& port, int baudrate) {
    if (sm_st.begin(baudrate, port.c_str())) {
        return true;
    }
    std::cerr << "Failed to open serial port: " << port << "\n";
    return false;
}

static void move_servo(SMS_STS& sm_st, int id, int pos, int speed = 800, int acc = 50) {
    sm_st.WritePosEx(id, pos, speed, acc);
}

static int read_servo_position_retry(SMS_STS& sm_st, int id, int attempts = 8) {
    for (int i = 0; i < attempts; ++i) {
        int pos = sm_st.ReadPos(id);
        if (pos >= 0) return pos;
    }
    return -1;
}

int run_follower_mode(const std::string& follower_yaml, int listen_port) {
    CalibrationData follower_cfg = load_calibration_yaml(follower_yaml);
    std::string validation_error;
    std::vector<int> sorted_ids;
    if (!validate_expected_arm_servos(follower_cfg, validation_error, &sorted_ids)) {
        std::cerr << "Follower YAML validation failed: " << validation_error << "\n";
        return 1;
    }

    std::cout << "\n=== Follower Startup Preflight ===\n";
    print_calibration("Follower calibration", follower_cfg);
    std::cout << "Expected servo IDs: [" << expected_arm_servo_ids_text() << "]\n";
    std::cout << "Detected servo IDs in YAML: [" << join_ints(sorted_ids) << "]\n";
    std::cout << "Follower UDP listen port: " << listen_port << "\n";

    SMS_STS sm_st;
    if (!open_servo_port(sm_st, follower_cfg.port, follower_cfg.baudrate)) {
        return 1;
    }

    std::vector<int> reachable_ids;
    for (int id : sorted_ids) {
        int pos = read_servo_position_retry(sm_st, id, 8);
        if (pos < 0) {
            std::cerr << "Follower servo ID " << id << " is configured but not reachable.\n";
            return 1;
        }
        reachable_ids.push_back(id);
        std::cout << "  Reachable follower servo ID " << id
                  << " (" << joint_name_for_servo_id(id) << ")"
                  << " at raw position " << pos << "\n";
    }
    std::cout << "Reachable follower servo count: " << reachable_ids.size() << "\n";

    int sock_fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock_fd < 0) {
        std::cerr << "UDP socket creation failed.\n";
        return 1;
    }

    int opt = 1;
    setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(listen_port);

    if (bind(sock_fd, (sockaddr*)&address, sizeof(address)) < 0) {
        std::cerr << "UDP bind failed on port " << listen_port << ".\n";
        close(sock_fd);
        return 1;
    }

    std::cout << "Follower listening on UDP port " << listen_port << " ...\n";
    std::cout << "Follower ready for leader option 6.\n";
    std::cout << "Waiting for leader packets...\n";

    while (true) {
        FollowPacket pkt{};
        sockaddr_in sender_addr{};
        socklen_t sender_len = sizeof(sender_addr);

        ssize_t bytes = recvfrom(
            sock_fd,
            &pkt,
            sizeof(pkt),
            0,
            (sockaddr*)&sender_addr,
            &sender_len
        );

        if (bytes < 0) {
            std::cerr << "recvfrom failed.\n";
            continue;
        }

        if (bytes < (ssize_t)sizeof(int)) {
            std::cerr << "Received packet too small.\n";
            continue;
        }

        std::cout << "\nReceived packet with " << pkt.count << " joints\n";

        for (int i = 0; i < pkt.count && i < 16; ++i) {
            int id = pkt.ids[i];
            int target = pkt.positions[i];

            bool found = false;
            for (const auto& s : follower_cfg.servos) {
                if (s.id == id) {
                    target = clamp_int(
                        target,
                        std::min(s.min_pos, s.max_pos),
                        std::max(s.min_pos, s.max_pos)
                    );

                    std::cout << "  ID " << id << " -> " << target << "\n";
                    move_servo(sm_st, id, target);
                    found = true;
                    break;
                }
            }

            if (!found) {
                std::cerr << "Unknown servo ID in packet: " << id << "\n";
            }
        }
    }

    close(sock_fd);
    return 0;
}

void run_follower() {
    // unused stub
}
