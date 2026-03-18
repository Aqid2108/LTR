#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <thread>
#include <chrono>
#include <cstring>
#include <map>
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

static void move_servo(SMS_STS& sm_st, int id, int pos, int speed = 800, int acc = 50) {
    sm_st.WritePosEx(id, pos, speed, acc);
}

static std::map<int, int> parse_targets(const std::string& line) {
    std::map<int, int> targets;
    std::stringstream ss(line);
    std::string token;

    while (std::getline(ss, token, ',')) {
        auto colon = token.find(':');
        if (colon == std::string::npos) continue;

        int id = std::stoi(token.substr(0, colon));
        int pos = std::stoi(token.substr(colon + 1));
        targets[id] = pos;
    }

    return targets;
}

int run_follower_mode(const std::string& follower_yaml, int listen_port) {
    CalibrationData follower_cfg = load_calibration_yaml(follower_yaml);
    print_calibration("Follower calibration", follower_cfg);

    SMS_STS sm_st;
    if (!open_servo_port(sm_st, follower_cfg.port, follower_cfg.baudrate)) {
        return 1;
    }

    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        std::cerr << "Socket creation failed.\n";
        return 1;
    }

    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(listen_port);

    if (bind(server_fd, (sockaddr*)&address, sizeof(address)) < 0) {
        std::cerr << "Bind failed.\n";
        close(server_fd);
        return 1;
    }

    if (listen(server_fd, 1) < 0) {
        std::cerr << "Listen failed.\n";
        close(server_fd);
        return 1;
    }

    std::cout << "Follower listening on port " << listen_port << " ...\n";

    sockaddr_in client_addr{};
    socklen_t client_len = sizeof(client_addr);
    int client_fd = accept(server_fd, (sockaddr*)&client_addr, &client_len);
    if (client_fd < 0) {
        std::cerr << "Accept failed.\n";
        close(server_fd);
        return 1;
    }

    std::cout << "Leader connected.\n";

    char buffer[2048];
    std::string pending;

    while (true) {
        ssize_t bytes = recv(client_fd, buffer, sizeof(buffer) - 1, 0);
        if (bytes <= 0) {
            std::cerr << "Leader disconnected.\n";
            break;
        }

        buffer[bytes] = '\0';
        pending += buffer;

        size_t newline_pos;
        while ((newline_pos = pending.find('\n')) != std::string::npos) {
            std::string line = pending.substr(0, newline_pos);
            pending.erase(0, newline_pos + 1);

            auto targets = parse_targets(line);

            for (const auto& s : follower_cfg.servos) {
                auto it = targets.find(s.id);
                if (it != targets.end()) {
                    int target = clamp_int(
                        it->second,
                        std::min(s.min_pos, s.max_pos),
                        std::max(s.min_pos, s.max_pos)
                    );
                    move_servo(sm_st, s.id, target);
                }
            }
        }
    }

    close(client_fd);
    close(server_fd);
    return 0;
}