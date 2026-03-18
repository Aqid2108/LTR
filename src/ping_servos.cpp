#include <iostream>
#include <string>
#include <vector>

#include "SCServo.h"
#include "SMS_STS.h"

void run_ping_servos() {
    std::string port;
    int baudrate = 1000000;
    int start_id = 1;
    int end_id = 20;

    std::cout << "\n=== Ping Servos ===\n";
    std::cout << "Port (example /dev/ttyACM0): ";
    std::getline(std::cin, port);

    std::cout << "Baudrate [default 1000000]: ";
    std::string baud_str;
    std::getline(std::cin, baud_str);
    if (!baud_str.empty()) baudrate = std::stoi(baud_str);

    SMS_STS sms;
    if (!sms.begin(baudrate, port.c_str())) {
        std::cerr << "Failed to open serial port: " << port << "\n";
        return;
    }

    std::vector<int> found_ids;
    for (int id = start_id; id <= end_id; ++id) {
        int pos = sms.ReadPos(id);
        if (pos >= 0) {
            found_ids.push_back(id);
            std::cout << "Found servo ID " << id << " | position = " << pos << "\n";
        }
    }

    if (found_ids.empty()) {
        std::cout << "No servos responded.\n";
    }
}