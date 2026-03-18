#include <iostream>
#include <string>

void run_calibration();
void run_create_mapping();
void run_leader();
void run_follower();
void run_ping_servos();

int main() {
    while (true) {
        std::cout << "\n=== STS3215 Leader-Follower Menu ===\n";
        std::cout << "1. Calibrate all 6 servos\n";
        std::cout << "2. Create leader-follower mapping/alignment\n";
        std::cout << "3. Run leader\n";
        std::cout << "4. Run follower\n";
        std::cout << "5. Ping servos\n";
        std::cout << "6. Exit\n";
        std::cout << "Enter choice: ";

        std::string choice;
        std::getline(std::cin, choice);

        try {
            if (choice == "1") {
                run_calibration();
            } else if (choice == "2") {
                run_create_mapping();
            } else if (choice == "3") {
                run_leader();
            } else if (choice == "4") {
                run_follower();
            } else if (choice == "5") {
                run_ping_servos();
            } else if (choice == "6") {
                return 0;
            } else {
                std::cout << "Invalid choice.\n";
            }
        } catch (const std::exception& e) {
            std::cerr << "Error: " << e.what() << "\n";
        }
    }
}