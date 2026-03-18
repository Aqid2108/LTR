#include <iostream>
#include <string>
#include <limits>

int run_calibrate_all_mode();
int run_create_mapping_mode();
int run_leader_mode(const std::string& leader_yaml,
                    const std::string& follower_yaml,
                    const std::string& mapping_yaml,
                    const std::string& follower_ip,
                    int follower_port);
int run_follower_mode(const std::string& follower_yaml, int listen_port);
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

        int choice;
        std::cin >> choice;
        std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

        if (choice == 1) {
            run_calibrate_all_mode();
        }
        else if (choice == 2) {
            run_create_mapping_mode();
        }
        else if (choice == 3) {
            std::string leader_yaml, follower_yaml, mapping_yaml, follower_ip;
            int follower_port;

            std::cout << "\n=== Leader Mode ===\n";
            std::cout << "Leader YAML path: ";
            std::cin >> leader_yaml;
            std::cout << "Follower YAML path: ";
            std::cin >> follower_yaml;
            std::cout << "Mapping YAML path: ";
            std::cin >> mapping_yaml;
            std::cout << "Follower IP: ";
            std::cin >> follower_ip;
            std::cout << "Follower port: ";
            std::cin >> follower_port;

            run_leader_mode(leader_yaml, follower_yaml, mapping_yaml, follower_ip, follower_port);
        }
        else if (choice == 4) {
            std::string follower_yaml;
            int listen_port;

            std::cout << "\n=== Follower Mode ===\n";
            std::cout << "Follower YAML path: ";
            std::cin >> follower_yaml;
            std::cout << "Listen port: ";
            std::cin >> listen_port;

            run_follower_mode(follower_yaml, listen_port);
        }
        else if (choice == 5) {
            run_ping_servos();
            break;
        }
        else if (choice == 6) {
            break;
        }
        else {
            std::cout << "Invalid choice.\n";
        }
    }

    return 0;
}