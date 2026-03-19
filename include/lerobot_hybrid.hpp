#pragma once

#include <string>

int run_lerobot_hybrid_mode(const std::string& leader_yaml,
                            const std::string& follower_yaml,
                            const std::string& mapping_yaml,
                            const std::string& follower_ip,
                            int follower_port);