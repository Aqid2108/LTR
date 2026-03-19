#pragma once

#include <string>
#include <vector>
#include <fstream>
#include <iostream>
#include <cmath>
#include <algorithm>
#include <array>
#include <set>
#include <sstream>
#include <stdexcept>
#include <yaml-cpp/yaml.h>

struct ServoConfig {
    std::string name;
    int id = 0;
    int min_pos = 0;
    int max_pos = 4095;
    bool invert = false;
};

struct CalibrationData {
    std::string port;
    int baudrate = 1000000;
    std::vector<ServoConfig> servos;
};

struct JointMap {
    std::string joint;
    int leader_id = 0;
    int follower_id = 0;
    bool invert = false;
    int leader_ref = 0;
    int follower_ref = 0;
    int trim = 0;
};

struct MappingData {
    std::string leader_yaml;
    std::string follower_yaml;
    std::vector<JointMap> pairs;
};

inline int clamp_int(int x, int lo, int hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

inline double clamp_double(double x, double lo, double hi) {
    if (x < lo) return lo;
    if (x > hi) return hi;
    return x;
}

inline CalibrationData load_calibration_yaml(const std::string& path) {
    CalibrationData cfg;
    YAML::Node root = YAML::LoadFile(path);

    cfg.port = root["port"] ? root["port"].as<std::string>() : "/dev/ttyACM0";
    cfg.baudrate = root["baudrate"] ? root["baudrate"].as<int>() : 1000000;

    if (!root["servos"] || !root["servos"].IsSequence()) {
        throw std::runtime_error("Invalid YAML: missing 'servos' sequence");
    }

    for (const auto& s : root["servos"]) {
        ServoConfig sc;
        sc.name = s["name"] ? s["name"].as<std::string>() : "";
        sc.id = s["id"] ? s["id"].as<int>() : 0;
        sc.min_pos = s["min"] ? s["min"].as<int>() : 0;
        sc.max_pos = s["max"] ? s["max"].as<int>() : 4095;
        sc.invert = s["invert"] ? s["invert"].as<bool>() : false;
        cfg.servos.push_back(sc);
    }

    return cfg;
}

inline void save_calibration_yaml(const std::string& path, const CalibrationData& cfg) {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "port" << YAML::Value << cfg.port;
    out << YAML::Key << "baudrate" << YAML::Value << cfg.baudrate;
    out << YAML::Key << "servos" << YAML::Value << YAML::BeginSeq;

    for (const auto& s : cfg.servos) {
        out << YAML::BeginMap;
        out << YAML::Key << "name" << YAML::Value << s.name;
        out << YAML::Key << "id" << YAML::Value << s.id;
        out << YAML::Key << "min" << YAML::Value << s.min_pos;
        out << YAML::Key << "max" << YAML::Value << s.max_pos;
        out << YAML::Key << "invert" << YAML::Value << s.invert;
        out << YAML::EndMap;
    }

    out << YAML::EndSeq;
    out << YAML::EndMap;

    std::ofstream fout(path);
    if (!fout.is_open()) {
        throw std::runtime_error("Failed to open YAML output file: " + path);
    }
    fout << out.c_str();
}

inline MappingData load_mapping_yaml(const std::string& path) {
    MappingData map;
    YAML::Node root = YAML::LoadFile(path);

    map.leader_yaml = root["leader_yaml"] ? root["leader_yaml"].as<std::string>() : "";
    map.follower_yaml = root["follower_yaml"] ? root["follower_yaml"].as<std::string>() : "";

    if (!root["pairs"] || !root["pairs"].IsSequence()) {
        throw std::runtime_error("Invalid YAML: missing 'pairs' sequence");
    }

    for (const auto& p : root["pairs"]) {
        JointMap jm;
        jm.joint = p["joint"] ? p["joint"].as<std::string>() : "";
        jm.leader_id = p["leader_id"] ? p["leader_id"].as<int>() : 0;
        jm.follower_id = p["follower_id"] ? p["follower_id"].as<int>() : 0;
        jm.invert = p["invert"] ? p["invert"].as<bool>() : false;
        jm.leader_ref = p["leader_ref"] ? p["leader_ref"].as<int>() : 0;
        jm.follower_ref = p["follower_ref"] ? p["follower_ref"].as<int>() : 0;
        jm.trim = p["trim"] ? p["trim"].as<int>() : 0;
        map.pairs.push_back(jm);
    }

    return map;
}

inline void save_mapping_yaml(const std::string& path, const MappingData& map) {
    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "leader_yaml" << YAML::Value << map.leader_yaml;
    out << YAML::Key << "follower_yaml" << YAML::Value << map.follower_yaml;
    out << YAML::Key << "pairs" << YAML::Value << YAML::BeginSeq;

    for (const auto& p : map.pairs) {
        out << YAML::BeginMap;
        out << YAML::Key << "joint" << YAML::Value << p.joint;
        out << YAML::Key << "leader_id" << YAML::Value << p.leader_id;
        out << YAML::Key << "follower_id" << YAML::Value << p.follower_id;
        out << YAML::Key << "invert" << YAML::Value << p.invert;
        out << YAML::Key << "leader_ref" << YAML::Value << p.leader_ref;
        out << YAML::Key << "follower_ref" << YAML::Value << p.follower_ref;
        out << YAML::Key << "trim" << YAML::Value << p.trim;
        out << YAML::EndMap;
    }

    out << YAML::EndSeq;
    out << YAML::EndMap;

    std::ofstream fout(path);
    if (!fout.is_open()) {
        throw std::runtime_error("Failed to open mapping YAML output file: " + path);
    }
    fout << out.c_str();
}

inline const ServoConfig* find_servo_by_id(const CalibrationData& cfg, int id) {
    for (const auto& s : cfg.servos) {
        if (s.id == id) return &s;
    }
    return nullptr;
}

inline void print_calibration(const std::string& label, const CalibrationData& cfg) {
    std::cout << label << ":\n";
    std::cout << "Port: " << cfg.port << "\n";
    std::cout << "Baudrate: " << cfg.baudrate << "\n";
    std::cout << "Servo count: " << cfg.servos.size() << "\n";
    for (const auto& s : cfg.servos) {
        std::cout
            << "  - " << s.name
            << " | id=" << s.id
            << " | min=" << s.min_pos
            << " | max=" << s.max_pos
            << " | invert=" << (s.invert ? "true" : "false")
            << "\n";
    }
    std::cout << std::endl;
}

inline void print_mapping(const std::string& label, const MappingData& map) {
    std::cout << label << ":\n";
    std::cout << "Leader YAML: " << map.leader_yaml << "\n";
    std::cout << "Follower YAML: " << map.follower_yaml << "\n";
    std::cout << "Pair count: " << map.pairs.size() << "\n";
    for (const auto& p : map.pairs) {
        std::cout
            << "  - " << p.joint
            << " | leader_id=" << p.leader_id
            << " | follower_id=" << p.follower_id
            << " | invert=" << (p.invert ? "true" : "false")
            << " | leader_ref=" << p.leader_ref
            << " | follower_ref=" << p.follower_ref
            << " | trim=" << p.trim
            << "\n";
    }
    std::cout << std::endl;
}

inline std::vector<int> expected_arm_servo_ids() {
    return {1, 2, 3, 4, 5, 6};
}

inline std::string expected_arm_servo_ids_text() {
    return "1, 2, 3, 4, 5, 6";
}

inline std::string joint_name_for_servo_id(int id) {
    switch (id) {
        case 1: return "base";
        case 2: return "shoulder";
        case 3: return "elbow";
        case 4: return "wrist_pitch";
        case 5: return "wrist_roll";
        case 6: return "gripper";
        default: return "servo_" + std::to_string(id);
    }
}

inline std::string join_ints(const std::vector<int>& values) {
    std::ostringstream oss;
    for (size_t i = 0; i < values.size(); ++i) {
        if (i > 0) oss << ", ";
        oss << values[i];
    }
    return oss.str();
}

inline bool validate_expected_arm_servos(const CalibrationData& cfg,
                                         std::string& error,
                                         std::vector<int>* sorted_ids_out = nullptr) {
    std::vector<int> ids;
    ids.reserve(cfg.servos.size());

    std::set<int> seen;
    for (const auto& servo : cfg.servos) {
        ids.push_back(servo.id);
        if (!seen.insert(servo.id).second) {
            error = "Duplicate servo ID found in YAML: " + std::to_string(servo.id);
            return false;
        }
    }

    std::sort(ids.begin(), ids.end());
    if (sorted_ids_out) {
        *sorted_ids_out = ids;
    }

    const std::vector<int> expected = expected_arm_servo_ids();
    if (ids.size() != expected.size()) {
        error = "Expected exactly 6 servos with IDs [" + expected_arm_servo_ids_text() +
                "], but found " + std::to_string(ids.size()) +
                " servo(s): [" + join_ints(ids) + "]";
        return false;
    }

    if (ids != expected) {
        error = "Expected servo IDs [" + expected_arm_servo_ids_text() +
                "], but found [" + join_ints(ids) + "]";
        return false;
    }

    return true;
}
