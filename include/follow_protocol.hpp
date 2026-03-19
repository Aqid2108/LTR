#pragma once

#include <array>
#include <cstdint>
#include <cstddef>
#include <cstring>

#include <arpa/inet.h>

constexpr std::uint32_t kFollowMagic = 0x46504B54u; // "FPKT"
constexpr std::uint16_t kFollowVersion = 1;
constexpr std::size_t kFollowMaxJoints = 16;

enum class FollowPacketType : std::uint16_t {
    HandshakeRequest = 1,
    HandshakeAck = 2,
    Command = 3,
};

struct FollowPacketData {
    int count = 0;
    int ids[kFollowMaxJoints]{};
    int positions[kFollowMaxJoints]{};
};

struct FollowWireMessage {
    FollowPacketType type = FollowPacketType::Command;
    FollowPacketData data{};
};

constexpr std::size_t kFollowWirePacketSize =
    4 + 2 + 2 + 4 + (kFollowMaxJoints * 4) + (kFollowMaxJoints * 4);

inline bool serialize_follow_message(const FollowWireMessage& msg,
                                     std::array<unsigned char, kFollowWirePacketSize>& out) {
    std::memset(out.data(), 0, out.size());

    std::uint32_t magic = htonl(kFollowMagic);
    std::uint16_t version = htons(kFollowVersion);
    std::uint16_t type = htons(static_cast<std::uint16_t>(msg.type));
    std::uint32_t count = htonl(static_cast<std::uint32_t>(msg.data.count));

    std::size_t offset = 0;
    std::memcpy(out.data() + offset, &magic, sizeof(magic));
    offset += sizeof(magic);
    std::memcpy(out.data() + offset, &version, sizeof(version));
    offset += sizeof(version);
    std::memcpy(out.data() + offset, &type, sizeof(type));
    offset += sizeof(type);
    std::memcpy(out.data() + offset, &count, sizeof(count));
    offset += sizeof(count);

    for (std::size_t i = 0; i < kFollowMaxJoints; ++i) {
        std::uint32_t id = htonl(static_cast<std::uint32_t>(msg.data.ids[i]));
        std::memcpy(out.data() + offset, &id, sizeof(id));
        offset += sizeof(id);
    }

    for (std::size_t i = 0; i < kFollowMaxJoints; ++i) {
        std::uint32_t pos = htonl(static_cast<std::uint32_t>(msg.data.positions[i]));
        std::memcpy(out.data() + offset, &pos, sizeof(pos));
        offset += sizeof(pos);
    }

    return true;
}

inline bool deserialize_follow_message(const unsigned char* bytes,
                                       std::size_t len,
                                       FollowWireMessage& out) {
    if (len != kFollowWirePacketSize) {
        return false;
    }

    std::size_t offset = 0;

    std::uint32_t magic = 0;
    std::uint16_t version = 0;
    std::uint16_t type = 0;
    std::uint32_t count = 0;

    std::memcpy(&magic, bytes + offset, sizeof(magic));
    offset += sizeof(magic);
    std::memcpy(&version, bytes + offset, sizeof(version));
    offset += sizeof(version);
    std::memcpy(&type, bytes + offset, sizeof(type));
    offset += sizeof(type);
    std::memcpy(&count, bytes + offset, sizeof(count));
    offset += sizeof(count);

    magic = ntohl(magic);
    version = ntohs(version);
    type = ntohs(type);
    count = ntohl(count);

    if (magic != kFollowMagic || version != kFollowVersion) {
        return false;
    }

    if (count > kFollowMaxJoints) {
        return false;
    }

    out = FollowWireMessage{};
    out.type = static_cast<FollowPacketType>(type);
    out.data.count = static_cast<int>(count);

    for (std::size_t i = 0; i < kFollowMaxJoints; ++i) {
        std::uint32_t id = 0;
        std::memcpy(&id, bytes + offset, sizeof(id));
        offset += sizeof(id);
        out.data.ids[i] = static_cast<int>(ntohl(id));
    }

    for (std::size_t i = 0; i < kFollowMaxJoints; ++i) {
        std::uint32_t pos = 0;
        std::memcpy(&pos, bytes + offset, sizeof(pos));
        offset += sizeof(pos);
        out.data.positions[i] = static_cast<int>(ntohl(pos));
    }

    return true;
}
