#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace sensor_sync {

enum class SensorType {
    Camera,
    Imu,
};

struct SensorPacket {
    SensorType type;
    std::uint64_t sequence;
    std::int64_t timestamp_us;
    std::string payload;
};

struct Config {
    std::filesystem::path camera_file;
    std::filesystem::path imu_file;
    std::filesystem::path output_file;
    std::size_t queue_capacity{8};
    std::uint64_t max_delta_us{1000};
    std::uint64_t camera_delay_us{10000};
    std::uint64_t imu_delay_us{2500};
};

struct MatchResult {
    SensorPacket camera;
    std::optional<SensorPacket> imu;
    std::uint64_t delta_us{0};
};

Config load_config(const std::filesystem::path& config_file);

std::vector<MatchResult> match_packets(
    const std::vector<SensorPacket>& cameras,
    const std::vector<SensorPacket>& imus,
    std::uint64_t max_delta_us);

std::vector<std::string> format_results(
    const std::vector<MatchResult>& matches);

int run(const std::filesystem::path& config_file);

}  // namespace sensor_sync
