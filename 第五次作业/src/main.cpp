#include "sensor_sync.hpp"
#include "thread_safe_queue.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <exception>
#include <fstream>
#include <iostream>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <unordered_map>
#include <utility>

namespace sensor_sync {
namespace {

std::string trim(std::string value) {
    const auto first = std::find_if_not(value.begin(), value.end(), [](unsigned char ch) {
        return std::isspace(ch) != 0;
    });
    const auto last = std::find_if_not(value.rbegin(), value.rend(), [](unsigned char ch) {
        return std::isspace(ch) != 0;
    }).base();

    if (first >= last) {
        return {};
    }
    return std::string(first, last);
}

std::uint64_t parse_unsigned(const std::string& value, const std::string& key) {
    if (value.empty() || value.front() == '-') {
        throw std::runtime_error("invalid unsigned value for " + key + ": " + value);
    }
    std::size_t consumed = 0;
    unsigned long long parsed = 0;
    try {
        parsed = std::stoull(value, &consumed);
    } catch (const std::exception&) {
        throw std::runtime_error("invalid unsigned value for " + key + ": " + value);
    }
    if (consumed != value.size()) {
        throw std::runtime_error("invalid unsigned value for " + key + ": " + value);
    }
    return static_cast<std::uint64_t>(parsed);
}

std::int64_t parse_signed(const std::string& value, const std::string& key) {
    std::size_t consumed = 0;
    long long parsed = 0;
    try {
        parsed = std::stoll(value, &consumed);
    } catch (const std::exception&) {
        throw std::runtime_error("invalid signed value for " + key + ": " + value);
    }
    if (consumed != value.size()) {
        throw std::runtime_error("invalid signed value for " + key + ": " + value);
    }
    return static_cast<std::int64_t>(parsed);
}

std::filesystem::path resolve_path(
    const std::filesystem::path& config_file,
    const std::string& value) {
    std::filesystem::path path(value);
    if (path.is_relative()) {
        path = config_file.parent_path() / path;
    }
    return path.lexically_normal();
}

SensorPacket parse_packet(
    const std::string& line,
    SensorType type,
    const std::filesystem::path& source,
    std::size_t line_number) {
    std::istringstream input(line);
    SensorPacket packet{type, 0, 0, {}};
    std::string sequence;
    std::string timestamp;
    if (!(input >> sequence >> timestamp)) {
        throw std::runtime_error(
            source.string() + ":" + std::to_string(line_number) +
            ": expected '<sequence> <timestamp_us> [payload]'");
    }

    const std::string location = source.string() + ":" + std::to_string(line_number);
    packet.sequence = parse_unsigned(sequence, "sequence at " + location);
    packet.timestamp_us = parse_signed(timestamp, "timestamp at " + location);

    std::getline(input, packet.payload);
    packet.payload = trim(packet.payload);
    return packet;
}

class ErrorState {
public:
    void capture(std::exception_ptr error) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!error_) {
            error_ = std::move(error);
        }
    }

    void rethrow_if_present() const {
        std::lock_guard<std::mutex> lock(mutex_);
        if (error_) {
            std::rethrow_exception(error_);
        }
    }

private:
    mutable std::mutex mutex_;
    std::exception_ptr error_;
};

class ProducerTracker {
public:
    ProducerTracker(int producer_count, ThreadSafeQueue<SensorPacket>& queue)
        : remaining_(producer_count), queue_(queue) {}

    void finished() {
        if (remaining_.fetch_sub(1) == 1) {
            queue_.close();
        }
    }

private:
    std::atomic<int> remaining_;
    ThreadSafeQueue<SensorPacket>& queue_;
};

class CompletionGuard {
public:
    explicit CompletionGuard(ProducerTracker& tracker) : tracker_(tracker) {}
    ~CompletionGuard() { tracker_.finished(); }

private:
    ProducerTracker& tracker_;
};

void produce_file(
    SensorType type,
    const std::filesystem::path& file,
    std::uint64_t delay_us,
    ThreadSafeQueue<SensorPacket>& queue,
    ProducerTracker& tracker,
    ErrorState& errors) noexcept {
    CompletionGuard completion(tracker);
    try {
        std::ifstream input(file);
        if (!input) {
            throw std::runtime_error("cannot open input file: " + file.string());
        }

        std::string line;
        std::size_t line_number = 0;
        while (std::getline(input, line)) {
            ++line_number;
            const std::string cleaned = trim(line);
            if (cleaned.empty() || cleaned.front() == '#') {
                continue;
            }

            if (!queue.push(parse_packet(cleaned, type, file, line_number))) {
                return;
            }

            if (delay_us > 0) {
                std::this_thread::sleep_for(std::chrono::microseconds(delay_us));
            }
        }

        if (!input.eof()) {
            throw std::runtime_error("failed while reading input file: " + file.string());
        }
    } catch (...) {
        errors.capture(std::current_exception());
        queue.close();
    }
}

std::uint64_t ordered_timestamp(std::int64_t timestamp) {
    constexpr std::uint64_t sign_bit = std::uint64_t{1} << 63U;
    return static_cast<std::uint64_t>(timestamp) ^ sign_bit;
}

std::uint64_t absolute_delta(std::int64_t left, std::int64_t right) {
    const auto left_ordered = ordered_timestamp(left);
    const auto right_ordered = ordered_timestamp(right);
    return left_ordered >= right_ordered
        ? left_ordered - right_ordered
        : right_ordered - left_ordered;
}

bool better_candidate(
    const SensorPacket& candidate,
    const SensorPacket& current,
    std::int64_t camera_timestamp) {
    const auto candidate_delta = absolute_delta(candidate.timestamp_us, camera_timestamp);
    const auto current_delta = absolute_delta(current.timestamp_us, camera_timestamp);
    if (candidate_delta != current_delta) {
        return candidate_delta < current_delta;
    }
    if (candidate.timestamp_us != current.timestamp_us) {
        return candidate.timestamp_us < current.timestamp_us;
    }
    return candidate.sequence < current.sequence;
}

void write_results(
    const std::filesystem::path& output_file,
    const std::vector<std::string>& lines) {
    if (!output_file.parent_path().empty()) {
        std::filesystem::create_directories(output_file.parent_path());
    }

    std::ofstream output(output_file);
    if (!output) {
        throw std::runtime_error("cannot open output file: " + output_file.string());
    }
    for (const auto& line : lines) {
        output << line << '\n';
    }
    if (!output) {
        throw std::runtime_error("failed while writing output file: " + output_file.string());
    }
}

}  // namespace

Config load_config(const std::filesystem::path& config_file) {
    std::ifstream input(config_file);
    if (!input) {
        throw std::runtime_error("cannot open config file: " + config_file.string());
    }

    std::unordered_map<std::string, std::string> values;
    std::string line;
    std::size_t line_number = 0;
    while (std::getline(input, line)) {
        ++line_number;
        const std::string cleaned = trim(line);
        if (cleaned.empty() || cleaned.front() == '#') {
            continue;
        }

        const auto separator = cleaned.find('=');
        if (separator == std::string::npos) {
            throw std::runtime_error(
                config_file.string() + ":" + std::to_string(line_number) +
                ": expected 'key=value'");
        }

        const std::string key = trim(cleaned.substr(0, separator));
        const std::string value = trim(cleaned.substr(separator + 1));
        if (key.empty() || value.empty()) {
            throw std::runtime_error(
                config_file.string() + ":" + std::to_string(line_number) +
                ": key and value must not be empty");
        }
        if (!values.emplace(key, value).second) {
            throw std::runtime_error("duplicate config key: " + key);
        }
    }

    const auto require = [&values](const std::string& key) -> const std::string& {
        const auto it = values.find(key);
        if (it == values.end()) {
            throw std::runtime_error("missing config key: " + key);
        }
        return it->second;
    };

    Config config;
    config.camera_file = resolve_path(config_file, require("camera_file"));
    config.imu_file = resolve_path(config_file, require("imu_file"));
    config.output_file = resolve_path(config_file, require("output_file"));
    config.queue_capacity = static_cast<std::size_t>(
        parse_unsigned(require("queue_capacity"), "queue_capacity"));
    config.max_delta_us = parse_unsigned(require("max_delta_us"), "max_delta_us");
    config.camera_delay_us = parse_unsigned(require("camera_delay_us"), "camera_delay_us");
    config.imu_delay_us = parse_unsigned(require("imu_delay_us"), "imu_delay_us");

    if (config.queue_capacity == 0) {
        throw std::runtime_error("queue_capacity must be greater than zero");
    }
    return config;
}

std::vector<MatchResult> match_packets(
    const std::vector<SensorPacket>& cameras,
    const std::vector<SensorPacket>& imus,
    std::uint64_t max_delta_us) {
    std::vector<SensorPacket> sorted_imus = imus;
    std::sort(sorted_imus.begin(), sorted_imus.end(), [](const auto& left, const auto& right) {
        if (left.timestamp_us != right.timestamp_us) {
            return left.timestamp_us < right.timestamp_us;
        }
        return left.sequence < right.sequence;
    });

    std::vector<MatchResult> results;
    results.reserve(cameras.size());
    for (const auto& camera : cameras) {
        MatchResult result{camera, std::nullopt, 0};
        if (!sorted_imus.empty()) {
            const auto next = std::lower_bound(
                sorted_imus.begin(), sorted_imus.end(), camera.timestamp_us,
                [](const SensorPacket& packet, std::int64_t timestamp) {
                    return packet.timestamp_us < timestamp;
                });

            const SensorPacket* best = nullptr;
            if (next != sorted_imus.end()) {
                best = &*next;
            }
            if (next != sorted_imus.begin()) {
                const auto previous_timestamp = std::prev(next)->timestamp_us;
                const auto previous_position = std::lower_bound(
                    sorted_imus.begin(), next, previous_timestamp,
                    [](const SensorPacket& packet, std::int64_t timestamp) {
                        return packet.timestamp_us < timestamp;
                    });
                const auto& previous = *previous_position;
                if (best == nullptr || better_candidate(previous, *best, camera.timestamp_us)) {
                    best = &previous;
                }
            }

            const auto delta = absolute_delta(camera.timestamp_us, best->timestamp_us);
            if (delta <= max_delta_us) {
                result.imu = *best;
                result.delta_us = delta;
            }
        }
        results.push_back(std::move(result));
    }
    return results;
}

std::vector<std::string> format_results(const std::vector<MatchResult>& matches) {
    std::vector<std::string> lines;
    lines.reserve(matches.size());
    for (const auto& match : matches) {
        std::ostringstream line;
        line << "CAMERA " << match.camera.sequence;
        if (match.imu) {
            line << " IMU " << match.imu->sequence
                 << " DELTA " << match.delta_us;
        } else {
            line << " UNMATCHED";
        }
        lines.push_back(line.str());
    }
    return lines;
}

int run(const std::filesystem::path& config_file) {
    const Config config = load_config(config_file);
    ThreadSafeQueue<SensorPacket> queue(config.queue_capacity);
    ProducerTracker tracker(2, queue);
    ErrorState errors;
    std::vector<SensorPacket> cameras;
    std::vector<SensorPacket> imus;

    std::thread consumer([&] {
        try {
            SensorPacket packet{SensorType::Camera, 0, 0, {}};
            while (queue.pop(packet)) {
                if (packet.type == SensorType::Camera) {
                    cameras.push_back(std::move(packet));
                } else {
                    imus.push_back(std::move(packet));
                }
            }
        } catch (...) {
            errors.capture(std::current_exception());
            queue.close();
        }
    });

    std::thread camera_producer(
        produce_file,
        SensorType::Camera,
        config.camera_file,
        config.camera_delay_us,
        std::ref(queue),
        std::ref(tracker),
        std::ref(errors));
    std::thread imu_producer(
        produce_file,
        SensorType::Imu,
        config.imu_file,
        config.imu_delay_us,
        std::ref(queue),
        std::ref(tracker),
        std::ref(errors));

    camera_producer.join();
    imu_producer.join();
    consumer.join();
    errors.rethrow_if_present();

    const auto matches = match_packets(cameras, imus, config.max_delta_us);
    const auto lines = format_results(matches);
    write_results(config.output_file, lines);
    for (const auto& line : lines) {
        std::cout << line << '\n';
    }
    return 0;
}

}  // namespace sensor_sync

int main(int argc, char* argv[]) {
    try {
        if (argc > 2) {
            throw std::runtime_error("usage: sensor_sync [config_file]");
        }
        const std::filesystem::path config_file =
            argc == 2 ? argv[1] : "config/sensor_sync.conf";
        return sensor_sync::run(config_file);
    } catch (const std::exception& error) {
        std::cerr << "sensor_sync: " << error.what() << '\n';
        return 1;
    }
}
