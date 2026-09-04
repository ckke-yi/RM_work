#include "config.hpp"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace {

std::string trim(std::string value) {
    const auto notSpace = [](unsigned char c) { return !std::isspace(c); };
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), notSpace));
    value.erase(std::find_if(value.rbegin(), value.rend(), notSpace).base(), value.end());
    return value;
}

}  // namespace

Config Config::load(const std::string& path) {
    std::ifstream input(path);
    if (!input) {
        throw std::runtime_error("cannot open config: " + path);
    }

    Config config;
    std::string line;
    int lineNumber = 0;
    while (std::getline(input, line)) {
        ++lineNumber;
        line = trim(line);
        if (line.empty() || line.front() == '#') {
            continue;
        }
        const auto separator = line.find('=');
        if (separator == std::string::npos) {
            throw std::runtime_error("invalid config line " + std::to_string(lineNumber));
        }
        const std::string key = trim(line.substr(0, separator));
        const std::string value = trim(line.substr(separator + 1));
        if (key.empty() || value.empty()) {
            throw std::runtime_error("empty config field on line " + std::to_string(lineNumber));
        }
        config.values_[key] = value;
    }
    return config;
}

std::string Config::getString(const std::string& key) const {
    const auto found = values_.find(key);
    if (found == values_.end()) {
        throw std::runtime_error("missing config key: " + key);
    }
    return found->second;
}

int Config::getInt(const std::string& key) const {
    return std::stoi(getString(key));
}

double Config::getDouble(const std::string& key) const {
    return std::stod(getString(key));
}
