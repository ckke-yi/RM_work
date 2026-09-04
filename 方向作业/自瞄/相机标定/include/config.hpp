#pragma once

#include <string>
#include <unordered_map>

class Config {
public:
    static Config load(const std::string& path);

    std::string getString(const std::string& key) const;
    int getInt(const std::string& key) const;
    double getDouble(const std::string& key) const;

private:
    std::unordered_map<std::string, std::string> values_;
};
