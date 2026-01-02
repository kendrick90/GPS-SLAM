


#pragma once

#include <yaml-cpp/yaml.h>
#include <string>
#include <map>
#include <vector>
#include <stdexcept>
#include <optional>

namespace sap {

// Configuration wrapper around YAML::Node with convenience methods
class Config {
public:
    Config() = default;
    Config(const YAML::Node& node) : node_(node) {}

    // Load from file
    static Config loadFile(const std::string& path) {
        return Config(YAML::LoadFile(path));
    }

    // Load from string
    static Config loadString(const std::string& yaml_str) {
        return Config(YAML::Load(yaml_str));
    }

    // Check if key exists
    bool has(const std::string& key) const {
        return node_[key].IsDefined();
    }

    // Get value with type
    template<typename T>
    T get(const std::string& key) const {
        if (!node_[key].IsDefined()) {
            throw std::runtime_error("Config key not found: " + key);
        }
        return node_[key].as<T>();
    }

    // Get value with default
    template<typename T>
    T get(const std::string& key, const T& default_value) const {
        if (!node_[key].IsDefined()) {
            return default_value;
        }
        return node_[key].as<T>();
    }

    // Get optional value
    template<typename T>
    std::optional<T> getOptional(const std::string& key) const {
        if (!node_[key].IsDefined()) {
            return std::nullopt;
        }
        return node_[key].as<T>();
    }

    // Get nested config
    Config operator[](const std::string& key) const {
        return Config(node_[key]);
    }

    // Get as vector
    template<typename T>
    std::vector<T> getVector(const std::string& key) const {
        std::vector<T> result;
        if (node_[key].IsSequence()) {
            for (const auto& item : node_[key]) {
                result.push_back(item.as<T>());
            }
        }
        return result;
    }

    // Get as map
    template<typename T>
    std::map<std::string, T> getMap(const std::string& key) const {
        std::map<std::string, T> result;
        if (node_[key].IsMap()) {
            for (const auto& item : node_[key]) {
                result[item.first.as<std::string>()] = item.second.as<T>();
            }
        }
        return result;
    }

    // Set value
    template<typename T>
    void set(const std::string& key, const T& value) {
        node_[key] = value;
    }

    // Merge with another config (other takes precedence)
    void merge(const Config& other) {
        mergeNodes(node_, other.node_);
    }

    // Get raw YAML node
    const YAML::Node& node() const { return node_; }
    YAML::Node& node() { return node_; }

    // Check if valid/non-empty
    bool isValid() const { return node_.IsDefined() && !node_.IsNull(); }
    operator bool() const { return isValid(); }

    // Iteration support
    YAML::const_iterator begin() const { return node_.begin(); }
    YAML::const_iterator end() const { return node_.end(); }

private:
    YAML::Node node_;

    static void mergeNodes(YAML::Node& target, const YAML::Node& source) {
        if (!source.IsMap()) {
            target = source;
            return;
        }
        for (const auto& item : source) {
            const std::string key = item.first.as<std::string>();
            if (target[key].IsMap() && item.second.IsMap()) {
                mergeNodes(target[key], item.second);
            } else {
                target[key] = item.second;
            }
        }
    }
};

}  // namespace sap
