


#pragma once

#include <string>
#include <memory>
#include <functional>
#include <unordered_map>
#include <vector>
#include <stdexcept>

namespace sap {

// Forward declarations
class Config;

// Plugin metadata
struct PluginInfo {
    std::string name;           // Unique plugin name
    std::string type;           // Plugin type: input, depth, segmentation, features, slam, rendering, viewer
    std::string version;        // Semantic version
    std::string description;    // Human-readable description
    std::string language;       // "cpp" or "python"
    std::vector<std::string> dependencies;  // Required plugins or libraries
};

// Base plugin interface - all plugins inherit from this
class IPlugin {
public:
    virtual ~IPlugin() = default;

    // Plugin metadata
    virtual const PluginInfo& info() const = 0;

    // Lifecycle management
    virtual bool initialize(const Config& config) = 0;
    virtual void shutdown() = 0;
    virtual bool isInitialized() const = 0;
};

// Plugin factory function type
using PluginFactory = std::function<std::unique_ptr<IPlugin>()>;

// Global plugin registry (singleton pattern)
class PluginRegistry {
public:
    // Get singleton instance
    static PluginRegistry& instance() {
        static PluginRegistry registry;
        return registry;
    }

    // Register a C++ plugin factory
    void registerPlugin(const std::string& type, const std::string& name, PluginFactory factory) {
        factories_[type][name] = std::move(factory);
    }

    // Register a Python plugin (module path)
    void registerPythonPlugin(const std::string& type, const std::string& name,
                              const std::string& module_path) {
        python_plugins_[type][name] = module_path;
    }

    // Check if plugin exists
    bool hasPlugin(const std::string& type, const std::string& name) const {
        auto type_it = factories_.find(type);
        if (type_it != factories_.end()) {
            if (type_it->second.find(name) != type_it->second.end()) {
                return true;
            }
        }
        auto py_it = python_plugins_.find(type);
        if (py_it != python_plugins_.end()) {
            if (py_it->second.find(name) != py_it->second.end()) {
                return true;
            }
        }
        return false;
    }

    // List all plugins of a type
    std::vector<std::string> listPlugins(const std::string& type) const {
        std::vector<std::string> result;
        auto type_it = factories_.find(type);
        if (type_it != factories_.end()) {
            for (const auto& [name, _] : type_it->second) {
                result.push_back(name);
            }
        }
        auto py_it = python_plugins_.find(type);
        if (py_it != python_plugins_.end()) {
            for (const auto& [name, _] : py_it->second) {
                result.push_back(name + " (python)");
            }
        }
        return result;
    }

    // List all plugin types
    std::vector<std::string> listTypes() const {
        std::vector<std::string> result;
        for (const auto& [type, _] : factories_) {
            result.push_back(type);
        }
        for (const auto& [type, _] : python_plugins_) {
            if (factories_.find(type) == factories_.end()) {
                result.push_back(type);
            }
        }
        return result;
    }

    // Create a plugin instance
    std::unique_ptr<IPlugin> create(const std::string& type, const std::string& name) const {
        auto type_it = factories_.find(type);
        if (type_it != factories_.end()) {
            auto plugin_it = type_it->second.find(name);
            if (plugin_it != type_it->second.end()) {
                return plugin_it->second();
            }
        }
        // Check Python plugins
        auto py_it = python_plugins_.find(type);
        if (py_it != python_plugins_.end()) {
            auto plugin_it = py_it->second.find(name);
            if (plugin_it != py_it->second.end()) {
                return createPythonPlugin(type, name, plugin_it->second);
            }
        }
        throw std::runtime_error("Plugin not found: " + type + "/" + name);
    }

    // Create plugin with type casting
    template<typename T>
    std::unique_ptr<T> createAs(const std::string& type, const std::string& name) const {
        auto plugin = create(type, name);
        auto* typed = dynamic_cast<T*>(plugin.get());
        if (!typed) {
            throw std::runtime_error("Plugin type mismatch: " + type + "/" + name);
        }
        plugin.release();
        return std::unique_ptr<T>(typed);
    }

    // Auto-discover plugins from directory
    void scanPluginDirectory(const std::string& path);

private:
    PluginRegistry() = default;
    PluginRegistry(const PluginRegistry&) = delete;
    PluginRegistry& operator=(const PluginRegistry&) = delete;

    // Create Python plugin via pybind11 bridge
    std::unique_ptr<IPlugin> createPythonPlugin(const std::string& type,
                                                 const std::string& name,
                                                 const std::string& module_path) const;

    // C++ plugin factories
    std::unordered_map<std::string, std::unordered_map<std::string, PluginFactory>> factories_;

    // Python plugin module paths
    std::unordered_map<std::string, std::unordered_map<std::string, std::string>> python_plugins_;
};

// Macro for automatic C++ plugin registration
// Usage: SAP_REGISTER_PLUGIN(input, webcam, WebcamSource)
#define SAP_REGISTER_PLUGIN(Type, Name, Class) \
    namespace { \
        static bool _sap_reg_##Type##_##Name = []() { \
            sap::PluginRegistry::instance().registerPlugin(#Type, #Name, \
                []() -> std::unique_ptr<sap::IPlugin> { \
                    return std::make_unique<Class>(); \
                }); \
            return true; \
        }(); \
    }

// Plugin type constants
namespace PluginTypes {
    constexpr const char* INPUT = "input";
    constexpr const char* DEPTH = "depth";
    constexpr const char* SEGMENTATION = "segmentation";
    constexpr const char* FEATURES = "features";
    constexpr const char* SLAM = "slam";
    constexpr const char* RENDERING = "rendering";
    constexpr const char* VIEWER = "viewer";
}

}  // namespace sap
