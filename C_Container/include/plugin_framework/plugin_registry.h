#pragma once
#include <memory>
#include <unordered_map>
#include <string>
#include "plugin_interfaces.h"

namespace tracker {

class PluginRegistry {
public:
    static PluginRegistry& instance();
    
    // Register a plugin instance
    template<typename T>
    void registerPlugin(const std::string& name, std::unique_ptr<T> plugin) {
        plugins_[name] = std::move(plugin);
    }
    
    // Get plugin by name and type
    template<typename T>
    T* getPlugin(const std::string& name) {
        auto it = plugins_.find(name);
        if (it != plugins_.end()) {
            return dynamic_cast<T*>(it->second.get());
        }
        return nullptr;
    }
    
    // Initialize all plugins
    bool initializeAll();
    void shutdownAll();
    
private:
    PluginRegistry() = default;
    std::unordered_map<std::string, std::unique_ptr<IPlugin>> plugins_;
};

} // namespace tracker