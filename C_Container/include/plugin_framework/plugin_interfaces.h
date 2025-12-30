#pragma once
#include <string>
#include <memory>

namespace tracker {

// Base interface all plugins inherit from
class IPlugin {
public:
    virtual ~IPlugin() = default;
    virtual std::string getName() const = 0;
    virtual std::string getVersion() const = 0;
    virtual bool initialize() = 0;
    virtual void shutdown() = 0;
};

} // namespace tracker