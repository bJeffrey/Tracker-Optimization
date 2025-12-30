#pragma once
#include "plugin_framework/plugin_interfaces.h"
#include "trk/track_meta.h"  // Your existing track types
#include "index/scan_volume.h"

namespace tracker::plugins {

class ISpatialIndex : public IPlugin {
public:
    virtual void insert(const Track& track) = 0;
    virtual void remove(TrackID id) = 0;
    virtual void update(const Track& track) = 0;
    
    // Query tracks in scan volume
    virtual std::vector<TrackID> query(const ScanVolume& volume) = 0;
    
    // Bulk operations for efficiency
    virtual void insertBatch(const std::vector<Track>& tracks) = 0;
    virtual std::vector<Track> queryBatch(const ScanVolume& volume) = 0;
};

} // namespace tracker::plugins