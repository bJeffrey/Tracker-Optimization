#pragma once
#include "plugin_framework/plugin_interfaces.h"
#include "la/la.h"  // Your existing LA types

namespace tracker::plugins {

class IPropagator : public IPlugin {
public:
    // Propagate single track
    virtual void propagate(
        VectorXd& state, 
        MatrixXd& cov, 
        double dt) = 0;
    
    // Batch propagation (what you already have)
    virtual void propagateBatch(
        std::vector<VectorXd>& states,
        std::vector<MatrixXd>& covs,
        const std::vector<double>& dts) = 0;
    
    // Set process noise
    virtual void setProcessNoise(const MatrixXd& Q) = 0;
};

} // namespace tracker::plugins