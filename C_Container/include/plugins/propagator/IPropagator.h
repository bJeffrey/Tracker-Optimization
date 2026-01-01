#pragma once
#include "plugin_framework/plugin_interfaces.h"
#include "plugins/propagator/la.h"  // Backend-agnostic matrix views

#include <vector>

namespace trk::plugins {

// Backend-agnostic views (row-major). Treat vectors as n x 1 MatrixView.
using VectorXd = la::MatrixView;
using MatrixXd = la::MatrixView;

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

} // namespace trk::plugins
