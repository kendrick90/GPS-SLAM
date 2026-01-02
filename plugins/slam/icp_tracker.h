#pragma once
// ICP (Iterative Closest Point) tracker plugin
// Depth-based camera tracking similar to InfiniTAM

#include "sap/interfaces/slam_backend.h"

namespace sap::plugins {

class ICPTracker : public ITracker {
public:
    // IPlugin interface
    bool initialize(const Config& config) override;
    void shutdown() override;
    std::string name() const override { return "icp_tracker"; }
    std::string version() const override { return "1.0.0"; }

    // ITracker interface
    TrackingResult track(const Frame& frame) override;
    TrackingResult track(const Frame& frame, const SceneData& scene) override;
    void reset() override;
    void setPose(const torch::Tensor& pose) override;
    torch::Tensor getLastPose() const override;
    bool requiresDepth() const override { return true; }

    // ICP-specific
    void setIterations(int iterations_per_level);
    void setLevels(int pyramid_levels);
    void setDistanceThreshold(float threshold);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sap::plugins
