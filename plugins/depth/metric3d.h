#pragma once
// Metric3D depth estimator plugin
// Uses Metric3D V2 for metric-scale monocular depth estimation

#include "sap/interfaces/depth_estimator.h"

namespace sap::plugins {

class Metric3DEstimator : public IDepthEstimator {
public:
    // IPlugin interface
    bool initialize(const Config& config) override;
    void shutdown() override;
    std::string name() const override { return "metric3d_v2"; }
    std::string version() const override { return "1.0.0"; }

    // IDepthEstimator interface
    torch::Tensor estimate(const torch::Tensor& rgb) override;
    torch::Tensor estimateBatch(const torch::Tensor& rgb_batch) override;
    bool isMetric() const override { return true; }  // Metric depth!
    float getMaxDepth() const override { return 80.0f; }

    // Set camera intrinsics for better metric estimation
    void setIntrinsics(const CameraIntrinsics& intrinsics);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sap::plugins
