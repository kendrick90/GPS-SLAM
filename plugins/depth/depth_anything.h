#pragma once
// Depth Anything V2 depth estimator plugin
// Uses the Depth Anything model for monocular depth estimation

#include "sap/interfaces/depth_estimator.h"

namespace sap::plugins {

class DepthAnythingEstimator : public IDepthEstimator {
public:
    // IPlugin interface
    bool initialize(const Config& config) override;
    void shutdown() override;
    std::string name() const override { return "depth_anything_v2"; }
    std::string version() const override { return "1.0.0"; }

    // IDepthEstimator interface
    torch::Tensor estimate(const torch::Tensor& rgb) override;
    torch::Tensor estimateBatch(const torch::Tensor& rgb_batch) override;
    bool isMetric() const override { return false; }  // Relative depth
    float getMaxDepth() const override { return 10.0f; }

    // Model variants: vits, vitb, vitl
    void setModelSize(const std::string& size);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sap::plugins
