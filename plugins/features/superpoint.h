#pragma once
// SuperPoint + LightGlue feature extractor plugin
// Provides keypoint detection and matching

#include "sap/interfaces/feature_extractor.h"

namespace sap::plugins {

class SuperPointExtractor : public IFeatureExtractor {
public:
    // IPlugin interface
    bool initialize(const Config& config) override;
    void shutdown() override;
    std::string name() const override { return "superpoint"; }
    std::string version() const override { return "1.0.0"; }

    // IFeatureExtractor interface
    Features extract(const torch::Tensor& rgb) override;
    Matches match(const Features& features1, const Features& features2) override;

    // SuperPoint-specific
    void setMaxKeypoints(int max_keypoints);
    void setDetectionThreshold(float threshold);
    void enableLightGlue(bool enable);  // Use LightGlue for matching

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sap::plugins
