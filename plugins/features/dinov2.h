#pragma once
// DINOv2 feature extractor plugin
// Provides dense visual features for matching and recognition

#include "sap/interfaces/feature_extractor.h"

namespace sap::plugins {

class DINOv2Extractor : public IFeatureExtractor {
public:
    // IPlugin interface
    bool initialize(const Config& config) override;
    void shutdown() override;
    std::string name() const override { return "dinov2"; }
    std::string version() const override { return "1.0.0"; }

    // IFeatureExtractor interface
    Features extract(const torch::Tensor& rgb) override;
    Matches match(const Features& features1, const Features& features2) override;

    // DINOv2-specific
    void setModelSize(const std::string& size);  // vits14, vitb14, vitl14, vitg14
    torch::Tensor getDenseFeatures(const torch::Tensor& rgb);
    torch::Tensor getPatchTokens(const torch::Tensor& rgb);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sap::plugins
