#pragma once
// Segment Anything (SAM/SAM2) segmentation plugin
// Provides interactive and automatic segmentation

#include "sap/interfaces/segmentor.h"

namespace sap::plugins {

class SAMSegmentor : public ISegmentor {
public:
    // IPlugin interface
    bool initialize(const Config& config) override;
    void shutdown() override;
    std::string name() const override { return "sam"; }
    std::string version() const override { return "1.0.0"; }

    // ISegmentor interface
    torch::Tensor segment(const torch::Tensor& rgb) override;
    torch::Tensor segmentWithPrompt(const torch::Tensor& rgb,
                                     const torch::Tensor& points,
                                     const torch::Tensor& labels) override;
    torch::Tensor segmentWithBox(const torch::Tensor& rgb,
                                  const torch::Tensor& box) override;

    // SAM-specific
    void setModelSize(const std::string& size);  // vit_h, vit_l, vit_b
    void enableSAM2(bool enable);  // Use SAM2 for video
    torch::Tensor getImageEmbedding(const torch::Tensor& rgb);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sap::plugins
