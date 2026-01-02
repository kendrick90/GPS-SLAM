#pragma once
// Gaussian Splatting renderer plugin
// Real-time differentiable Gaussian splatting

#include "sap/interfaces/renderer.h"

namespace sap::plugins {

class GaussianRenderer : public IRenderer {
public:
    // IPlugin interface
    bool initialize(const Config& config) override;
    void shutdown() override;
    std::string name() const override { return "gaussian_splatting"; }
    std::string version() const override { return "1.0.0"; }

    // IRenderer interface
    RenderOutput render(const SceneData& scene,
                        const torch::Tensor& pose,
                        const CameraIntrinsics& intrinsics) override;

    bool supportsDifferentiable() const override { return true; }
    RenderOutput renderWithGradients(const SceneData& scene,
                                      const torch::Tensor& pose,
                                      const CameraIntrinsics& intrinsics) override;

    // Gaussian-specific
    void setBackgroundColor(float r, float g, float b);
    void setTileSize(int size);
    void enableAntialiasing(bool enable);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sap::plugins
