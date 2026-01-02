#pragma once
// Mesh rasterization renderer plugin
// Standard triangle rasterization via OpenGL/CUDA

#include "sap/interfaces/renderer.h"

namespace sap::plugins {

class MeshRenderer : public IRenderer {
public:
    // IPlugin interface
    bool initialize(const Config& config) override;
    void shutdown() override;
    std::string name() const override { return "mesh_renderer"; }
    std::string version() const override { return "1.0.0"; }

    // IRenderer interface
    RenderOutput render(const SceneData& scene,
                        const torch::Tensor& pose,
                        const CameraIntrinsics& intrinsics) override;

    bool supportsDifferentiable() const override { return false; }

    // Mesh-specific
    void setShading(const std::string& mode);  // flat, smooth, pbr
    void setWireframe(bool enable);
    void setLighting(const torch::Tensor& light_dir);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sap::plugins
