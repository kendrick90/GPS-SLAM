#pragma once
// Gaussian Splatting mapper plugin
// 3D Gaussian representation for high-quality rendering

#include "sap/interfaces/slam_backend.h"

namespace sap::plugins {

class GaussianMapper : public IMapper {
public:
    // IPlugin interface
    bool initialize(const Config& config) override;
    void shutdown() override;
    std::string name() const override { return "gaussian_mapper"; }
    std::string version() const override { return "1.0.0"; }

    // IMapper interface
    void integrate(const Frame& frame, const torch::Tensor& pose) override;
    SceneData getScene() const override;
    SceneRepresentation representationType() const override {
        return SceneRepresentation::GAUSSIAN;
    }
    void reset() override;
    TensorDict raycast(const torch::Tensor& pose,
                       const CameraIntrinsics& intrinsics) const override;
    void exportMesh(const std::string& path) const override;
    void exportPointCloud(const std::string& path) const override;
    int64_t pointCount() const override;
    std::array<float, 6> boundingBox() const override;

    // Gaussian-specific
    void setLearningRate(float lr);
    void setDensificationInterval(int interval);
    void setOpacityResetInterval(int interval);
    void optimize(int iterations);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sap::plugins
