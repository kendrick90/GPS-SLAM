#pragma once
// Pangolin-based 3D viewer plugin
// Interactive 3D visualization with camera control

#include "sap/interfaces/viewer.h"

namespace sap::plugins {

class PangolinViewer : public IViewer {
public:
    // IPlugin interface
    bool initialize(const Config& config) override;
    void shutdown() override;
    std::string name() const override { return "pangolin_viewer"; }
    std::string version() const override { return "1.0.0"; }

    // IViewer interface
    void show(const Frame& frame) override;
    void show(const RenderOutput& render) override;
    void showSideBySide(const Frame& frame, const RenderOutput& render) override;
    bool shouldClose() const override;
    void processEvents() override;

    // 3D visualization
    void showPointCloud(const torch::Tensor& points, const torch::Tensor& colors);
    void showTrajectory(const std::vector<torch::Tensor>& poses);
    void showMesh(const SceneData& scene);
    void showGaussians(const SceneData& scene);

    // Camera control
    void setCameraFollow(bool follow);
    void setTopDownView(bool enable);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sap::plugins
