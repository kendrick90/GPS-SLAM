#pragma once
// OpenCV-based 2D viewer plugin
// Simple image display for debugging and visualization

#include "sap/interfaces/viewer.h"

namespace sap::plugins {

class OpenCVViewer : public IViewer {
public:
    // IPlugin interface
    bool initialize(const Config& config) override;
    void shutdown() override;
    std::string name() const override { return "opencv_viewer"; }
    std::string version() const override { return "1.0.0"; }

    // IViewer interface
    void show(const Frame& frame) override;
    void show(const RenderOutput& render) override;
    void showSideBySide(const Frame& frame, const RenderOutput& render) override;
    bool shouldClose() const override;
    void processEvents() override;

    // OpenCV-specific
    void setWindowName(const std::string& name);
    void setScale(float scale);
    void enableDepthColormap(bool enable);

private:
    std::string window_name_ = "Spatial AI Playground";
    float scale_ = 1.0f;
    bool should_close_ = false;
};

}  // namespace sap::plugins
