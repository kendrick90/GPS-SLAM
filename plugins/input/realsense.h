#pragma once
// Intel RealSense input source plugin
// Provides RGB-D frames from RealSense cameras

#include "sap/interfaces/input_source.h"

namespace sap::plugins {

class RealSenseInputSource : public IInputSource {
public:
    // IPlugin interface
    bool initialize(const Config& config) override;
    void shutdown() override;
    std::string name() const override { return "realsense"; }
    std::string version() const override { return "1.0.0"; }

    // IInputSource interface
    bool hasMoreFrames() const override;
    Frame getNextFrame() override;
    CameraIntrinsics getIntrinsics() const override;
    bool hasDepth() const override { return true; }
    bool hasIMU() const override;

    // RealSense-specific
    bool setDepthMode(int mode);
    bool setColorResolution(int width, int height);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sap::plugins
