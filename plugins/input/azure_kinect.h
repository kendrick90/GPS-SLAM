#pragma once
// Azure Kinect input source plugin
// Provides RGB-D + IMU frames from Azure Kinect DK

#include "sap/interfaces/input_source.h"

namespace sap::plugins {

class AzureKinectInputSource : public IInputSource {
public:
    // IPlugin interface
    bool initialize(const Config& config) override;
    void shutdown() override;
    std::string name() const override { return "azure_kinect"; }
    std::string version() const override { return "1.0.0"; }

    // IInputSource interface
    bool hasMoreFrames() const override;
    Frame getNextFrame() override;
    CameraIntrinsics getIntrinsics() const override;
    bool hasDepth() const override { return true; }
    bool hasIMU() const override { return true; }

    // Azure Kinect-specific
    // Depth modes: 0=OFF, 1=NFOV_2X2BINNED, 2=NFOV_UNBINNED, 3=WFOV_2X2BINNED, 4=WFOV_UNBINNED
    bool setDepthMode(int mode);
    bool setColorResolution(int resolution);
    bool setFPS(int fps);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sap::plugins
