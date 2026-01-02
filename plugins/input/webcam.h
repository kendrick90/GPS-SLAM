#pragma once
// Webcam input source plugin
// Wraps OpenCV VideoCapture for standard webcam access

#include "sap/interfaces/input_source.h"
#include <opencv2/videoio.hpp>

namespace sap::plugins {

class WebcamInputSource : public IInputSource {
public:
    // IPlugin interface
    bool initialize(const Config& config) override;
    void shutdown() override;
    std::string name() const override { return "webcam"; }
    std::string version() const override { return "1.0.0"; }

    // IInputSource interface
    bool hasMoreFrames() const override;
    Frame getNextFrame() override;
    CameraIntrinsics getIntrinsics() const override;
    bool hasDepth() const override { return false; }

private:
    cv::VideoCapture cap_;
    CameraIntrinsics intrinsics_;
    int64_t frame_id_ = 0;
};

}  // namespace sap::plugins
