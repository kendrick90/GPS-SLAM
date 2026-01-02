#pragma once

#include "camera.h"
#include <opencv2/videoio.hpp>

namespace sap {

class WebcamCamera : public Camera {
public:
    WebcamCamera() = default;
    ~WebcamCamera() override { close(); }

    bool open(const std::string& config = "") override {
        // Config format: "device_id" or "device_id:width:height"
        int device_id = 0;
        int width = 1280;
        int height = 720;

        if (!config.empty()) {
            // Parse config string
            size_t pos1 = config.find(':');
            if (pos1 != std::string::npos) {
                device_id = std::stoi(config.substr(0, pos1));
                size_t pos2 = config.find(':', pos1 + 1);
                if (pos2 != std::string::npos) {
                    width = std::stoi(config.substr(pos1 + 1, pos2 - pos1 - 1));
                    height = std::stoi(config.substr(pos2 + 1));
                }
            } else {
                device_id = std::stoi(config);
            }
        }

        cap_.open(device_id, cv::CAP_ANY);
        if (!cap_.isOpened()) {
            return false;
        }

        // Set resolution
        cap_.set(cv::CAP_PROP_FRAME_WIDTH, width);
        cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height);

        // Get actual resolution
        width_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
        height_ = static_cast<int>(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));

        // Estimate focal length (typical webcam FOV ~60 degrees)
        float fov_rad = 60.0f * 3.14159f / 180.0f;
        float focal = width_ / (2.0f * std::tan(fov_rad / 2.0f));

        intrinsics_ = CameraIntrinsics{
            focal, focal,
            width_ / 2.0f, height_ / 2.0f,
            width_, height_
        };

        return true;
    }

    void close() override {
        if (cap_.isOpened()) {
            cap_.release();
        }
    }

    bool isOpen() const override { return cap_.isOpened(); }

    FrameData getFrame() override {
        FrameData frame;
        frame.frame_id = frame_count_++;
        frame.intrinsics = intrinsics_;

        cv::Mat raw;
        if (cap_.read(raw)) {
            frame.rgb = raw;
            frame.timestamp_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::steady_clock::now().time_since_epoch()
            ).count();
        }

        return frame;
    }

    CameraIntrinsics getIntrinsics() const override { return intrinsics_; }
    std::string name() const override { return "webcam"; }

private:
    cv::VideoCapture cap_;
    CameraIntrinsics intrinsics_;
    int width_ = 0;
    int height_ = 0;
    int frame_count_ = 0;
};

}  // namespace sap
