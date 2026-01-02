


#pragma once

#include <torch/torch.h>
#include <map>
#include <string>
#include <optional>
#include <memory>
#include <vector>

namespace sap {

// Tensor dictionary for flexible data passing
using TensorDict = std::map<std::string, torch::Tensor>;

// Camera intrinsic parameters
struct CameraIntrinsics {
    float fx = 0.0f, fy = 0.0f;  // Focal length
    float cx = 0.0f, cy = 0.0f;  // Principal point
    int width = 0, height = 0;
    std::string model = "pinhole";  // pinhole, fisheye, equirectangular

    // Distortion coefficients (for fisheye, etc.)
    std::vector<float> distortion;

    // Convert to 3x3 intrinsic matrix
    torch::Tensor toMatrix() const {
        auto K = torch::eye(3, torch::kFloat32);
        K[0][0] = fx;
        K[1][1] = fy;
        K[0][2] = cx;
        K[1][2] = cy;
        return K;
    }

    // Create from InfiniTAM-style parameters
    static CameraIntrinsics fromSimple(int w, int h, float fx_, float fy_, float cx_, float cy_) {
        CameraIntrinsics intr;
        intr.width = w;
        intr.height = h;
        intr.fx = fx_;
        intr.fy = fy_;
        intr.cx = cx_;
        intr.cy = cy_;
        return intr;
    }
};

// IMU sample data
struct IMUSample {
    float acc[3] = {0, 0, 0};     // Accelerometer (m/s^2)
    float gyro[3] = {0, 0, 0};    // Gyroscope (rad/s)
    int64_t timestamp_ns = 0;

    // Optional: orientation quaternion (w, x, y, z)
    std::optional<std::array<float, 4>> orientation;
};

// Core frame container - the primary data structure passed through the pipeline
struct Frame {
    // Identification
    int64_t timestamp_ns = 0;
    int frame_id = -1;

    // Core image data (all optional - depends on input source)
    std::optional<torch::Tensor> rgb;           // [H, W, 3] uint8 or [3, H, W] float
    std::optional<torch::Tensor> depth;         // [H, W] float32, in meters
    std::optional<torch::Tensor> confidence;    // [H, W] float32, depth confidence

    // Camera pose (camera-to-world transform)
    std::optional<torch::Tensor> c2w;           // [4, 4] ground truth pose
    std::optional<torch::Tensor> c2w_estimated; // [4, 4] SLAM-estimated pose

    // Camera parameters
    CameraIntrinsics intrinsics;

    // IMU data (if available)
    std::vector<IMUSample> imu_samples;

    // Extension data - populated by plugins (features, segmentation, etc.)
    TensorDict extensions;

    // Metadata
    std::string source_name;  // Name of input source that produced this frame
    bool is_keyframe = false;

    // Device management
    torch::Device device() const {
        if (rgb.has_value()) return rgb->device();
        if (depth.has_value()) return depth->device();
        return torch::kCPU;
    }

    // Move frame data to device
    Frame to(torch::Device dev) const {
        Frame f = *this;
        if (f.rgb.has_value()) f.rgb = f.rgb->to(dev);
        if (f.depth.has_value()) f.depth = f.depth->to(dev);
        if (f.confidence.has_value()) f.confidence = f.confidence->to(dev);
        if (f.c2w.has_value()) f.c2w = f.c2w->to(dev);
        if (f.c2w_estimated.has_value()) f.c2w_estimated = f.c2w_estimated->to(dev);
        for (auto& [key, tensor] : f.extensions) {
            f.extensions[key] = tensor.to(dev);
        }
        return f;
    }

    // Clone frame
    Frame clone() const {
        Frame f;
        f.timestamp_ns = timestamp_ns;
        f.frame_id = frame_id;
        if (rgb.has_value()) f.rgb = rgb->clone();
        if (depth.has_value()) f.depth = depth->clone();
        if (confidence.has_value()) f.confidence = confidence->clone();
        if (c2w.has_value()) f.c2w = c2w->clone();
        if (c2w_estimated.has_value()) f.c2w_estimated = c2w_estimated->clone();
        f.intrinsics = intrinsics;
        f.imu_samples = imu_samples;
        for (const auto& [key, tensor] : extensions) {
            f.extensions[key] = tensor.clone();
        }
        f.source_name = source_name;
        f.is_keyframe = is_keyframe;
        return f;
    }

    // Check if frame has valid RGB
    bool hasRGB() const { return rgb.has_value() && rgb->numel() > 0; }

    // Check if frame has valid depth
    bool hasDepth() const { return depth.has_value() && depth->numel() > 0; }

    // Check if frame has pose
    bool hasPose() const { return c2w.has_value() || c2w_estimated.has_value(); }

    // Get best available pose (estimated preferred over ground truth for SLAM)
    torch::Tensor getPose() const {
        if (c2w_estimated.has_value()) return c2w_estimated.value();
        if (c2w.has_value()) return c2w.value();
        return torch::eye(4, torch::kFloat32);
    }
};

// Batch of frames for temporal processing
struct FrameBatch {
    std::vector<Frame> frames;

    int64_t size() const { return static_cast<int64_t>(frames.size()); }
    bool empty() const { return frames.empty(); }

    Frame& operator[](size_t idx) { return frames[idx]; }
    const Frame& operator[](size_t idx) const { return frames[idx]; }

    void push_back(Frame&& frame) { frames.push_back(std::move(frame)); }
    void push_back(const Frame& frame) { frames.push_back(frame); }

    // Move all frames to device
    FrameBatch to(torch::Device dev) const {
        FrameBatch batch;
        for (const auto& f : frames) {
            batch.frames.push_back(f.to(dev));
        }
        return batch;
    }
};

}  // namespace sap
