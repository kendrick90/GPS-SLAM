#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <optional>
#include <memory>
#include <string>

namespace sap {

// Camera intrinsics
struct CameraIntrinsics {
    float fx, fy;       // Focal lengths
    float cx, cy;       // Principal point
    int width, height;  // Image dimensions

    Eigen::Matrix3f K() const {
        Eigen::Matrix3f k = Eigen::Matrix3f::Identity();
        k(0, 0) = fx; k(1, 1) = fy;
        k(0, 2) = cx; k(1, 2) = cy;
        return k;
    }
};

// IMU sample
struct IMUSample {
    double timestamp;
    Eigen::Vector3f accel;      // m/s^2
    Eigen::Vector3f gyro;       // rad/s
    std::optional<Eigen::Quaternionf> orientation;  // From sensor fusion
};

// Frame data returned by camera
struct FrameData {
    int64_t timestamp_ns = 0;
    int frame_id = -1;

    cv::Mat rgb;                // BGR format from OpenCV
    std::optional<cv::Mat> depth;  // Depth in meters (float)

    CameraIntrinsics intrinsics;
    std::optional<CameraIntrinsics> depth_intrinsics;

    std::vector<IMUSample> imu_samples;

    bool valid() const { return !rgb.empty(); }
};

// Abstract camera base class
class Camera {
public:
    virtual ~Camera() = default;

    // Initialize camera with optional config
    virtual bool open(const std::string& config = "") = 0;

    // Close camera and release resources
    virtual void close() = 0;

    // Check if camera is open
    virtual bool isOpen() const = 0;

    // Get next frame (blocking)
    virtual FrameData getFrame() = 0;

    // Try to get frame without blocking
    virtual std::optional<FrameData> tryGetFrame() {
        return getFrame();  // Default: blocking
    }

    // Camera properties
    virtual CameraIntrinsics getIntrinsics() const = 0;
    virtual bool hasDepth() const { return false; }
    virtual bool hasIMU() const { return false; }

    // Camera name/type
    virtual std::string name() const = 0;

    // Factory method
    static std::unique_ptr<Camera> create(const std::string& type);
};

}  // namespace sap
