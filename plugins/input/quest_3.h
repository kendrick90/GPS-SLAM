#pragma once
// Meta Quest 3 Input Source Plugin
// Provides stereo RGB, depth, IMU, hand tracking, and eye tracking from Quest 3
// Uses Meta's Passthrough Camera API (Android Camera2 NDK) and Depth API

#include "sap/interfaces/input_source.h"
#include <memory>
#include <functional>
#include <array>

namespace sap::plugins {

// Quest 3 camera selection
enum class Quest3Camera {
    LEFT = 0,           // Left passthrough camera
    RIGHT = 1,          // Right passthrough camera
    STEREO = 2,         // Both cameras (synchronized)
    CENTER_VIRTUAL = 3  // Virtual center camera (software-rectified)
};

// Quest 3 depth mode
enum class Quest3DepthMode {
    OFF = 0,
    ENVIRONMENTAL = 1,  // Environmental depth from stereo + ML
    HAND_OCCLUSION = 2, // Optimized for hand occlusion
    FULL = 3            // Full depth with highest quality
};

// Quest 3 camera resolution
enum class Quest3Resolution {
    RES_960P = 0,   // 1280x960 (original)
    RES_1280 = 1,   // 1280x1280 (HzOS v83+)
    RES_720P = 2,   // 1280x720 (cropped for 16:9)
    RES_NATIVE = 3  // Native sensor resolution
};

// Hand tracking keypoint (21 joints per hand)
struct HandKeypoints {
    std::array<std::array<float, 3>, 21> positions;  // World space positions
    std::array<std::array<float, 4>, 21> rotations;  // Quaternions (w,x,y,z)
    std::array<float, 21> confidences;               // Per-joint confidence
    float hand_confidence = 0.0f;                    // Overall hand confidence
    bool is_tracked = false;
};

// Eye tracking data
struct EyeTrackingData {
    std::array<float, 3> left_gaze;      // Left eye gaze direction
    std::array<float, 3> right_gaze;     // Right eye gaze direction
    std::array<float, 3> combined_gaze;  // Combined gaze direction
    float convergence_distance = 0.0f;   // Distance where eyes converge
    float left_openness = 1.0f;          // 0=closed, 1=open
    float right_openness = 1.0f;
    bool is_valid = false;
};

// Quest 3 configuration
struct Quest3Config {
    // Camera settings
    Quest3Camera camera_mode = Quest3Camera::STEREO;
    Quest3Resolution resolution = Quest3Resolution::RES_1280;
    int target_fps = 30;  // 30, 60, or 72

    // Depth settings
    Quest3DepthMode depth_mode = Quest3DepthMode::ENVIRONMENTAL;
    bool depth_aligned_to_color = true;

    // Additional sensors
    bool enable_imu = true;
    bool enable_hand_tracking = true;
    bool enable_eye_tracking = false;  // Requires user permission

    // Connection settings (for PC streaming)
    enum class ConnectionMode {
        ON_DEVICE,      // Running on Quest 3 directly
        USB_LINK,       // Quest Link over USB-C
        AIR_LINK,       // Quest Air Link (WiFi 6E)
        CUSTOM_STREAM   // Custom streaming protocol
    };
    ConnectionMode connection = ConnectionMode::ON_DEVICE;

    // For streaming modes
    std::string host_address = "";
    int port = 8888;

    // Auto-exposure
    bool auto_exposure = true;
    float manual_exposure_ms = 8.0f;  // If auto_exposure is false

    // Timestamp synchronization
    bool sync_timestamps = true;
    int64_t timestamp_offset_ns = 0;
};

// Callback for async frame delivery (for on-device use)
using Quest3FrameCallback = std::function<void(const Frame&)>;

class Quest3InputSource : public IInputSource {
public:
    Quest3InputSource();
    ~Quest3InputSource() override;

    // IPlugin interface
    bool initialize(const Config& config) override;
    void shutdown() override;
    std::string name() const override { return "quest_3"; }
    std::string version() const override { return "1.0.0"; }

    // IInputSource interface
    bool hasMoreFrames() const override;
    bool hasFramesNow() const override;  // Non-blocking check
    Frame getNextFrame() override;
    CameraIntrinsics getIntrinsics() const override;

    // Sensor capabilities
    bool hasDepth() const override;
    bool hasIMU() const override { return config_.enable_imu; }
    bool hasStereo() const override { return config_.camera_mode == Quest3Camera::STEREO; }

    // Live control
    bool startCapture() override;
    void stopCapture() override;
    float getExposure() const override;
    bool setExposure(float ms) override;

    // ==================== Quest 3 Specific ====================

    // Configuration
    bool setConfig(const Quest3Config& config);
    Quest3Config getConfig() const { return config_; }

    // Camera mode
    bool setCameraMode(Quest3Camera mode);
    bool setResolution(Quest3Resolution res);
    bool setTargetFPS(int fps);

    // Depth settings
    bool setDepthMode(Quest3DepthMode mode);
    float getDepthScale() const override { return 0.001f; }  // mm to meters

    // Stereo access (when in stereo mode)
    CameraIntrinsics getLeftIntrinsics() const;
    CameraIntrinsics getRightIntrinsics() const;
    torch::Tensor getStereoBaseline() const;  // [4,4] transform from left to right

    // Hand tracking
    bool enableHandTracking(bool enable);
    bool isHandTrackingEnabled() const { return config_.enable_hand_tracking; }
    std::pair<HandKeypoints, HandKeypoints> getLatestHandData() const;

    // Eye tracking
    bool enableEyeTracking(bool enable);
    bool isEyeTrackingEnabled() const { return config_.enable_eye_tracking; }
    EyeTrackingData getLatestEyeData() const;

    // Quest tracking (6DoF headset pose from Quest's SLAM)
    bool hasQuestTracking() const { return true; }
    torch::Tensor getQuestPose() const;  // [4,4] headset pose in Quest's world frame
    torch::Tensor getQuestVelocity() const;  // [6] linear + angular velocity

    // Anchor/spatial anchor support
    bool createSpatialAnchor(const torch::Tensor& pose, const std::string& id);
    bool getSpatialAnchorPose(const std::string& id, torch::Tensor& pose) const;
    std::vector<std::string> listSpatialAnchors() const;

    // ==================== Streaming Support ====================

    // For PC-based processing: receive frames over network
    bool connectToQuest(const std::string& host, int port);
    bool isConnected() const;

    // For on-device: set callback for async frame delivery
    void setFrameCallback(Quest3FrameCallback callback);

    // Streaming stats
    struct StreamingStats {
        float fps = 0.0f;
        float latency_ms = 0.0f;
        int64_t frames_received = 0;
        int64_t frames_dropped = 0;
        float bandwidth_mbps = 0.0f;
    };
    StreamingStats getStreamingStats() const;

    // ==================== Calibration ====================

    // Load custom calibration (overrides factory calibration)
    bool loadCalibration(const std::string& path) override;

    // Get factory stereo calibration
    struct StereoCalibration {
        CameraIntrinsics left;
        CameraIntrinsics right;
        torch::Tensor R;  // [3,3] rotation from left to right
        torch::Tensor T;  // [3] translation from left to right
        torch::Tensor E;  // [3,3] essential matrix
        torch::Tensor F;  // [3,3] fundamental matrix
    };
    StereoCalibration getStereoCalibration() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
    Quest3Config config_;
    bool is_capturing_ = false;
    int64_t frame_count_ = 0;

    // Frame building helpers
    Frame buildFrame(/* internal camera data */);
    void populateIMUData(Frame& frame);
    void populateHandData(Frame& frame);
    void populateEyeData(Frame& frame);
};

// ==================== Quest 3 Frame Extensions ====================
// When Quest3InputSource populates a Frame, it adds these to frame.extensions:
//
// Stereo mode:
//   "left_rgb"  : [H, W, 3] uint8 - left camera image
//   "right_rgb" : [H, W, 3] uint8 - right camera image
//
// Hand tracking:
//   "hand_left_positions"  : [21, 3] float32 - left hand joint positions
//   "hand_left_rotations"  : [21, 4] float32 - left hand joint rotations
//   "hand_left_confidence" : [21] float32 - per-joint confidence
//   "hand_right_positions" : [21, 3] float32
//   "hand_right_rotations" : [21, 4] float32
//   "hand_right_confidence": [21] float32
//
// Eye tracking:
//   "eye_gaze_left"       : [3] float32 - left eye gaze direction
//   "eye_gaze_right"      : [3] float32 - right eye gaze direction
//   "eye_gaze_combined"   : [3] float32 - combined gaze direction
//   "eye_convergence"     : [1] float32 - convergence distance
//   "eye_openness"        : [2] float32 - left, right openness
//
// Quest tracking:
//   "quest_pose"          : [4, 4] float32 - Quest's SLAM pose
//   "quest_velocity"      : [6] float32 - linear + angular velocity

}  // namespace sap::plugins
