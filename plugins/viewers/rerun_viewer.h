#pragma once
// Rerun.io Viewer Plugin
// All-in-one SLAM visualization and data logging using rerun.io SDK
// Provides live streaming of frames, poses, point clouds, and Gaussian splats

#include "sap/interfaces/viewer.h"
#include "sap/interfaces/slam_backend.h"
#include <memory>
#include <string>
#include <vector>

namespace sap::plugins {

// Rerun spatial entity paths for organized logging
struct RerunPaths {
    static constexpr const char* WORLD = "world";
    static constexpr const char* CAMERA = "world/camera";
    static constexpr const char* CAMERA_RGB = "world/camera/rgb";
    static constexpr const char* CAMERA_DEPTH = "world/camera/depth";
    static constexpr const char* TRAJECTORY = "world/trajectory";
    static constexpr const char* POINT_CLOUD = "world/map/points";
    static constexpr const char* GAUSSIANS = "world/map/gaussians";
    static constexpr const char* MESH = "world/map/mesh";
    static constexpr const char* IMU = "sensors/imu";
    static constexpr const char* HANDS = "sensors/hands";
    static constexpr const char* EYES = "sensors/eye_tracking";
};

// Configuration for Rerun viewer
struct RerunConfig {
    std::string application_id = "spatial-ai-playground";
    std::string recording_id = "";  // Empty = auto-generate

    // Connection options
    bool spawn_viewer = true;       // Spawn local viewer process
    std::string connect_addr = "";  // Connect to remote viewer (e.g., "127.0.0.1:9876")
    std::string save_path = "";     // Save to .rrd file

    // Logging options
    bool log_rgb = true;
    bool log_depth = true;
    bool log_poses = true;
    bool log_point_cloud = true;
    bool log_gaussians = true;
    bool log_imu = true;
    bool log_hands = false;         // Quest 3 hand tracking
    bool log_eye_gaze = false;      // Quest 3 eye tracking

    // Downsampling for performance
    int rgb_downsample = 1;         // 1 = full res, 2 = half, etc.
    int depth_downsample = 2;
    int point_cloud_stride = 4;     // Every Nth point

    // Timeline configuration
    std::string time_name = "frame";
    bool use_wall_clock = false;    // Also log wall clock time
};

// Viewer that streams all SLAM data to Rerun.io
class RerunViewer : public IViewer {
public:
    RerunViewer();
    ~RerunViewer() override;

    // IPlugin interface
    bool initialize(const Config& config) override;
    void shutdown() override;
    std::string name() const override { return "rerun"; }
    std::string version() const override { return "1.0.0"; }

    // IViewer interface
    void showFrame(const Frame& frame) override;
    void showRender(const RenderResult& render) override;
    void showComparison(const Frame& input, const RenderResult& render) override;
    void showImage(const std::string& window_name, const torch::Tensor& image) override;
    void showDepth(const std::string& window_name, const torch::Tensor& depth,
                   float min_depth = 0.0f, float max_depth = 10.0f) override;

    // 3D visualization
    void showScene(const SceneData& scene) override;
    void showTrajectory(const std::vector<torch::Tensor>& poses) override;
    void addCameraFrustum(const torch::Tensor& pose,
                          const CameraIntrinsics& intrinsics,
                          float scale = 0.1f) override;

    // Window management (Rerun handles its own window)
    bool shouldClose() const override { return false; }
    void processEvents() override {}
    int waitKey(int timeout_ms = 0) override { return -1; }

    // Network/remote
    bool isRemote() const override { return !config_.connect_addr.empty(); }
    void setRemoteAddress(const std::string& host, int port) override;
    bool isConnected() const override;

    // ==================== SLAM-Specific Logging ====================

    // Log a complete SLAM frame with all available data
    void logSLAMFrame(const Frame& frame, const torch::Tensor& pose, int64_t frame_id);

    // Log scene representations
    void logPointCloud(const torch::Tensor& points, const torch::Tensor& colors,
                       const std::string& entity_path = RerunPaths::POINT_CLOUD);
    void logGaussians(const SceneData& gaussians);
    void logMesh(const torch::Tensor& vertices, const torch::Tensor& faces,
                 const torch::Tensor& colors = {});

    // Log tracking results
    void logPose(const torch::Tensor& c2w, int64_t frame_id);
    void logTrackingStatus(const std::string& status, float confidence);

    // Log sensor data
    void logIMU(const std::vector<IMUSample>& samples);
    void logHandTracking(const torch::Tensor& left_keypoints,
                         const torch::Tensor& right_keypoints);
    void logEyeGaze(const torch::Tensor& gaze_direction,
                    float convergence_distance);

    // Log depth estimation results
    void logDepthEstimation(const torch::Tensor& mono_depth,
                            const torch::Tensor& confidence,
                            const std::string& method);

    // Log optimization/bundle adjustment
    void logOptimizationGraph(const std::vector<torch::Tensor>& poses,
                              const std::vector<std::pair<int, int>>& edges);
    void logLoopClosure(int frame_i, int frame_j, const torch::Tensor& relative_pose);

    // ==================== Recording Control ====================

    bool startRecording(const std::string& path, int fps = 30) override;
    void stopRecording() override;
    bool isRecording() const override;

    // Flush pending data
    void flush();

    // ==================== Timeline Control ====================

    void setTimeSequence(const std::string& name, int64_t value);
    void setTimeSeconds(const std::string& name, double seconds);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
    RerunConfig config_;
    bool connected_ = false;
    int64_t current_frame_ = 0;
};

}  // namespace sap::plugins
