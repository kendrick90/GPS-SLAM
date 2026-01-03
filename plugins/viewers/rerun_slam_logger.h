#pragma once
// Rerun SLAM Logger
// Convenience wrapper for logging complete SLAM sessions to Rerun
// Integrates with ISLAMSystem to automatically log all pipeline stages

#include "rerun_viewer.h"
#include "sap/interfaces/slam_backend.h"
#include <chrono>
#include <mutex>

namespace sap::plugins {

// SLAM session metadata
struct SLAMSessionInfo {
    std::string session_id;
    std::string input_source;       // "quest_3", "azure_kinect", etc.
    std::string tracker_type;       // "icp", "feature", "droid", etc.
    std::string mapper_type;        // "tsdf", "gaussian", "surfel", etc.
    std::chrono::system_clock::time_point start_time;
    int64_t total_frames = 0;
    int64_t keyframes = 0;
    float total_distance_m = 0.0f;
    bool has_loop_closures = false;
};

// Logging level for different components
enum class RerunLogLevel {
    MINIMAL,    // Only poses and keyframes
    STANDARD,   // Poses, RGB, depth, point cloud
    DETAILED,   // + features, tracking status, optimization
    FULL        // Everything including debug visualizations
};

// SLAM-specific logger that wraps RerunViewer
class RerunSLAMLogger {
public:
    RerunSLAMLogger();
    ~RerunSLAMLogger();

    // ==================== Session Management ====================

    // Start a new SLAM session
    bool startSession(const SLAMSessionInfo& info, const RerunConfig& config);

    // End the current session
    void endSession();

    // Get session info
    SLAMSessionInfo getSessionInfo() const { return session_info_; }

    // Set logging level
    void setLogLevel(RerunLogLevel level) { log_level_ = level; }
    RerunLogLevel getLogLevel() const { return log_level_; }

    // ==================== Pipeline Integration ====================

    // Log a complete tracking iteration
    void logTrackingIteration(
        const Frame& frame,
        const TrackingResult& result,
        const SceneData* scene = nullptr  // Optional scene for map visualization
    );

    // Log mapping update
    void logMappingUpdate(
        const Frame& frame,
        const torch::Tensor& pose,
        const SceneData& scene_before,
        const SceneData& scene_after
    );

    // Log optimization step
    void logOptimization(
        const std::vector<Frame>& keyframes,
        const std::vector<torch::Tensor>& poses_before,
        const std::vector<torch::Tensor>& poses_after,
        const OptimizationResult& result
    );

    // Log loop closure
    void logLoopClosure(
        const Frame& query_frame,
        const Frame& match_frame,
        const torch::Tensor& relative_pose,
        float confidence
    );

    // ==================== Specialized Logging ====================

    // Log depth estimation comparison (mono vs sensor)
    void logDepthComparison(
        const torch::Tensor& sensor_depth,
        const torch::Tensor& estimated_depth,
        const torch::Tensor& confidence
    );

    // Log feature matches between frames
    void logFeatureMatches(
        const Frame& frame1,
        const Frame& frame2,
        const torch::Tensor& keypoints1,  // [N, 2]
        const torch::Tensor& keypoints2,  // [M, 2]
        const torch::Tensor& matches      // [K, 2] indices
    );

    // Log Gaussian splat training progress
    void logGaussianTraining(
        int iteration,
        float loss,
        const SceneData& gaussians,
        const RenderResult& render
    );

    // Log render quality metrics
    void logRenderMetrics(
        const Frame& ground_truth,
        const RenderResult& render,
        float psnr,
        float ssim,
        float lpips
    );

    // ==================== Quest 3 Specific ====================

    // Log Quest 3 multimodal data
    void logQuest3Frame(
        const Frame& frame,
        const HandKeypoints& left_hand,
        const HandKeypoints& right_hand,
        const EyeTrackingData& eye_data,
        const torch::Tensor& quest_slam_pose
    );

    // Log Quest spatial anchors
    void logSpatialAnchor(
        const std::string& anchor_id,
        const torch::Tensor& pose,
        float confidence
    );

    // ==================== Export & Analysis ====================

    // Export trajectory for evaluation (TUM, KITTI format)
    bool exportTrajectory(const std::string& path, const std::string& format = "tum");

    // Export point cloud
    bool exportPointCloud(const std::string& path, const std::string& format = "ply");

    // Get accumulated statistics
    struct SLAMStats {
        int64_t total_frames;
        int64_t keyframes;
        int64_t tracking_failures;
        float avg_tracking_time_ms;
        float avg_mapping_time_ms;
        float total_trajectory_length_m;
        int loop_closures;
        float final_ate_rmse;  // Absolute trajectory error (if GT available)
    };
    SLAMStats getStats() const;

    // Access underlying viewer
    RerunViewer* getViewer() { return viewer_.get(); }
    const RerunViewer* getViewer() const { return viewer_.get(); }

private:
    std::unique_ptr<RerunViewer> viewer_;
    SLAMSessionInfo session_info_;
    RerunLogLevel log_level_ = RerunLogLevel::STANDARD;
    bool session_active_ = false;
    mutable std::mutex mutex_;

    // Tracking history for trajectory visualization
    std::vector<torch::Tensor> pose_history_;
    std::vector<int64_t> timestamp_history_;

    // Accumulated statistics
    SLAMStats stats_;

    // Helper methods
    void logFrameBasic(const Frame& frame, int64_t frame_id);
    void logFrameDetailed(const Frame& frame, int64_t frame_id);
    void updateTrajectory(const torch::Tensor& pose, int64_t frame_id);
    void computeStats();
};

// ==================== Pipeline Hooks ====================

// RAII helper for timing and logging pipeline stages
class RerunScopedTimer {
public:
    RerunScopedTimer(RerunSLAMLogger* logger, const std::string& stage_name);
    ~RerunScopedTimer();

private:
    RerunSLAMLogger* logger_;
    std::string stage_name_;
    std::chrono::high_resolution_clock::time_point start_;
};

// Macro for easy timing
#define RERUN_TIMED_SCOPE(logger, name) \
    RerunScopedTimer _rerun_timer_##__LINE__(logger, name)

// ==================== Factory Functions ====================

// Create a logger connected to a local viewer
std::unique_ptr<RerunSLAMLogger> createLocalRerunLogger(
    const std::string& session_name = "SLAM Session"
);

// Create a logger that saves to file
std::unique_ptr<RerunSLAMLogger> createRecordingRerunLogger(
    const std::string& output_path,
    const std::string& session_name = "SLAM Session"
);

// Create a logger that connects to a remote viewer
std::unique_ptr<RerunSLAMLogger> createRemoteRerunLogger(
    const std::string& host,
    int port,
    const std::string& session_name = "SLAM Session"
);

}  // namespace sap::plugins
