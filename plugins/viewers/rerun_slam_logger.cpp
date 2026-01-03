// Rerun SLAM Logger Implementation
// Provides high-level logging for complete SLAM sessions

#include "rerun_slam_logger.h"
#include <chrono>
#include <iomanip>
#include <sstream>

namespace sap::plugins {

// ============================================================================
// RerunSLAMLogger Implementation
// ============================================================================

RerunSLAMLogger::RerunSLAMLogger()
    : viewer_(std::make_unique<RerunViewer>()) {
    stats_ = {};
}

RerunSLAMLogger::~RerunSLAMLogger() {
    if (session_active_) {
        endSession();
    }
}

bool RerunSLAMLogger::startSession(const SLAMSessionInfo& info, const RerunConfig& config) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (session_active_) {
        endSession();
    }

    session_info_ = info;
    session_info_.start_time = std::chrono::system_clock::now();

    // Generate session ID if not provided
    if (session_info_.session_id.empty()) {
        auto now = std::chrono::system_clock::now();
        auto time_t = std::chrono::system_clock::to_time_t(now);
        std::stringstream ss;
        ss << "slam_" << std::put_time(std::localtime(&time_t), "%Y%m%d_%H%M%S");
        session_info_.session_id = ss.str();
    }

    // Initialize viewer with session-specific config
    RerunConfig session_config = config;
    if (session_config.recording_id.empty()) {
        session_config.recording_id = session_info_.session_id;
    }

    Config viewer_config;
    // TODO: Convert RerunConfig to Config
    if (!viewer_->initialize(viewer_config)) {
        return false;
    }

    // Clear tracking history
    pose_history_.clear();
    timestamp_history_.clear();
    stats_ = {};

    session_active_ = true;
    return true;
}

void RerunSLAMLogger::endSession() {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!session_active_) return;

    // Compute final statistics
    computeStats();

    // Log session summary
    // TODO: Log final stats to Rerun

    viewer_->flush();
    session_active_ = false;
}

void RerunSLAMLogger::logTrackingIteration(
    const Frame& frame,
    const TrackingResult& result,
    const SceneData* scene
) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!session_active_) return;

    int64_t frame_id = frame.frame_id;
    stats_.total_frames++;

    // Set timeline
    viewer_->setTimeSequence("frame", frame_id);
    if (frame.timestamp_ns > 0) {
        viewer_->setTimeSeconds("time", frame.timestamp_ns / 1e9);
    }

    // Log based on level
    if (log_level_ >= RerunLogLevel::MINIMAL) {
        // Always log pose
        if (result.pose.defined()) {
            viewer_->logPose(result.pose, frame_id);
            updateTrajectory(result.pose, frame_id);
        }

        // Log tracking status
        std::string status_str;
        switch (result.status) {
            case TrackingStatus::OK: status_str = "OK"; break;
            case TrackingStatus::LOST: status_str = "LOST"; stats_.tracking_failures++; break;
            case TrackingStatus::INITIALIZING: status_str = "INIT"; break;
            default: status_str = "UNKNOWN";
        }
        viewer_->logTrackingStatus(status_str, result.confidence);
    }

    if (log_level_ >= RerunLogLevel::STANDARD) {
        logFrameBasic(frame, frame_id);

        // Log scene if provided
        if (scene) {
            viewer_->showScene(*scene);
        }
    }

    if (log_level_ >= RerunLogLevel::DETAILED) {
        logFrameDetailed(frame, frame_id);
    }

    // Update session info
    session_info_.total_frames = stats_.total_frames;
    if (frame.is_keyframe) {
        session_info_.keyframes++;
        stats_.keyframes++;
    }
}

void RerunSLAMLogger::logMappingUpdate(
    const Frame& frame,
    const torch::Tensor& pose,
    const SceneData& scene_before,
    const SceneData& scene_after
) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!session_active_) return;
    if (log_level_ < RerunLogLevel::DETAILED) return;

    // Log scene state after mapping
    viewer_->showScene(scene_after);

    // TODO: Log point count change, memory usage, etc.
}

void RerunSLAMLogger::logOptimization(
    const std::vector<Frame>& keyframes,
    const std::vector<torch::Tensor>& poses_before,
    const std::vector<torch::Tensor>& poses_after,
    const OptimizationResult& result
) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!session_active_) return;
    if (log_level_ < RerunLogLevel::DETAILED) return;

    // Log pose graph edges
    std::vector<std::pair<int, int>> edges;
    // TODO: Extract edges from optimization result

    viewer_->logOptimizationGraph(poses_after, edges);

    // TODO: Log optimization metrics (iterations, residuals, etc.)
}

void RerunSLAMLogger::logLoopClosure(
    const Frame& query_frame,
    const Frame& match_frame,
    const torch::Tensor& relative_pose,
    float confidence
) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!session_active_) return;

    session_info_.has_loop_closures = true;
    stats_.loop_closures++;

    viewer_->logLoopClosure(query_frame.frame_id, match_frame.frame_id, relative_pose);

    // TODO: Log match visualization (side-by-side images, feature matches)
}

void RerunSLAMLogger::logDepthComparison(
    const torch::Tensor& sensor_depth,
    const torch::Tensor& estimated_depth,
    const torch::Tensor& confidence
) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!session_active_) return;
    if (log_level_ < RerunLogLevel::DETAILED) return;

    // TODO: Log depth comparison visualization
    // - sensor_depth as "depth/sensor"
    // - estimated_depth as "depth/estimated"
    // - difference map as "depth/error"
}

void RerunSLAMLogger::logFeatureMatches(
    const Frame& frame1,
    const Frame& frame2,
    const torch::Tensor& keypoints1,
    const torch::Tensor& keypoints2,
    const torch::Tensor& matches
) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!session_active_) return;
    if (log_level_ < RerunLogLevel::FULL) return;

    // TODO: Log feature matches as line segments in Rerun
}

void RerunSLAMLogger::logGaussianTraining(
    int iteration,
    float loss,
    const SceneData& gaussians,
    const RenderResult& render
) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!session_active_) return;

    // Log on a separate timeline for Gaussian optimization
    viewer_->setTimeSequence("gs_iteration", iteration);

    // Log loss as scalar
    // TODO: rr.log("training/loss", rerun::Scalar(loss))

    // Periodically log Gaussians and renders
    if (iteration % 100 == 0) {
        viewer_->logGaussians(gaussians);
        viewer_->showRender(render);
    }
}

void RerunSLAMLogger::logRenderMetrics(
    const Frame& ground_truth,
    const RenderResult& render,
    float psnr,
    float ssim,
    float lpips
) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!session_active_) return;

    // TODO: Log metrics as scalars
    // rr.log("metrics/psnr", rerun::Scalar(psnr))
    // rr.log("metrics/ssim", rerun::Scalar(ssim))
    // rr.log("metrics/lpips", rerun::Scalar(lpips))
}

void RerunSLAMLogger::logQuest3Frame(
    const Frame& frame,
    const HandKeypoints& left_hand,
    const HandKeypoints& right_hand,
    const EyeTrackingData& eye_data,
    const torch::Tensor& quest_slam_pose
) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!session_active_) return;

    // Log basic frame
    logFrameBasic(frame, frame.frame_id);

    // Log Quest's own SLAM pose
    if (quest_slam_pose.defined()) {
        viewer_->logPose(quest_slam_pose, frame.frame_id);
    }

    // Log hand tracking
    if (left_hand.is_tracked || right_hand.is_tracked) {
        torch::Tensor left_positions, right_positions;

        if (left_hand.is_tracked) {
            left_positions = torch::from_blob(
                (void*)left_hand.positions.data(),
                {21, 3}, torch::kFloat32
            ).clone();
        }

        if (right_hand.is_tracked) {
            right_positions = torch::from_blob(
                (void*)right_hand.positions.data(),
                {21, 3}, torch::kFloat32
            ).clone();
        }

        viewer_->logHandTracking(left_positions, right_positions);
    }

    // Log eye tracking
    if (eye_data.is_valid) {
        torch::Tensor gaze = torch::from_blob(
            (void*)eye_data.combined_gaze.data(),
            {3}, torch::kFloat32
        ).clone();
        viewer_->logEyeGaze(gaze, eye_data.convergence_distance);
    }
}

void RerunSLAMLogger::logSpatialAnchor(
    const std::string& anchor_id,
    const torch::Tensor& pose,
    float confidence
) {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!session_active_) return;

    // TODO: Log anchor as a labeled coordinate frame
    // rr.log("world/anchors/" + anchor_id, rerun::Transform3D(...))
}

bool RerunSLAMLogger::exportTrajectory(const std::string& path, const std::string& format) {
    std::lock_guard<std::mutex> lock(mutex_);

    // TODO: Export trajectory in TUM/KITTI format
    return false;
}

bool RerunSLAMLogger::exportPointCloud(const std::string& path, const std::string& format) {
    std::lock_guard<std::mutex> lock(mutex_);

    // TODO: Export accumulated point cloud
    return false;
}

RerunSLAMLogger::SLAMStats RerunSLAMLogger::getStats() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return stats_;
}

void RerunSLAMLogger::logFrameBasic(const Frame& frame, int64_t frame_id) {
    // Log RGB if available
    if (frame.hasRGB()) {
        viewer_->showImage(RerunPaths::CAMERA_RGB, frame.rgb.value());
    }

    // Log depth if available
    if (frame.hasDepth()) {
        viewer_->showDepth(RerunPaths::CAMERA_DEPTH, frame.depth.value());
    }
}

void RerunSLAMLogger::logFrameDetailed(const Frame& frame, int64_t frame_id) {
    // Log IMU data
    if (!frame.imu_samples.empty()) {
        viewer_->logIMU(frame.imu_samples);
    }

    // Log any extensions (features, etc.)
    for (const auto& [key, tensor] : frame.extensions) {
        // TODO: Log extension data based on type
    }
}

void RerunSLAMLogger::updateTrajectory(const torch::Tensor& pose, int64_t frame_id) {
    pose_history_.push_back(pose.clone());
    timestamp_history_.push_back(frame_id);

    // Update trajectory visualization
    if (pose_history_.size() > 1) {
        viewer_->showTrajectory(pose_history_);
    }

    // Update total distance
    if (pose_history_.size() > 1) {
        auto prev_pos = pose_history_[pose_history_.size() - 2].slice(0, 0, 3).slice(1, 3, 4);
        auto curr_pos = pose.slice(0, 0, 3).slice(1, 3, 4);
        float dist = (curr_pos - prev_pos).norm().item<float>();
        stats_.total_trajectory_length_m += dist;
        session_info_.total_distance_m = stats_.total_trajectory_length_m;
    }
}

void RerunSLAMLogger::computeStats() {
    // Final stats computation
}

// ============================================================================
// RerunScopedTimer Implementation
// ============================================================================

RerunScopedTimer::RerunScopedTimer(RerunSLAMLogger* logger, const std::string& stage_name)
    : logger_(logger)
    , stage_name_(stage_name)
    , start_(std::chrono::high_resolution_clock::now()) {
}

RerunScopedTimer::~RerunScopedTimer() {
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - start_);
    float ms = duration.count() / 1000.0f;

    // TODO: Log timing to Rerun
    // rr.log("timing/" + stage_name_, rerun::Scalar(ms))
}

// ============================================================================
// Factory Functions
// ============================================================================

std::unique_ptr<RerunSLAMLogger> createLocalRerunLogger(const std::string& session_name) {
    auto logger = std::make_unique<RerunSLAMLogger>();

    SLAMSessionInfo info;
    info.session_id = session_name;

    RerunConfig config;
    config.spawn_viewer = true;
    config.application_id = "spatial-ai-playground";

    if (logger->startSession(info, config)) {
        return logger;
    }
    return nullptr;
}

std::unique_ptr<RerunSLAMLogger> createRecordingRerunLogger(
    const std::string& output_path,
    const std::string& session_name
) {
    auto logger = std::make_unique<RerunSLAMLogger>();

    SLAMSessionInfo info;
    info.session_id = session_name;

    RerunConfig config;
    config.spawn_viewer = false;
    config.save_path = output_path;
    config.application_id = "spatial-ai-playground";

    if (logger->startSession(info, config)) {
        return logger;
    }
    return nullptr;
}

std::unique_ptr<RerunSLAMLogger> createRemoteRerunLogger(
    const std::string& host,
    int port,
    const std::string& session_name
) {
    auto logger = std::make_unique<RerunSLAMLogger>();

    SLAMSessionInfo info;
    info.session_id = session_name;

    RerunConfig config;
    config.spawn_viewer = false;
    config.connect_addr = host + ":" + std::to_string(port);
    config.application_id = "spatial-ai-playground";

    if (logger->startSession(info, config)) {
        return logger;
    }
    return nullptr;
}

}  // namespace sap::plugins
