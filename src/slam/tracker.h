#pragma once

#include "../sensors/camera.h"
#include <Eigen/Geometry>
#include <vector>
#include <memory>

namespace sap {

// Tracking status
enum class TrackingStatus {
    GOOD,           // Tracking succeeded with high confidence
    POOR,           // Tracking succeeded but low confidence
    LOST,           // Tracking failed
    INITIALIZING   // Not enough data yet
};

// Result of tracking a frame
struct TrackingResult {
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity();  // Camera-to-world
    TrackingStatus status = TrackingStatus::INITIALIZING;
    float confidence = 0.0f;
    bool is_keyframe = false;
    int frame_id = -1;
};

// Point in the map
struct MapPoint {
    Eigen::Vector3f position;
    Eigen::Vector3f color;  // RGB [0, 1]
    Eigen::Vector3f normal;
    float confidence = 1.0f;
};

// Abstract tracker interface
class Tracker {
public:
    virtual ~Tracker() = default;

    // Process a new frame, returns pose estimate
    virtual TrackingResult track(const FrameData& frame) = 0;

    // Track with external depth (from monocular depth estimation)
    virtual TrackingResult track(const FrameData& frame, const cv::Mat& estimated_depth) {
        return track(frame);  // Default: ignore external depth
    }

    // Reset tracker state
    virtual void reset() = 0;

    // Set initial pose (for relocalization)
    virtual void setPose(const Eigen::Matrix4f& pose) = 0;

    // Get current pose
    virtual Eigen::Matrix4f getCurrentPose() const = 0;

    // Get all tracked poses (trajectory)
    virtual std::vector<Eigen::Matrix4f> getTrajectory() const = 0;

    // Get keyframes
    virtual std::vector<TrackingResult> getKeyframes() const = 0;

    // Get current point cloud / map
    virtual std::vector<MapPoint> getPointCloud() const { return {}; }

    // Tracker properties
    virtual bool requiresDepth() const = 0;
    virtual bool supportsIMU() const { return false; }
    virtual std::string name() const = 0;

    // Factory
    static std::unique_ptr<Tracker> create(const std::string& type);
};

// Simple visual odometry tracker (placeholder for now)
class SimpleVO : public Tracker {
public:
    TrackingResult track(const FrameData& frame) override;
    void reset() override;
    void setPose(const Eigen::Matrix4f& pose) override;
    Eigen::Matrix4f getCurrentPose() const override;
    std::vector<Eigen::Matrix4f> getTrajectory() const override;
    std::vector<TrackingResult> getKeyframes() const override;
    bool requiresDepth() const override { return false; }
    std::string name() const override { return "simple_vo"; }

private:
    Eigen::Matrix4f current_pose_ = Eigen::Matrix4f::Identity();
    std::vector<Eigen::Matrix4f> trajectory_;
    std::vector<TrackingResult> keyframes_;
    cv::Mat prev_gray_;
    std::vector<cv::Point2f> prev_points_;
    int frame_count_ = 0;
};

}  // namespace sap
