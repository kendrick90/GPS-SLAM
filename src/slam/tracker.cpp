#include "tracker.h"
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

namespace sap {

TrackingResult SimpleVO::track(const FrameData& frame) {
    TrackingResult result;
    result.frame_id = frame.frame_id;

    if (!frame.valid()) {
        result.status = TrackingStatus::LOST;
        return result;
    }

    // Convert to grayscale
    cv::Mat gray;
    if (frame.rgb.channels() == 3) {
        cv::cvtColor(frame.rgb, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = frame.rgb;
    }

    // First frame initialization
    if (prev_gray_.empty()) {
        prev_gray_ = gray.clone();
        cv::goodFeaturesToTrack(prev_gray_, prev_points_, 500, 0.01, 10);
        result.status = TrackingStatus::INITIALIZING;
        result.pose = current_pose_;
        trajectory_.push_back(current_pose_);
        return result;
    }

    // Track features with optical flow
    std::vector<cv::Point2f> curr_points;
    std::vector<uchar> status;
    std::vector<float> err;

    if (!prev_points_.empty()) {
        cv::calcOpticalFlowPyrLK(prev_gray_, gray, prev_points_, curr_points, status, err);

        // Filter valid points
        std::vector<cv::Point2f> good_prev, good_curr;
        for (size_t i = 0; i < status.size(); i++) {
            if (status[i] && err[i] < 12.0f) {
                good_prev.push_back(prev_points_[i]);
                good_curr.push_back(curr_points[i]);
            }
        }

        // Estimate essential matrix if enough points
        if (good_prev.size() >= 8) {
            cv::Mat K = (cv::Mat_<float>(3, 3) <<
                frame.intrinsics.fx, 0, frame.intrinsics.cx,
                0, frame.intrinsics.fy, frame.intrinsics.cy,
                0, 0, 1);

            cv::Mat E, mask;
            E = cv::findEssentialMat(good_curr, good_prev, K, cv::RANSAC, 0.999, 1.0, mask);

            if (!E.empty()) {
                cv::Mat R, t;
                int inliers = cv::recoverPose(E, good_curr, good_prev, K, R, t, mask);

                if (inliers > 10) {
                    // Convert to Eigen
                    Eigen::Matrix3f R_eigen;
                    for (int i = 0; i < 3; i++) {
                        for (int j = 0; j < 3; j++) {
                            R_eigen(i, j) = static_cast<float>(R.at<double>(i, j));
                        }
                    }
                    Eigen::Vector3f t_eigen(
                        static_cast<float>(t.at<double>(0)),
                        static_cast<float>(t.at<double>(1)),
                        static_cast<float>(t.at<double>(2))
                    );

                    // Scale is unknown in monocular VO, use unit scale
                    float scale = 0.1f;  // Arbitrary scale factor

                    // Update pose: new_pose = old_pose * [R|t]
                    Eigen::Matrix4f delta = Eigen::Matrix4f::Identity();
                    delta.block<3, 3>(0, 0) = R_eigen;
                    delta.block<3, 1>(0, 3) = t_eigen * scale;

                    current_pose_ = current_pose_ * delta;

                    result.status = TrackingStatus::GOOD;
                    result.confidence = static_cast<float>(inliers) / good_prev.size();
                } else {
                    result.status = TrackingStatus::POOR;
                }
            } else {
                result.status = TrackingStatus::POOR;
            }
        } else {
            result.status = TrackingStatus::POOR;
        }

        // Update for next frame
        prev_points_ = good_curr;
    }

    // Redetect features if too few
    if (prev_points_.size() < 100) {
        cv::goodFeaturesToTrack(gray, prev_points_, 500, 0.01, 10);
    }

    result.pose = current_pose_;
    trajectory_.push_back(current_pose_);

    // Mark keyframes every 10 frames
    if (frame_count_++ % 10 == 0) {
        result.is_keyframe = true;
        keyframes_.push_back(result);
    }

    prev_gray_ = gray.clone();
    return result;
}

void SimpleVO::reset() {
    current_pose_ = Eigen::Matrix4f::Identity();
    trajectory_.clear();
    keyframes_.clear();
    prev_gray_ = cv::Mat();
    prev_points_.clear();
    frame_count_ = 0;
}

void SimpleVO::setPose(const Eigen::Matrix4f& pose) {
    current_pose_ = pose;
}

Eigen::Matrix4f SimpleVO::getCurrentPose() const {
    return current_pose_;
}

std::vector<Eigen::Matrix4f> SimpleVO::getTrajectory() const {
    return trajectory_;
}

std::vector<TrackingResult> SimpleVO::getKeyframes() const {
    return keyframes_;
}

// Factory method
std::unique_ptr<Tracker> Tracker::create(const std::string& type) {
    if (type == "simple_vo" || type == "default") {
        return std::make_unique<SimpleVO>();
    }
    return nullptr;
}

// Camera factory (placeholder)
std::unique_ptr<Camera> Camera::create(const std::string& type) {
    // Will be extended with more camera types
    return nullptr;
}

}  // namespace sap
