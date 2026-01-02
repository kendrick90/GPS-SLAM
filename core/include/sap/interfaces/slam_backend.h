

//
// SLAM is decomposed into modular components that can be mixed and matched:
// - ITracker: Visual odometry / pose estimation
// - IMapper: Scene representation (TSDF, surfels, Gaussians, etc.)
// - IOptimizer: Bundle adjustment / pose graph optimization
// - ILoopClosure: Loop closure detection and correction
//
// This allows building different SLAM systems by combining components:
// - RGB-D SLAM: ICP Tracker + TSDF Mapper
// - Monocular SLAM: Feature Tracker + Point Cloud Mapper + BA Optimizer
// - Gaussian SLAM: Depth Tracker + Gaussian Mapper + Photometric Optimizer

#pragma once

#include "sap/core/plugin.h"
#include "sap/core/frame.h"
#include "sap/core/config.h"
#include <optional>

namespace sap {

// ============================================================================
// Scene Representation Types
// ============================================================================

enum class SceneRepresentation {
    TSDF,           // Truncated Signed Distance Function (voxel grid)
    SURFEL,         // Surface elements (oriented disks)
    POINT_CLOUD,    // Sparse or dense point cloud
    GAUSSIAN,       // 3D Gaussians (for Gaussian splatting)
    MESH,           // Triangle mesh
    OCTREE,         // Octree-based representation
    NEURAL,         // Neural implicit (NeRF-style)
    HYBRID,         // Combination of representations
    UNKNOWN
};

// Scene data container - varies by representation type
struct SceneData {
    SceneRepresentation type = SceneRepresentation::UNKNOWN;
    TensorDict data;  // Representation-specific tensors

    // Convenience accessors for common representations

    // Point cloud / Gaussian means: [N, 3]
    torch::Tensor& points() { return data["points"]; }
    const torch::Tensor& points() const { return data.at("points"); }

    // Colors: [N, 3] or SH coefficients [N, K, 3]
    torch::Tensor& colors() { return data["colors"]; }

    // Normals: [N, 3]
    torch::Tensor& normals() { return data["normals"]; }

    // For Gaussians: scales [N, 3], rotations [N, 4]
    torch::Tensor& scales() { return data["scales"]; }
    torch::Tensor& rotations() { return data["rotations"]; }
    torch::Tensor& opacities() { return data["opacities"]; }

    // For mesh: vertices [V, 3], faces [F, 3]
    torch::Tensor& vertices() { return data["vertices"]; }
    torch::Tensor& faces() { return data["faces"]; }

    // For TSDF: volume [X, Y, Z], voxel_size, origin
    torch::Tensor& volume() { return data["volume"]; }
};

// ============================================================================
// Tracking Result
// ============================================================================

enum class TrackingStatus {
    GOOD,           // Tracking succeeded with high confidence
    POOR,           // Tracking succeeded but low confidence
    LOST,           // Tracking failed
    INITIALIZING    // Not enough data yet
};

struct TrackingResult {
    torch::Tensor pose;         // [4, 4] camera-to-world transform
    TrackingStatus status = TrackingStatus::INITIALIZING;
    float confidence = 0.0f;    // 0.0 to 1.0
    bool is_keyframe = false;

    // Optional: covariance of pose estimate [6, 6]
    std::optional<torch::Tensor> covariance;

    // Optional: tracking residual/error
    float residual = 0.0f;

    // Extra data from tracker
    TensorDict extras;
};

// ============================================================================
// ITracker - Pose Estimation / Visual Odometry
// ============================================================================

class ITracker : public IPlugin {
public:
    virtual ~ITracker() = default;

    // Track camera pose for a new frame
    // May use depth, RGB, features, or IMU depending on implementation
    virtual TrackingResult track(const Frame& frame) = 0;

    // Track with reference to scene/map (for map-based tracking)
    virtual TrackingResult track(const Frame& frame, const SceneData& scene) {
        return track(frame);  // Default: ignore scene
    }

    // Reset tracker state
    virtual void reset() = 0;

    // Set initial pose (for relocalization or manual init)
    virtual void setPose(const torch::Tensor& pose) = 0;

    // Get last known pose
    virtual torch::Tensor getLastPose() const = 0;

    // Tracker capabilities
    virtual bool requiresDepth() const = 0;
    virtual bool requiresRGB() const { return true; }
    virtual bool supportsIMU() const { return false; }
};

// Tracker implementations to support:
// - ICP (depth-based, InfiniTAM style)
// - Feature-based (ORB, SuperPoint, etc.)
// - Direct/photometric (LSD-SLAM style)
// - Hybrid (depth + RGB)
// - IMU-fused (extended Kalman filter)
// - Learning-based (DROID-SLAM style)

// ============================================================================
// IMapper - Scene Representation / Geometry
// ============================================================================

class IMapper : public IPlugin {
public:
    virtual ~IMapper() = default;

    // Integrate a new frame into the map
    virtual void integrate(const Frame& frame, const torch::Tensor& pose) = 0;

    // Get current scene representation
    virtual SceneData getScene() const = 0;

    // Get scene representation type
    virtual SceneRepresentation representationType() const = 0;

    // Reset/clear the map
    virtual void reset() = 0;

    // Raycast from viewpoint (for rendering or tracking)
    // Returns: depth [H, W], color [H, W, 3], normal [H, W, 3] (optional)
    virtual TensorDict raycast(const torch::Tensor& pose,
                               const CameraIntrinsics& intrinsics) const = 0;

    // Export to standard formats
    virtual void exportMesh(const std::string& path) const = 0;
    virtual void exportPointCloud(const std::string& path) const = 0;

    // Map statistics
    virtual int64_t pointCount() const = 0;
    virtual std::array<float, 6> boundingBox() const = 0;  // [min_x, min_y, min_z, max_x, max_y, max_z]
};

// Mapper implementations to support:
// - TSDF (InfiniTAM voxel hashing)
// - Surfel (ElasticFusion style)
// - Point cloud (dense or sparse)
// - Gaussian (3DGS representation)
// - Neural SDF (instant-ngp style)

// ============================================================================
// IOptimizer - Bundle Adjustment / Pose Graph Optimization
// ============================================================================

struct OptimizationResult {
    bool converged = false;
    int iterations = 0;
    float final_cost = 0.0f;
    std::vector<torch::Tensor> optimized_poses;
    std::optional<SceneData> optimized_scene;
};

class IOptimizer : public IPlugin {
public:
    virtual ~IOptimizer() = default;

    // Optimize poses and optionally scene
    virtual OptimizationResult optimize(
        const std::vector<Frame>& frames,
        const std::vector<torch::Tensor>& initial_poses,
        const SceneData* scene = nullptr  // Optional scene to co-optimize
    ) = 0;

    // Add constraint between frames
    virtual void addConstraint(int frame_i, int frame_j,
                               const torch::Tensor& relative_pose,
                               const torch::Tensor& information) = 0;

    // Clear all constraints
    virtual void clearConstraints() = 0;

    // Optimization settings
    virtual void setMaxIterations(int iters) = 0;
    virtual void setConvergenceThreshold(float threshold) = 0;
};

// Optimizer implementations:
// - Gauss-Newton / Levenberg-Marquardt
// - g2o wrapper
// - GTSAM wrapper
// - Photometric bundle adjustment

// ============================================================================
// ILoopClosure - Loop Detection and Correction
// ============================================================================

struct LoopCandidate {
    int query_frame_id;
    int match_frame_id;
    float similarity_score;
    std::optional<torch::Tensor> relative_pose;  // If geometric verification done
};

class ILoopClosure : public IPlugin {
public:
    virtual ~ILoopClosure() = default;

    // Add frame to database
    virtual void addFrame(const Frame& frame, int frame_id) = 0;

    // Query for loop candidates
    virtual std::vector<LoopCandidate> detectLoops(const Frame& query, int query_id) = 0;

    // Verify loop geometrically (estimate relative pose)
    virtual std::optional<torch::Tensor> verifyLoop(
        const Frame& frame_i, const Frame& frame_j) = 0;

    // Clear database
    virtual void reset() = 0;
};

// Loop closure implementations:
// - Bag of Words (DBoW2, FBoW)
// - NetVLAD / learned descriptors
// - Fern-based (InfiniTAM FernRelocLib)

// ============================================================================
// ISLAMSystem - Complete SLAM Pipeline (orchestrates components)
// ============================================================================

// High-level SLAM system that combines tracker, mapper, optimizer, loop closure
class ISLAMSystem : public IPlugin {
public:
    virtual ~ISLAMSystem() = default;

    // Process a new frame through the full pipeline
    virtual TrackingResult processFrame(Frame& frame) = 0;

    // Access individual components
    virtual ITracker* getTracker() = 0;
    virtual IMapper* getMapper() = 0;
    virtual IOptimizer* getOptimizer() { return nullptr; }
    virtual ILoopClosure* getLoopClosure() { return nullptr; }

    // Get all keyframes
    virtual std::vector<Frame> getKeyframes() const = 0;

    // Get all poses
    virtual std::vector<torch::Tensor> getPoses() const = 0;

    // Reset entire system
    virtual void reset() = 0;

    // Save/load state
    virtual void save(const std::string& path) const = 0;
    virtual void load(const std::string& path) = 0;
};

}  // namespace sap
