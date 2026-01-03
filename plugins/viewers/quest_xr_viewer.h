#pragma once
// Quest XR Viewer Plugin
// Renders SLAM data directly in Quest 3 VR/MR passthrough using OpenXR
// Enables walking through your reconstruction in real-time

#include "sap/interfaces/viewer.h"
#include "sap/interfaces/slam_backend.h"
#include <memory>

namespace sap::plugins {

// OpenXR rendering modes for SLAM visualization
enum class XRRenderMode {
    PASSTHROUGH_AR,     // Overlay reconstruction on real world (MR)
    VIRTUAL_VR,         // Full VR environment with reconstruction
    SPLIT_COMPARE,      // Side-by-side real vs reconstructed
    FIRST_PERSON_REPLAY // Replay trajectory from camera's POV
};

// Configuration for Quest XR viewer
struct QuestXRConfig {
    XRRenderMode render_mode = XRRenderMode::PASSTHROUGH_AR;

    // Rendering options
    bool render_gaussians = true;      // Render 3DGS splats
    bool render_point_cloud = false;   // Fallback point cloud
    bool render_mesh = false;          // TSDF mesh
    bool render_trajectory = true;     // Camera path visualization
    bool render_hands = true;          // Show tracked hands

    // AR overlay settings
    float overlay_opacity = 0.8f;      // Blend with passthrough
    float reconstruction_scale = 1.0f; // Scale factor for reconstruction

    // Performance
    int gaussian_budget = 500000;      // Max Gaussians to render
    bool use_foveated_rendering = true;
    int msaa_samples = 4;

    // Alignment
    bool auto_align_to_quest_slam = true;  // Align with Quest's world
    bool use_spatial_anchors = true;       // Persist alignment
};

// Quest XR Viewer - renders SLAM in VR/AR
class QuestXRViewer : public IViewer {
public:
    QuestXRViewer();
    ~QuestXRViewer() override;

    // IPlugin interface
    bool initialize(const Config& config) override;
    void shutdown() override;
    std::string name() const override { return "quest_xr"; }
    std::string version() const override { return "1.0.0"; }

    // IViewer interface
    void showFrame(const Frame& frame) override;
    void showRender(const RenderResult& render) override;
    void showComparison(const Frame& input, const RenderResult& render) override;
    void showImage(const std::string& window_name, const torch::Tensor& image) override;
    void showDepth(const std::string& window_name, const torch::Tensor& depth,
                   float min_depth = 0.0f, float max_depth = 10.0f) override;

    // 3D visualization (these are the key methods for XR)
    void showScene(const SceneData& scene) override;
    void showTrajectory(const std::vector<torch::Tensor>& poses) override;
    void addCameraFrustum(const torch::Tensor& pose,
                          const CameraIntrinsics& intrinsics,
                          float scale = 0.1f) override;

    // Window management
    bool shouldClose() const override;
    void processEvents() override;
    int waitKey(int timeout_ms = 0) override { return -1; }

    // Interactive camera (Quest headset IS the camera)
    bool supportsInteractiveCamera() const override { return true; }
    std::optional<torch::Tensor> getInteractiveCameraPose() const override;
    void setInteractiveCameraPose(const torch::Tensor& pose) override {}

    // ==================== Quest XR Specific ====================

    // Set rendering mode
    void setRenderMode(XRRenderMode mode);
    XRRenderMode getRenderMode() const { return config_.render_mode; }

    // Configuration
    void setConfig(const QuestXRConfig& config) { config_ = config; }
    QuestXRConfig getConfig() const { return config_; }

    // Scene management
    void setGaussianScene(const SceneData& gaussians);
    void setMeshScene(const torch::Tensor& vertices, const torch::Tensor& faces,
                      const torch::Tensor& colors = {});
    void setPointCloud(const torch::Tensor& points, const torch::Tensor& colors);
    void clearScene();

    // World alignment
    // Align reconstruction coordinate frame to Quest's world
    bool alignToQuestWorld(const torch::Tensor& transform);
    bool alignUsingSpatialAnchor(const std::string& anchor_id);
    torch::Tensor getWorldAlignment() const;

    // Controller interaction
    struct ControllerState {
        torch::Tensor pose;          // [4,4] controller pose
        bool trigger_pressed;
        bool grip_pressed;
        float trigger_value;
        torch::Tensor thumbstick;    // [2] x,y
    };
    ControllerState getLeftController() const;
    ControllerState getRightController() const;

    // Interaction modes
    enum class InteractionMode {
        VIEW_ONLY,      // Just look around
        TELEPORT,       // Point and teleport
        GRAB_WORLD,     // Grab and move/scale reconstruction
        MEASURE,        // Measure distances in reconstruction
        ANNOTATE        // Add 3D annotations
    };
    void setInteractionMode(InteractionMode mode);

    // Teleportation
    void teleportTo(const torch::Tensor& position);
    void resetPosition();  // Return to origin

    // Scale controls (useful for room-scale vs building-scale)
    void setWorldScale(float scale);
    float getWorldScale() const;

    // Screenshot/recording in VR
    bool captureStereoPair(const std::string& left_path, const std::string& right_path);
    bool startRecording(const std::string& path, int fps = 72) override;
    void stopRecording() override;
    bool isRecording() const override;

    // ==================== OpenXR Session ====================

    // Check if OpenXR is available
    static bool isOpenXRAvailable();

    // Session state
    bool isSessionRunning() const;
    bool isPassthroughEnabled() const;

    // Enable/disable passthrough
    bool enablePassthrough(bool enable);

    // Frame timing for smooth rendering
    struct FrameTiming {
        int64_t predicted_display_time;
        float display_period_ns;
        int64_t frame_id;
    };
    FrameTiming getFrameTiming() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
    QuestXRConfig config_;
    bool session_running_ = false;
};

// ==================== OpenXR Gaussian Splatting Renderer ====================
// Custom renderer for 3DGS in OpenXR stereo

class OpenXRGaussianRenderer {
public:
    OpenXRGaussianRenderer();
    ~OpenXRGaussianRenderer();

    // Initialize with OpenXR session
    bool initialize(/* XrSession, XrSpace */);
    void shutdown();

    // Upload Gaussian data to GPU
    void setGaussians(const SceneData& gaussians);

    // Render for both eyes
    struct StereoView {
        torch::Tensor left_pose;   // [4,4]
        torch::Tensor right_pose;  // [4,4]
        float left_fov[4];         // L, R, U, D tangents
        float right_fov[4];
        int width, height;
    };
    void render(const StereoView& view);

    // Performance stats
    struct RenderStats {
        float render_time_ms;
        int gaussians_rendered;
        int gaussians_culled;
    };
    RenderStats getStats() const;

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace sap::plugins

// ============================================================================
// Usage Example - Quest 3 SLAM Viewer App
// ============================================================================
//
// This would be deployed as a Quest APK:
//
// int main() {
//     // Initialize Quest XR viewer
//     QuestXRViewer viewer;
//     QuestXRConfig config;
//     config.render_mode = XRRenderMode::PASSTHROUGH_AR;
//     config.render_gaussians = true;
//     viewer.initialize(config);
//
//     // Load reconstruction from PC or local storage
//     SceneData gaussians = loadGaussians("reconstruction.ply");
//     viewer.setGaussianScene(gaussians);
//
//     // Align to Quest's world using a spatial anchor
//     viewer.alignUsingSpatialAnchor("room_origin");
//
//     // Main render loop
//     while (!viewer.shouldClose()) {
//         viewer.processEvents();
//
//         // Get headset pose for interaction
//         auto pose = viewer.getInteractiveCameraPose();
//
//         // Handle controller input
//         auto left = viewer.getLeftController();
//         if (left.trigger_pressed) {
//             viewer.teleportTo(/* raycast result */);
//         }
//
//         // Scene is rendered automatically by OpenXR compositor
//     }
//
//     return 0;
// }
