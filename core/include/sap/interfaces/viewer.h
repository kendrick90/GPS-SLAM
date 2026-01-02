


#pragma once

#include "sap/core/plugin.h"
#include "sap/core/frame.h"
#include "sap/core/config.h"
#include "sap/interfaces/renderer.h"

namespace sap {

// Viewer interface - displays frames, renders, and 3D scenes
class IViewer : public IPlugin {
public:
    virtual ~IViewer() = default;

    // ==================== Display Methods ====================

    // Show a single frame (RGB, depth, etc.)
    virtual void showFrame(const Frame& frame) = 0;

    // Show a render result
    virtual void showRender(const RenderResult& render) = 0;

    // Show frame and render side by side for comparison
    virtual void showComparison(const Frame& input, const RenderResult& render) = 0;

    // Show arbitrary image tensor
    virtual void showImage(const std::string& window_name, const torch::Tensor& image) = 0;

    // Show depth with colormap
    virtual void showDepth(const std::string& window_name, const torch::Tensor& depth,
                          float min_depth = 0.0f, float max_depth = 10.0f) = 0;

    // ==================== 3D Visualization ====================

    // Show 3D scene (point cloud, mesh, etc.)
    virtual void showScene(const SceneData& scene) { /* Optional */ }

    // Show camera trajectory
    virtual void showTrajectory(const std::vector<torch::Tensor>& poses) { /* Optional */ }

    // Add camera frustum visualization
    virtual void addCameraFrustum(const torch::Tensor& pose,
                                  const CameraIntrinsics& intrinsics,
                                  float scale = 0.1f) { /* Optional */ }

    // ==================== Window Management ====================

    // Check if viewer should close (user pressed ESC, clicked X, etc.)
    virtual bool shouldClose() const = 0;

    // Process UI events (must be called regularly)
    virtual void processEvents() = 0;

    // Wait for key press, return key code
    virtual int waitKey(int timeout_ms = 0) = 0;

    // Set window title
    virtual void setTitle(const std::string& title) {}

    // Resize window
    virtual void resize(int width, int height) {}

    // ==================== Interactive Camera ====================

    // Does this viewer support interactive camera control?
    virtual bool supportsInteractiveCamera() const { return false; }

    // Get camera pose from user interaction (for free-view rendering)
    virtual std::optional<torch::Tensor> getInteractiveCameraPose() const { return std::nullopt; }

    // Set interactive camera pose
    virtual void setInteractiveCameraPose(const torch::Tensor& pose) {}

    // ==================== Recording ====================

    // Start/stop recording frames to video
    virtual bool startRecording(const std::string& path, int fps = 30) { return false; }
    virtual void stopRecording() {}
    virtual bool isRecording() const { return false; }

    // Screenshot
    virtual bool saveScreenshot(const std::string& path) { return false; }

    // ==================== Network/Remote ====================

    // Is this a remote/network viewer?
    virtual bool isRemote() const { return false; }

    // Set remote connection
    virtual void setRemoteAddress(const std::string& host, int port) {}
    virtual bool isConnected() const { return true; }
};

// Viewer implementations to support:
// - OpenCV HighGUI (simple, cross-platform)
// - Pangolin (3D visualization, interactive)
// - ImGui + OpenGL (modern UI)
// - Remote/network streaming
// - Quest 3 passthrough

}  // namespace sap
