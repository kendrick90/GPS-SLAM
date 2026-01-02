


#pragma once

#include "sap/core/plugin.h"
#include "sap/core/frame.h"
#include "sap/core/config.h"
#include "sap/interfaces/slam_backend.h"

namespace sap {

// Render output
struct RenderResult {
    torch::Tensor color;        // [H, W, 3] RGB image
    torch::Tensor depth;        // [H, W] depth map
    torch::Tensor alpha;        // [H, W] alpha/transmittance

    // Optional outputs
    std::optional<torch::Tensor> normal;     // [H, W, 3] surface normals
    std::optional<torch::Tensor> semantic;   // [H, W] semantic labels
    std::optional<torch::Tensor> instance;   // [H, W] instance IDs
    std::optional<torch::Tensor> flow;       // [H, W, 2] optical flow

    // Renderer-specific extras
    TensorDict extras;

    // Timing info
    float render_time_ms = 0.0f;
};

// Renderer interface
// Supports various neural and classical rendering methods
class IRenderer : public IPlugin {
public:
    virtual ~IRenderer() = default;

    // ==================== Core Rendering ====================

    // Render scene from a camera viewpoint
    virtual RenderResult render(
        const SceneData& scene,
        const torch::Tensor& c2w,           // [4, 4] camera-to-world
        const CameraIntrinsics& intrinsics,
        int width, int height
    ) = 0;

    // Convenience: render from Frame
    RenderResult render(const SceneData& scene, const Frame& camera) {
        return render(scene, camera.getPose(), camera.intrinsics,
                     camera.intrinsics.width, camera.intrinsics.height);
    }

    // ==================== Differentiable Rendering ====================

    // Does this renderer support gradients?
    virtual bool supportsGradients() const = 0;

    // Render with gradient computation
    // Returns render result and sets up autograd for backward pass
    virtual RenderResult renderDifferentiable(
        SceneData& scene,  // Non-const for gradient attachment
        const torch::Tensor& c2w,
        const CameraIntrinsics& intrinsics,
        int width, int height
    ) {
        return render(scene, c2w, intrinsics, width, height);
    }

    // ==================== Scene Support ====================

    // What scene representations does this renderer support?
    virtual std::vector<SceneRepresentation> supportedRepresentations() const = 0;

    // Can this renderer handle this scene?
    bool supportsScene(const SceneData& scene) const {
        auto supported = supportedRepresentations();
        return std::find(supported.begin(), supported.end(), scene.type) != supported.end();
    }

    // ==================== Rendering Options ====================

    // Set background color
    virtual void setBackgroundColor(float r, float g, float b) {}

    // Set near/far planes
    virtual void setDepthRange(float near, float far) {}

    // Enable/disable specific outputs
    virtual void enableNormals(bool enable) {}
    virtual void enableSemantics(bool enable) {}

    // Anti-aliasing settings
    virtual void setAntialiasing(int samples) {}
};

// Renderer implementations to support:
// - Gaussian Splatting (3DGS, gsplat)
// - Triangle Splatting
// - Mesh Rasterization (OpenGL, Vulkan)
// - NeRF-style (volumetric ray marching)
// - TSDF Raycast (InfiniTAM style)
// - Point Splatting
// - Surfel Rendering

// ============================================================================
// Specialized Gaussian Splatting Interface
// ============================================================================

// Extra interface for Gaussian-specific operations
class IGaussianRenderer : public IRenderer {
public:
    // Gaussian-specific rendering with more control
    virtual RenderResult renderGaussians(
        const torch::Tensor& means,       // [N, 3]
        const torch::Tensor& scales,      // [N, 3]
        const torch::Tensor& rotations,   // [N, 4] quaternions
        const torch::Tensor& opacities,   // [N]
        const torch::Tensor& colors,      // [N, 3] or SH [N, K, 3]
        const torch::Tensor& c2w,
        const CameraIntrinsics& intrinsics,
        int width, int height,
        bool compute_gradients = false
    ) = 0;

    // Tile-based rendering stats
    virtual int getNumTilesX() const = 0;
    virtual int getNumTilesY() const = 0;

    // Culling statistics
    virtual int64_t getVisibleGaussianCount() const = 0;

    std::vector<SceneRepresentation> supportedRepresentations() const override {
        return {SceneRepresentation::GAUSSIAN};
    }
};

}  // namespace sap
