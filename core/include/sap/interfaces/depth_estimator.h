


#pragma once

#include "sap/core/plugin.h"
#include "sap/core/frame.h"
#include "sap/core/config.h"

namespace sap {

// Result from depth estimation
struct DepthResult {
    torch::Tensor depth;        // [H, W] predicted depth
    torch::Tensor confidence;   // [H, W] optional confidence map

    bool is_metric = false;     // true if depth is in meters, false if relative/disparity
    float scale = 1.0f;         // Scale factor (metric_depth = predicted * scale)
    float shift = 0.0f;         // Shift factor (metric_depth = predicted * scale + shift)

    // For multi-scale predictions
    std::vector<torch::Tensor> pyramid;  // Optional: multi-scale depth pyramid
};

// Depth estimation interface
// Supports various monocular and multi-view depth estimation methods
class IDepthEstimator : public IPlugin {
public:
    virtual ~IDepthEstimator() = default;

    // ==================== Core Estimation ====================

    // Estimate depth from single frame
    virtual DepthResult estimate(const Frame& frame) = 0;

    // Estimate depth with scale alignment to sparse points
    // points: [N, 3] sparse depth points from SLAM/SfM
    virtual DepthResult estimateWithAlignment(const Frame& frame,
                                               const torch::Tensor& sparse_points) {
        auto result = estimate(frame);
        // Default: no alignment, subclasses can override
        return result;
    }

    // ==================== Batch Processing ====================

    // Batch estimation (for efficiency on GPU)
    virtual std::vector<DepthResult> estimateBatch(const FrameBatch& batch) {
        std::vector<DepthResult> results;
        for (const auto& frame : batch.frames) {
            results.push_back(estimate(frame));
        }
        return results;
    }

    // ==================== Multi-View Estimation ====================

    // Some methods (DUSt3R, MonST3R) use pairs or sequences
    virtual DepthResult estimatePair(const Frame& frame1, const Frame& frame2) {
        // Default: just estimate first frame
        return estimate(frame1);
    }

    virtual std::vector<DepthResult> estimateSequence(const FrameBatch& sequence) {
        return estimateBatch(sequence);
    }

    // ==================== Properties ====================

    // Does this method produce metric (absolute) depth?
    virtual bool supportsMetricDepth() const = 0;

    // Does this method support batch processing efficiently?
    virtual bool supportsBatchProcessing() const { return false; }

    // Does this method support multi-view input?
    virtual bool supportsMultiView() const { return false; }

    // Get expected input resolution (0 = any)
    virtual std::pair<int, int> preferredResolution() const { return {0, 0}; }

    // Get inference time estimate (ms) for given resolution
    virtual float estimatedInferenceTime(int width, int height) const { return -1.0f; }
};

// Depth estimator implementations to support:
// - Depth Anything V2 (fast, relative depth)
// - Metric3D V2 (metric depth)
// - MiDaS (classic monocular)
// - DUSt3R / MonST3R (multi-view)
// - ZoeDepth (metric from relative)
// - UniDepth (unified metric depth)

}  // namespace sap
