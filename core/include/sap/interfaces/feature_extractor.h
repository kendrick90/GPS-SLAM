


#pragma once

#include "sap/core/plugin.h"
#include "sap/core/frame.h"
#include "sap/core/config.h"

namespace sap {

// Feature extraction result
struct FeatureResult {
    // Dense features: [C, H', W'] feature map
    // Sparse features: [N, C] feature vectors at keypoints
    torch::Tensor features;

    // For sparse features: keypoint locations [N, 2]
    std::optional<torch::Tensor> keypoints;

    // For sparse features: keypoint scores [N]
    std::optional<torch::Tensor> scores;

    // For sparse features: descriptors [N, D]
    std::optional<torch::Tensor> descriptors;

    // Feature map scale relative to input (e.g., 0.25 for 1/4 resolution)
    float scale = 1.0f;

    // Feature dimension
    int feature_dim = 0;

    // Is this a dense feature map or sparse keypoints?
    bool is_dense = true;
};

// Feature matching result
struct MatchResult {
    torch::Tensor matches;      // [M, 2] pairs of (idx1, idx2)
    torch::Tensor scores;       // [M] match confidence
    torch::Tensor inlier_mask;  // [M] bool, after geometric verification
};

// Feature extractor interface
// Supports both dense (DINO, CLIP) and sparse (ORB, SuperPoint) features
class IFeatureExtractor : public IPlugin {
public:
    virtual ~IFeatureExtractor() = default;

    // ==================== Feature Extraction ====================

    // Extract features from frame
    virtual FeatureResult extract(const Frame& frame) = 0;

    // Extract features at specific points (for sparse methods)
    virtual FeatureResult extractAtPoints(
        const Frame& frame,
        const torch::Tensor& points  // [N, 2]
    ) {
        // Default: extract full then sample
        auto result = extract(frame);
        // Subclasses should override for efficiency
        return result;
    }

    // Batch extraction
    virtual std::vector<FeatureResult> extractBatch(const FrameBatch& batch) {
        std::vector<FeatureResult> results;
        for (const auto& frame : batch.frames) {
            results.push_back(extract(frame));
        }
        return results;
    }

    // ==================== Feature Matching ====================

    // Match features between two frames
    virtual MatchResult match(
        const FeatureResult& features1,
        const FeatureResult& features2
    ) {
        throw std::runtime_error("Feature matching not implemented");
    }

    // Match with geometric verification (using depth or motion)
    virtual MatchResult matchWithVerification(
        const Frame& frame1, const FeatureResult& features1,
        const Frame& frame2, const FeatureResult& features2
    ) {
        return match(features1, features2);
    }

    // ==================== Properties ====================

    // Get feature dimension
    virtual int featureDimension() const = 0;

    // Is this a dense feature extractor?
    virtual bool isDense() const = 0;

    // Does this support efficient point sampling?
    virtual bool supportsPointSampling() const { return false; }

    // Does this include a matcher?
    virtual bool supportMatching() const { return false; }

    // Get output scale (relative to input resolution)
    virtual float outputScale() const { return 1.0f; }

    // Get recommended input resolution (0 = any)
    virtual std::pair<int, int> recommendedResolution() const { return {0, 0}; }
};

// Feature extractor implementations to support:
// Dense:
// - DINOv2 (general features)
// - CLIP (text-aligned features)
// - SAM encoder (segmentation features)
// - SD features (diffusion model features)
//
// Sparse:
// - SuperPoint (learned keypoints)
// - ORB (classical)
// - SIFT (classical)
// - LightGlue/SuperGlue (with matching)
// - LoFTR (dense matching)

}  // namespace sap
