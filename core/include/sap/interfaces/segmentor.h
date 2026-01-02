


#pragma once

#include "sap/core/plugin.h"
#include "sap/core/frame.h"
#include "sap/core/config.h"

namespace sap {

// Segmentation result
struct SegmentationResult {
    torch::Tensor masks;        // [N, H, W] binary masks
    torch::Tensor scores;       // [N] confidence scores
    torch::Tensor boxes;        // [N, 4] bounding boxes (x1, y1, x2, y2)

    // Optional: semantic information
    std::optional<torch::Tensor> class_ids;  // [N] class indices
    std::vector<std::string> class_names;    // Class label strings

    // Optional: instance embedding for tracking
    std::optional<torch::Tensor> embeddings;  // [N, D] instance embeddings
};

// Dynamic object result (for SLAM)
struct DynamicResult {
    torch::Tensor static_mask;   // [H, W] 1=static, 0=dynamic
    torch::Tensor dynamic_mask;  // [H, W] 1=dynamic, 0=static
    std::vector<SegmentationResult> dynamic_objects;  // Per-object segmentation
    torch::Tensor uncertainty;   // [H, W] uncertainty map
};

// Segmentation interface
class ISegmentor : public IPlugin {
public:
    virtual ~ISegmentor() = default;

    // ==================== Automatic Segmentation ====================

    // Segment everything in the frame
    virtual SegmentationResult segment(const Frame& frame) = 0;

    // Segment with semantic classes
    virtual SegmentationResult segmentSemantic(const Frame& frame) {
        return segment(frame);  // Default: same as regular segment
    }

    // ==================== Prompted Segmentation ====================

    // Segment with point prompts
    // points: [N, 2] (x, y) coordinates
    // labels: [N] 1=foreground, 0=background
    virtual SegmentationResult segmentPoints(
        const Frame& frame,
        const torch::Tensor& points,
        const torch::Tensor& labels
    ) = 0;

    // Segment with box prompts
    // boxes: [N, 4] (x1, y1, x2, y2)
    virtual SegmentationResult segmentBoxes(
        const Frame& frame,
        const torch::Tensor& boxes
    ) = 0;

    // Segment with text prompts (CLIP-based segmenters)
    virtual SegmentationResult segmentText(
        const Frame& frame,
        const std::vector<std::string>& prompts
    ) {
        throw std::runtime_error("Text prompts not supported by this segmentor");
    }

    // Segment with mask prompt (refine existing mask)
    virtual SegmentationResult segmentMask(
        const Frame& frame,
        const torch::Tensor& mask_prompt
    ) {
        throw std::runtime_error("Mask prompts not supported by this segmentor");
    }

    // ==================== Dynamic Object Detection ====================

    // Detect dynamic (moving) regions for SLAM
    // Uses temporal information if available
    virtual DynamicResult detectDynamic(
        const Frame& current,
        const std::vector<Frame>& history = {}
    ) {
        // Default: assume everything is static
        DynamicResult result;
        int h = current.intrinsics.height;
        int w = current.intrinsics.width;
        result.static_mask = torch::ones({h, w}, torch::kFloat32);
        result.dynamic_mask = torch::zeros({h, w}, torch::kFloat32);
        result.uncertainty = torch::zeros({h, w}, torch::kFloat32);
        return result;
    }

    // ==================== Properties ====================

    // Does this segmentor support text prompts?
    virtual bool supportsTextPrompts() const { return false; }

    // Does this segmentor provide semantic classes?
    virtual bool providesSemantics() const { return false; }

    // Does this segmentor support dynamic detection?
    virtual bool supportsDynamicDetection() const { return false; }

    // Get supported class names (for semantic segmentation)
    virtual std::vector<std::string> getClassNames() const { return {}; }
};

// Segmentor implementations to support:
// - SAM (Segment Anything Model)
// - SAM 2 (with video support)
// - Grounded SAM (text-prompted)
// - Mask2Former (semantic/panoptic)
// - Mobile-SAM (fast)
// - FastSAM

}  // namespace sap
