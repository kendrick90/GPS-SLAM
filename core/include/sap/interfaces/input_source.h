


#pragma once

#include "sap/core/plugin.h"
#include "sap/core/frame.h"
#include "sap/core/config.h"

namespace sap {

// Input source interface - provides frames from cameras, files, or network
// Modeled after InfiniTAM's ImageSourceEngine pattern
class IInputSource : public IPlugin {
public:
    virtual ~IInputSource() = default;

    // ==================== Core Capture Interface ====================

    // Check if more frames can be provided (connection alive, file not ended)
    virtual bool hasMoreFrames() const = 0;

    // Check if frames are available RIGHT NOW (for non-blocking capture)
    virtual bool hasFramesNow() const { return hasMoreFrames(); }

    // Get the next frame (blocking or non-blocking depending on implementation)
    virtual Frame getNextFrame() = 0;

    // ==================== Camera Properties ====================

    // Get camera intrinsics
    virtual CameraIntrinsics getIntrinsics() const = 0;

    // Does this source provide depth data?
    virtual bool hasDepth() const = 0;

    // Does this source provide IMU data?
    virtual bool hasIMU() const { return false; }

    // Does this source provide stereo?
    virtual bool hasStereo() const { return false; }

    // ==================== Seekable Sources (files, datasets) ====================

    // Can we seek to arbitrary frames?
    virtual bool isSeekable() const { return false; }

    // Seek to specific frame (returns false if not supported or failed)
    virtual bool seek(int64_t frame_id) { return false; }

    // Total frame count (-1 if unknown/live)
    virtual int64_t frameCount() const { return -1; }

    // Current frame index
    virtual int64_t currentFrameIndex() const { return -1; }

    // ==================== Live Control ====================

    // Start/stop capture (for sources that need explicit control)
    virtual bool startCapture() { return true; }
    virtual void stopCapture() {}

    // Get/set exposure (if supported, returns -1 if not)
    virtual float getExposure() const { return -1.0f; }
    virtual bool setExposure(float ms) { return false; }

    // Get/set gain
    virtual float getGain() const { return -1.0f; }
    virtual bool setGain(float gain) { return false; }

    // ==================== Calibration ====================

    // Load calibration from file
    virtual bool loadCalibration(const std::string& path) { return false; }

    // Get depth scale (depth_meters = raw_depth * scale)
    virtual float getDepthScale() const { return 0.001f; }  // Default: mm to m
};

// Helper: Input source that wraps legacy InfiniTAM ImageSourceEngine
// This allows gradual migration of existing input sources
class LegacyInputSourceWrapper : public IInputSource {
    // Implementation would wrap InputSource::ImageSourceEngine
    // To be implemented when migrating InfiniTAM
};

}  // namespace sap
