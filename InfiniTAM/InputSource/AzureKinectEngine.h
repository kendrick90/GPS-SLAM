// Copyright 2025 GPS-SLAM Team
// Azure Kinect Engine for InfiniTAM/GPS-SLAM
// Based on RealSense2Engine pattern

#pragma once

#include "ImageSourceEngine.h"

#ifdef COMPILE_WITH_AzureKinect
#include <k4a/k4a.h>
#endif

namespace InputSource {

class AzureKinectEngine : public BaseImageSourceEngine
{
private:
    bool dataAvailable;
    bool imuAvailable;

#ifdef COMPILE_WITH_AzureKinect
    k4a_device_t device;
    k4a_device_configuration_t config;
    k4a_calibration_t calibration;
    k4a_transformation_t transformation;
#endif

    Vector2i imageSize_rgb, imageSize_d;
    bool alignDepthToColor;

    // IMU orientation state (quaternion: w, x, y, z)
    float imuQuat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    uint64_t lastImuTimestamp = 0;

public:
    // depth_mode: 0=NFOV_UNBINNED, 1=NFOV_2X2BINNED, 2=WFOV_UNBINNED, 3=WFOV_2X2BINNED
    // color_resolution: 0=720P, 1=1080P, 2=1440P, 3=1536P, 4=2160P, 5=3072P
    AzureKinectEngine(const char *calibFilename = nullptr,
                      bool alignDepthToColor = true,
                      int depth_mode = 0,
                      int color_resolution = 0,
                      int fps = 30);
    ~AzureKinectEngine();

    bool hasMoreImages(void) const override;
    bool hasImagesNow(void) const override;
    void getImages(ITMUChar4Image *rgb, ITMShortImage *rawDepth) override;
    Vector2i getDepthImageSize(void) const override;
    Vector2i getRGBImageSize(void) const override;

    // IMU support
    bool hasIMU(void) const;
    bool getIMUSample(float *acc, float *gyro, uint64_t *timestamp_usec);

    // Get IMU orientation as 3x3 rotation matrix (for InfiniTAM)
    bool getIMUOrientation(Matrix3f &R);

    // Update IMU orientation (call each frame to integrate gyro)
    void updateIMU();
};

}
