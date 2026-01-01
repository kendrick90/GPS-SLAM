// Copyright 2025 GPS-SLAM Team
// Azure Kinect Engine for InfiniTAM/GPS-SLAM

#include "AzureKinectEngine.h"
#include "../ORUtils/FileUtils.h"

#include <cstdio>
#include <cstring>
#include <iostream>

#ifdef COMPILE_WITH_AzureKinect

using namespace InputSource;
using namespace ITMLib;

// Helper to convert depth mode enum
static k4a_depth_mode_t getDepthMode(int mode) {
    switch (mode) {
        case 0: return K4A_DEPTH_MODE_NFOV_UNBINNED;  // 640x576
        case 1: return K4A_DEPTH_MODE_NFOV_2X2BINNED; // 320x288
        case 2: return K4A_DEPTH_MODE_WFOV_UNBINNED;  // 1024x1024
        case 3: return K4A_DEPTH_MODE_WFOV_2X2BINNED; // 512x512
        default: return K4A_DEPTH_MODE_NFOV_UNBINNED;
    }
}

// Helper to convert color resolution enum
static k4a_color_resolution_t getColorResolution(int res) {
    switch (res) {
        case 0: return K4A_COLOR_RESOLUTION_720P;   // 1280x720
        case 1: return K4A_COLOR_RESOLUTION_1080P;  // 1920x1080
        case 2: return K4A_COLOR_RESOLUTION_1440P;  // 2560x1440
        case 3: return K4A_COLOR_RESOLUTION_1536P;  // 2048x1536
        case 4: return K4A_COLOR_RESOLUTION_2160P;  // 3840x2160
        case 5: return K4A_COLOR_RESOLUTION_3072P;  // 4096x3072
        default: return K4A_COLOR_RESOLUTION_720P;
    }
}

// Helper to convert FPS
static k4a_fps_t getFPS(int fps) {
    if (fps <= 5) return K4A_FRAMES_PER_SECOND_5;
    if (fps <= 15) return K4A_FRAMES_PER_SECOND_15;
    return K4A_FRAMES_PER_SECOND_30;
}

// Get image dimensions for depth mode
static Vector2i getDepthDimensions(k4a_depth_mode_t mode) {
    switch (mode) {
        case K4A_DEPTH_MODE_NFOV_UNBINNED:  return Vector2i(640, 576);
        case K4A_DEPTH_MODE_NFOV_2X2BINNED: return Vector2i(320, 288);
        case K4A_DEPTH_MODE_WFOV_UNBINNED:  return Vector2i(1024, 1024);
        case K4A_DEPTH_MODE_WFOV_2X2BINNED: return Vector2i(512, 512);
        default: return Vector2i(640, 576);
    }
}

// Get image dimensions for color resolution
static Vector2i getColorDimensions(k4a_color_resolution_t res) {
    switch (res) {
        case K4A_COLOR_RESOLUTION_720P:  return Vector2i(1280, 720);
        case K4A_COLOR_RESOLUTION_1080P: return Vector2i(1920, 1080);
        case K4A_COLOR_RESOLUTION_1440P: return Vector2i(2560, 1440);
        case K4A_COLOR_RESOLUTION_1536P: return Vector2i(2048, 1536);
        case K4A_COLOR_RESOLUTION_2160P: return Vector2i(3840, 2160);
        case K4A_COLOR_RESOLUTION_3072P: return Vector2i(4096, 3072);
        default: return Vector2i(1280, 720);
    }
}

AzureKinectEngine::AzureKinectEngine(const char *calibFilename,
                                     bool alignDepthToColor,
                                     int depth_mode,
                                     int color_resolution,
                                     int fps)
    : BaseImageSourceEngine(calibFilename),
      device(nullptr),
      transformation(nullptr),
      alignDepthToColor(alignDepthToColor)
{
    dataAvailable = false;

    // Check for connected devices
    uint32_t device_count = k4a_device_get_installed_count();
    printf("Found %d Azure Kinect device(s)\n", device_count);

    if (device_count == 0) {
        printf("No Azure Kinect devices found!\n");
        return;
    }

    // Open the first device
    if (k4a_device_open(K4A_DEVICE_DEFAULT, &device) != K4A_RESULT_SUCCEEDED) {
        printf("Failed to open Azure Kinect device\n");
        device = nullptr;
        return;
    }

    // Print device info
    char serial_number[256];
    size_t serial_size = sizeof(serial_number);
    if (k4a_device_get_serialnum(device, serial_number, &serial_size) == K4A_BUFFER_RESULT_SUCCEEDED) {
        printf("Azure Kinect Serial: %s\n", serial_number);
    }

    // Configure the device
    config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
    config.color_format = K4A_IMAGE_FORMAT_COLOR_BGRA32;
    config.color_resolution = getColorResolution(color_resolution);
    config.depth_mode = getDepthMode(depth_mode);
    config.camera_fps = getFPS(fps);
    config.synchronized_images_only = false;  // Allow unsynchronized for WSL2 compatibility

    // Get image dimensions
    imageSize_d = getDepthDimensions(config.depth_mode);
    imageSize_rgb = getColorDimensions(config.color_resolution);

    printf("Azure Kinect Config:\n");
    printf("  Depth: %dx%d\n", imageSize_d.x, imageSize_d.y);
    printf("  Color: %dx%d\n", imageSize_rgb.x, imageSize_rgb.y);
    printf("  FPS: %d\n", fps);
    printf("  Align depth to color: %s\n", alignDepthToColor ? "yes" : "no");

    // Start cameras
    if (k4a_device_start_cameras(device, &config) != K4A_RESULT_SUCCEEDED) {
        printf("Failed to start Azure Kinect cameras\n");
        k4a_device_close(device);
        device = nullptr;
        return;
    }

    // Get calibration for depth-color alignment
    if (k4a_device_get_calibration(device, config.depth_mode, config.color_resolution, &calibration)
        != K4A_RESULT_SUCCEEDED) {
        printf("Failed to get calibration\n");
        k4a_device_stop_cameras(device);
        k4a_device_close(device);
        device = nullptr;
        return;
    }

    // Create transformation handle for depth-to-color alignment
    if (alignDepthToColor) {
        transformation = k4a_transformation_create(&calibration);
        // When aligning, depth output size matches color size
        imageSize_d = imageSize_rgb;
    }

    // Set up InfiniTAM calibration from Azure Kinect intrinsics
    k4a_calibration_intrinsic_parameters_t *depth_intrinsics =
        &calibration.depth_camera_calibration.intrinsics.parameters;
    k4a_calibration_intrinsic_parameters_t *color_intrinsics =
        &calibration.color_camera_calibration.intrinsics.parameters;

    if (alignDepthToColor) {
        // Use color intrinsics for both when aligned
        this->calib.intrinsics_d.projectionParamsSimple.fx = color_intrinsics->param.fx;
        this->calib.intrinsics_d.projectionParamsSimple.fy = color_intrinsics->param.fy;
        this->calib.intrinsics_d.projectionParamsSimple.px = color_intrinsics->param.cx;
        this->calib.intrinsics_d.projectionParamsSimple.py = color_intrinsics->param.cy;
    } else {
        this->calib.intrinsics_d.projectionParamsSimple.fx = depth_intrinsics->param.fx;
        this->calib.intrinsics_d.projectionParamsSimple.fy = depth_intrinsics->param.fy;
        this->calib.intrinsics_d.projectionParamsSimple.px = depth_intrinsics->param.cx;
        this->calib.intrinsics_d.projectionParamsSimple.py = depth_intrinsics->param.cy;
    }

    this->calib.intrinsics_rgb.projectionParamsSimple.fx = color_intrinsics->param.fx;
    this->calib.intrinsics_rgb.projectionParamsSimple.fy = color_intrinsics->param.fy;
    this->calib.intrinsics_rgb.projectionParamsSimple.px = color_intrinsics->param.cx;
    this->calib.intrinsics_rgb.projectionParamsSimple.py = color_intrinsics->param.cy;

    // Set disparity calibration (depth scale)
    // Azure Kinect depth is in millimeters, scale to meters
    this->calib.disparityCalib.SetFrom(0.001f, 0.0f, ITMLib::ITMDisparityCalib::TRAFO_AFFINE);

    // Identity extrinsics when aligned
    this->calib.trafo_rgb_to_depth = ITMExtrinsics();

    printf("Azure Kinect initialized successfully!\n");
    printf("Depth intrinsics: fx=%.2f fy=%.2f cx=%.2f cy=%.2f\n",
           this->calib.intrinsics_d.projectionParamsSimple.fx,
           this->calib.intrinsics_d.projectionParamsSimple.fy,
           this->calib.intrinsics_d.projectionParamsSimple.px,
           this->calib.intrinsics_d.projectionParamsSimple.py);

    dataAvailable = true;
}

AzureKinectEngine::~AzureKinectEngine()
{
    if (transformation) {
        k4a_transformation_destroy(transformation);
    }
    if (device) {
        k4a_device_stop_cameras(device);
        k4a_device_close(device);
    }
}

void AzureKinectEngine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage)
{
    if (!device) {
        dataAvailable = false;
        return;
    }

    k4a_capture_t capture = nullptr;

    // Wait for a capture (timeout 1000ms)
    k4a_wait_result_t result = k4a_device_get_capture(device, &capture, 1000);

    if (result != K4A_WAIT_RESULT_SUCCEEDED) {
        printf("Failed to get capture from Azure Kinect\n");
        dataAvailable = false;
        return;
    }

    // Get color image
    k4a_image_t color_image = k4a_capture_get_color_image(capture);
    if (color_image == nullptr) {
        printf("Failed to get color image\n");
        k4a_capture_release(capture);
        dataAvailable = false;
        return;
    }

    // Get depth image
    k4a_image_t depth_image = k4a_capture_get_depth_image(capture);
    if (depth_image == nullptr) {
        printf("Failed to get depth image\n");
        k4a_image_release(color_image);
        k4a_capture_release(capture);
        dataAvailable = false;
        return;
    }

    // Copy color data (BGRA -> RGBA)
    Vector4u *rgb = rgbImage->GetData(MEMORYDEVICE_CPU);
    uint8_t *color_buffer = k4a_image_get_buffer(color_image);
    int color_width = k4a_image_get_width_pixels(color_image);
    int color_height = k4a_image_get_height_pixels(color_image);

    for (int i = 0; i < color_width * color_height; i++) {
        rgb[i].x = color_buffer[i * 4 + 2]; // R <- B
        rgb[i].y = color_buffer[i * 4 + 1]; // G <- G
        rgb[i].z = color_buffer[i * 4 + 0]; // B <- R
        rgb[i].w = color_buffer[i * 4 + 3]; // A <- A
    }

    // Handle depth - either aligned or raw
    short *rawDepth = rawDepthImage->GetData(MEMORYDEVICE_CPU);

    if (alignDepthToColor && transformation) {
        // Transform depth to color camera space
        k4a_image_t transformed_depth = nullptr;
        if (k4a_image_create(K4A_IMAGE_FORMAT_DEPTH16,
                            imageSize_rgb.x, imageSize_rgb.y,
                            imageSize_rgb.x * sizeof(uint16_t),
                            &transformed_depth) == K4A_RESULT_SUCCEEDED) {

            if (k4a_transformation_depth_image_to_color_camera(transformation,
                    depth_image, transformed_depth) == K4A_RESULT_SUCCEEDED) {

                uint16_t *depth_buffer = (uint16_t*)k4a_image_get_buffer(transformed_depth);
                std::memcpy(rawDepth, depth_buffer,
                           imageSize_rgb.x * imageSize_rgb.y * sizeof(uint16_t));
            }
            k4a_image_release(transformed_depth);
        }
    } else {
        // Use raw depth
        uint16_t *depth_buffer = (uint16_t*)k4a_image_get_buffer(depth_image);
        int depth_width = k4a_image_get_width_pixels(depth_image);
        int depth_height = k4a_image_get_height_pixels(depth_image);
        std::memcpy(rawDepth, depth_buffer, depth_width * depth_height * sizeof(uint16_t));
    }

    // Cleanup
    k4a_image_release(depth_image);
    k4a_image_release(color_image);
    k4a_capture_release(capture);

    dataAvailable = true;
}

bool AzureKinectEngine::hasMoreImages(void) const
{
    return device != nullptr;
}

Vector2i AzureKinectEngine::getDepthImageSize(void) const
{
    return device ? imageSize_d : Vector2i(0, 0);
}

Vector2i AzureKinectEngine::getRGBImageSize(void) const
{
    return device ? imageSize_rgb : Vector2i(0, 0);
}

bool AzureKinectEngine::hasIMU(void) const
{
    return device != nullptr;
}

bool AzureKinectEngine::getIMUSample(float *acc, float *gyro, uint64_t *timestamp_usec)
{
    if (!device) return false;

    k4a_imu_sample_t imu_sample;
    k4a_wait_result_t result = k4a_device_get_imu_sample(device, &imu_sample, 0);

    if (result != K4A_WAIT_RESULT_SUCCEEDED) {
        return false;
    }

    if (acc) {
        acc[0] = imu_sample.acc_sample.xyz.x;
        acc[1] = imu_sample.acc_sample.xyz.y;
        acc[2] = imu_sample.acc_sample.xyz.z;
    }
    if (gyro) {
        gyro[0] = imu_sample.gyro_sample.xyz.x;
        gyro[1] = imu_sample.gyro_sample.xyz.y;
        gyro[2] = imu_sample.gyro_sample.xyz.z;
    }
    if (timestamp_usec) {
        *timestamp_usec = imu_sample.acc_timestamp_usec;
    }

    return true;
}

#else

// Stub implementation when compiled without Azure Kinect support

using namespace InputSource;

AzureKinectEngine::AzureKinectEngine(const char *calibFilename,
                                     bool alignDepthToColor,
                                     int depth_mode,
                                     int color_resolution,
                                     int fps)
    : BaseImageSourceEngine(calibFilename)
{
    printf("Compiled without Azure Kinect SDK support\n");
    dataAvailable = false;
}

AzureKinectEngine::~AzureKinectEngine() {}

void AzureKinectEngine::getImages(ITMUChar4Image *rgbImage, ITMShortImage *rawDepthImage) {}

bool AzureKinectEngine::hasMoreImages(void) const { return false; }

Vector2i AzureKinectEngine::getDepthImageSize(void) const { return Vector2i(0, 0); }

Vector2i AzureKinectEngine::getRGBImageSize(void) const { return Vector2i(0, 0); }

bool AzureKinectEngine::hasIMU(void) const { return false; }

bool AzureKinectEngine::getIMUSample(float *acc, float *gyro, uint64_t *timestamp_usec) { return false; }

#endif
