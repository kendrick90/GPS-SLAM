#include "TsdfFusion/CLIEngine.h"

#include <string.h>

#include "ORUtils/FileUtils.h"

using namespace InfiniTAM::Engine;
using namespace InputSource;
using namespace ITMLib;

CLIEngine *CLIEngine::instance;

// Offline mode initialization (pre-loaded images)
void CLIEngine::Initialise(std::vector<ITMUChar4Image *> rgb_images,
                           std::vector<ITMShortImage *> depth_images,
                           ITMMainEngine *mainEngine)
{
    this->rgb_images = rgb_images;
    this->depth_images = depth_images;
    this->mainEngine = mainEngine;
    this->currentFrameNo = 0;
    this->isLiveMode = false;
    this->shouldStop = false;
    this->imageSource = nullptr;

    bool allocateGPU = true;

    inputRGBImage = new ITMUChar4Image(GetRGBSize(), true, allocateGPU);
    inputRawDepthImage = new ITMShortImage(GetDepthSize(), true, allocateGPU);

    sdkCreateTimer(&timer_instant);
    sdkCreateTimer(&timer_average);
    sdkResetTimer(&timer_average);

    printf("CLIEngine initialised (offline mode, %zu frames).\n", rgb_images.size());
}

// Online/Live mode initialization (streaming from sensor)
void CLIEngine::InitialiseLive(ImageSourceEngine *imageSource,
                               ITMMainEngine *mainEngine)
{
    this->imageSource = imageSource;
    this->mainEngine = mainEngine;
    this->currentFrameNo = 0;
    this->isLiveMode = true;
    this->shouldStop = false;

    bool allocateGPU = true;

    Vector2i rgbSize = imageSource->getRGBImageSize();
    Vector2i depthSize = imageSource->getDepthImageSize();

    inputRGBImage = new ITMUChar4Image(rgbSize, true, allocateGPU);
    inputRawDepthImage = new ITMShortImage(depthSize, true, allocateGPU);

    sdkCreateTimer(&timer_instant);
    sdkCreateTimer(&timer_average);
    sdkResetTimer(&timer_average);

    printf("CLIEngine initialised (live mode).\n");
    printf("  RGB size: %dx%d\n", rgbSize.x, rgbSize.y);
    printf("  Depth size: %dx%d\n", depthSize.x, depthSize.y);
}

// Offline mode: process pre-loaded frame
bool CLIEngine::ProcessFrame()
{
    if (currentFrameNo >= rgb_images.size())
        return false;
    inputRGBImage = rgb_images[currentFrameNo];
    inputRawDepthImage = depth_images[currentFrameNo];

    sdkResetTimer(&timer_instant);
    sdkStartTimer(&timer_instant);
    sdkStartTimer(&timer_average);

    mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage);

    sdkStopTimer(&timer_instant);
    sdkStopTimer(&timer_average);

    float processedTime_inst = sdkGetTimerValue(&timer_instant);
    float processedTime_avg = sdkGetAverageTimerValue(&timer_average);

    currentFrameNo++;

    return true;
}

// Online/Live mode: get frame from sensor and process
bool CLIEngine::ProcessLiveFrame()
{
    if (!imageSource || !imageSource->hasMoreImages())
        return false;

    // Get images from the sensor
    imageSource->getImages(inputRGBImage, inputRawDepthImage);

    // Check if we got valid images (frame drop protection)
    if (!imageSource->hasImagesNow())
    {
        // Frame dropped, skip processing but don't fail
        return true;
    }

    sdkResetTimer(&timer_instant);
    sdkStartTimer(&timer_instant);
    sdkStartTimer(&timer_average);

    // Process the frame through TSDF fusion
    mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage);

    sdkStopTimer(&timer_instant);
    sdkStopTimer(&timer_average);

    processedTime = sdkGetTimerValue(&timer_instant);
    float processedTime_avg = sdkGetAverageTimerValue(&timer_average);

    currentFrameNo++;

    return true;
}

// Offline mode run loop
void CLIEngine::Run()
{
    while (true)
    {
        if (!ProcessFrame())
            break;
    }
}

// Online/Live mode run loop
void CLIEngine::RunLive(std::function<void(int, float)> frameCallback)
{
    printf("Starting live SLAM loop...\n");
    shouldStop = false;

    while (!shouldStop && imageSource && imageSource->hasMoreImages())
    {
        if (!ProcessLiveFrame())
            break;

        // Call the callback if provided (for Gaussian optimization, visualization, etc.)
        if (frameCallback)
        {
            frameCallback(currentFrameNo, processedTime);
        }

        // Print progress every 30 frames
        if (currentFrameNo % 30 == 0)
        {
            float avg_time = sdkGetAverageTimerValue(&timer_average);
            printf("Frame %d: %.2f ms (avg: %.2f ms, %.1f FPS)\n",
                   currentFrameNo, processedTime, avg_time, 1000.0f / avg_time);
        }
    }

    printf("Live SLAM loop ended after %d frames.\n", currentFrameNo);
}

void CLIEngine::Shutdown()
{
    sdkDeleteTimer(&timer_instant);
    sdkDeleteTimer(&timer_average);

    if (inputRGBImage) delete inputRGBImage;
    if (inputRawDepthImage) delete inputRawDepthImage;

    inputRGBImage = nullptr;
    inputRawDepthImage = nullptr;

    // Note: imageSource is owned by caller, don't delete here
    // delete instance; // Don't delete singleton in Shutdown
}
