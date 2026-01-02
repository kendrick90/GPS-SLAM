#pragma once

#include <iostream>
#include <functional>
#include "InputSource/ImageSourceEngine.h"
#include "ITMLib/Core/ITMMainEngine.h"
#include "ITMLib/Utils/ITMLibSettings.h"
#include "ORUtils/FileUtils.h"
#include "ORUtils/NVTimer.h"

namespace InfiniTAM
{
    namespace Engine
    {
        class CLIEngine
        {
            static CLIEngine *instance;

            // For offline mode (pre-loaded images)
            std::vector<ITMUChar4Image *> rgb_images;
            std::vector<ITMShortImage *> depth_images;

            // For online mode (live streaming)
            InputSource::ImageSourceEngine *imageSource;
            bool isLiveMode;

            ITMLib::ITMLibSettings internalSettings;
            ITMLib::ITMMainEngine *mainEngine;

            StopWatchInterface *timer_instant;
            StopWatchInterface *timer_average;

        private:
            ITMUChar4Image *inputRGBImage;
            ITMShortImage *inputRawDepthImage;

            int currentFrameNo;
            bool shouldStop;

        public:
            static CLIEngine *Instance(void)
            {
                if (instance == NULL)
                    instance = new CLIEngine();
                return instance;
            }

            float processedTime;

            // Offline mode initialization (pre-loaded images)
            void Initialise(std::vector<ITMUChar4Image *> rgb_images,
                            std::vector<ITMShortImage *> depth_images,
                            ITMLib::ITMMainEngine *mainEngine);

            // Online/Live mode initialization (streaming from sensor)
            void InitialiseLive(InputSource::ImageSourceEngine *imageSource,
                               ITMLib::ITMMainEngine *mainEngine);

            void Shutdown();

            // Offline mode
            void Run();
            bool ProcessFrame();

            // Online/Live mode
            void RunLive(std::function<void(int, float)> frameCallback = nullptr);
            bool ProcessLiveFrame();
            void RequestStop() { shouldStop = true; }
            bool IsLiveMode() const { return isLiveMode; }

            Vector2i GetDepthSize()
            {
                if (isLiveMode && imageSource)
                    return imageSource->getDepthImageSize();
                return depth_images.empty() ? Vector2i(0,0) : depth_images[0]->noDims;
            }

            Vector2i GetRGBSize()
            {
                if (isLiveMode && imageSource)
                    return imageSource->getRGBImageSize();
                return rgb_images.empty() ? Vector2i(0,0) : rgb_images[0]->noDims;
            }

            ITMLib::ITMMainEngine *getMainEngine()
            {
                return mainEngine;
            }

            int GetCurrentFrameNo() const { return currentFrameNo; }

            // Access to current frame images for visualization
            ITMUChar4Image* GetCurrentRGBImage() const { return inputRGBImage; }
            ITMShortImage* GetCurrentDepthImage() const { return inputRawDepthImage; }
        };
    }
}
