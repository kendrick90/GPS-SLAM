/**
 * Online SLAM with Azure Kinect
 *
 * This executable runs GPS-SLAM in real-time using an Azure Kinect sensor.
 * It performs:
 *   1. TSDF fusion for geometry
 *   2. Incremental Gaussian splatting optimization
 *   3. Real-time pose tracking
 *
 * Usage:
 *   ./online_slam [config.yaml] [options]
 *
 * Options:
 *   --depth-mode <0-3>     Depth mode (0=NFOV_UNBINNED, 1=NFOV_2X2BINNED,
 *                                      2=WFOV_UNBINNED, 3=WFOV_2X2BINNED)
 *   --color-res <0-5>      Color resolution (0=720P, 1=1080P, 2=1440P,
 *                                            3=1536P, 4=2160P, 5=3072P)
 *   --fps <5|15|30>        Frame rate
 *   --no-align             Don't align depth to color
 *   --max-frames <N>       Stop after N frames (0 = unlimited)
 *   --output <dir>         Output directory for results
 */

#include <iostream>
#include <csignal>
#include <atomic>
#include <chrono>

#include <yaml-cpp/yaml.h>
#include <torch/torch.h>
#include <opencv2/opencv.hpp>

#include "InputSource/AzureKinectEngine.h"
#include "InputSource/ImageSourceEngine.h"
#include "ITMLib/Core/ITMBasicEngine.h"
#include "ITMLib/Utils/ITMLibSettings.h"

#include "TsdfFusion/CLIEngine.h"
#include "InfiniTAM_tools.h"
#include "slam_pipeline.h"  // includes slam_gs_model.h

using namespace InputSource;
using namespace ITMLib;
using namespace InfiniTAM::Engine;

// Global flag for graceful shutdown
std::atomic<bool> g_shouldStop(false);

void signalHandler(int signum)
{
    std::cout << "\nInterrupt signal (" << signum << ") received. Stopping...\n";
    g_shouldStop = true;
}

void printUsage(const char* progName)
{
    std::cout << "Usage: " << progName << " [config.yaml] [options]\n\n"
              << "Options:\n"
              << "  --depth-mode <0-3>   Depth mode (default: 0=NFOV_UNBINNED)\n"
              << "  --color-res <0-5>    Color resolution (default: 0=720P)\n"
              << "  --fps <5|15|30>      Frame rate (default: 30)\n"
              << "  --no-align           Don't align depth to color\n"
              << "  --max-frames <N>     Stop after N frames (0 = unlimited)\n"
              << "  --output <dir>       Output directory\n"
              << "  --help               Show this help\n";
}

int main(int argc, char *argv[])
{
    // Register signal handler for graceful shutdown
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    // Default parameters
    std::string configFile = "";
    std::string outputDir = "output_online";
    int depthMode = 0;      // NFOV_UNBINNED (640x576)
    int colorRes = 0;       // 720P (1280x720)
    int fps = 30;
    bool alignDepth = true;
    int maxFrames = 0;      // 0 = unlimited

    // Parse command line arguments
    for (int i = 1; i < argc; i++)
    {
        std::string arg = argv[i];
        if (arg == "--help" || arg == "-h")
        {
            printUsage(argv[0]);
            return 0;
        }
        else if (arg == "--depth-mode" && i + 1 < argc)
        {
            depthMode = std::stoi(argv[++i]);
        }
        else if (arg == "--color-res" && i + 1 < argc)
        {
            colorRes = std::stoi(argv[++i]);
        }
        else if (arg == "--fps" && i + 1 < argc)
        {
            fps = std::stoi(argv[++i]);
        }
        else if (arg == "--no-align")
        {
            alignDepth = false;
        }
        else if (arg == "--max-frames" && i + 1 < argc)
        {
            maxFrames = std::stoi(argv[++i]);
        }
        else if (arg == "--output" && i + 1 < argc)
        {
            outputDir = argv[++i];
        }
        else if (arg[0] != '-' && configFile.empty())
        {
            configFile = arg;
        }
    }

    std::cout << "=== GPS-SLAM Online Mode ===\n";
    std::cout << "Depth mode: " << depthMode << "\n";
    std::cout << "Color resolution: " << colorRes << "\n";
    std::cout << "FPS: " << fps << "\n";
    std::cout << "Align depth to color: " << (alignDepth ? "yes" : "no") << "\n";
    std::cout << "Max frames: " << (maxFrames == 0 ? "unlimited" : std::to_string(maxFrames)) << "\n";
    std::cout << "Output directory: " << outputDir << "\n";

    // Create output directory
    std::filesystem::create_directories(outputDir);

    // Initialize Azure Kinect
    std::cout << "\nInitializing Azure Kinect...\n";
    AzureKinectEngine *kinect = new AzureKinectEngine(
        nullptr,      // calibFilename (use sensor calibration)
        alignDepth,   // align depth to color
        depthMode,
        colorRes,
        fps
    );

    if (!kinect->hasMoreImages())
    {
        std::cerr << "ERROR: Failed to initialize Azure Kinect!\n";
        std::cerr << "Make sure the device is connected and USB is passed through.\n";
        delete kinect;
        return 1;
    }

    Vector2i rgbSize = kinect->getRGBImageSize();
    Vector2i depthSize = kinect->getDepthImageSize();
    ITMRGBDCalib calib = kinect->getCalib();

    std::cout << "Azure Kinect initialized successfully!\n";
    std::cout << "  RGB size: " << rgbSize.x << "x" << rgbSize.y << "\n";
    std::cout << "  Depth size: " << depthSize.x << "x" << depthSize.y << "\n";
    std::cout << "  Depth intrinsics: fx=" << calib.intrinsics_d.projectionParamsSimple.fx
              << " fy=" << calib.intrinsics_d.projectionParamsSimple.fy
              << " cx=" << calib.intrinsics_d.projectionParamsSimple.px
              << " cy=" << calib.intrinsics_d.projectionParamsSimple.py << "\n";

    // Load config if provided, otherwise use defaults
    YAML::Node config;
    if (!configFile.empty())
    {
        std::cout << "\nLoading config from: " << configFile << "\n";
        config = YAML::LoadFile(configFile);
    }
    else
    {
        std::cout << "\nUsing default configuration.\n";
        // Create default config
        config["work_mode"] = "train";
        config["PIPE"]["TSDF"]["voxel_size"] = 0.005;
        config["PIPE"]["TSDF"]["trunc_dist"] = 0.02;
        config["PIPE"]["TSDF"]["viewFrustum_min"] = 0.2;
        config["PIPE"]["TSDF"]["viewFrustum_max"] = 5.0;
        config["PIPE"]["TSDF"]["use_gt_pose"] = false;  // Enable tracking for online mode
        config["PIPE"]["local_opt_interval"] = 10;
    }

    // Override config for online mode
    config["PIPE"]["TSDF"]["use_gt_pose"] = false;  // Must track pose in online mode

    // Initialize InfiniTAM/TSDF engine
    std::cout << "\nInitializing TSDF engine...\n";

    ITMLibSettings *settings = new ITMLibSettings();
    settings->deviceType = ITMLibSettings::DEVICE_CUDA;
    settings->useBilateralFilter = true;
    settings->useApproximateRaycast = false;

    // Create main TSDF engine
    ITMMainEngine *mainEngine = new ITMBasicEngine<ITMVoxel, ITMVoxelIndex>(
        settings, calib, rgbSize, depthSize
    );

    // Initialize CLI engine for live mode
    CLIEngine *cliEngine = CLIEngine::Instance();
    cliEngine->InitialiseLive(kinect, mainEngine);

    // Initialize Gaussian splatting model (if doing full SLAM)
    // SLAMGaussianModel gsModel;
    // SLAMPipeline pipeline;
    // gsModel.loadConfig(config["MODEL"]);

    std::cout << "\n=== Starting Online SLAM ===\n";
    std::cout << "Press Ctrl+C to stop.\n\n";

    int frameCount = 0;
    auto startTime = std::chrono::high_resolution_clock::now();

    // Main SLAM loop
    while (!g_shouldStop && kinect->hasMoreImages())
    {
        if (maxFrames > 0 && frameCount >= maxFrames)
        {
            std::cout << "Reached max frames limit.\n";
            break;
        }

        // Process one frame (TSDF fusion + tracking)
        if (!cliEngine->ProcessLiveFrame())
        {
            std::cerr << "Failed to process frame.\n";
            break;
        }

        frameCount++;

        // Get current pose
        ORUtils::SE3Pose *pose = mainEngine->GetTrackingState()->pose_d;

        // Print progress every 30 frames
        if (frameCount % 30 == 0)
        {
            auto now = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();
            float avgFps = (float)frameCount / (elapsed / 1000.0f);

            Matrix4f M = pose->GetM();
            std::cout << "Frame " << frameCount
                      << " | FPS: " << std::fixed << std::setprecision(1) << avgFps
                      << " | Pose: [" << M.m30 << ", " << M.m31 << ", " << M.m32 << "]\n";
        }

        // TODO: Add Gaussian splatting optimization at keyframes
        // if (frameCount % config["PIPE"]["local_opt_interval"].as<int>() == 0)
        // {
        //     pipeline.localOptimization(gsModel, ...);
        // }
    }

    auto endTime = std::chrono::high_resolution_clock::now();
    auto totalTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime).count();

    std::cout << "\n=== Online SLAM Complete ===\n";
    std::cout << "Total frames: " << frameCount << "\n";
    std::cout << "Total time: " << totalTime / 1000.0f << " seconds\n";
    std::cout << "Average FPS: " << (float)frameCount / (totalTime / 1000.0f) << "\n";

    // Save results
    std::cout << "\nSaving results to " << outputDir << "...\n";

    // Save TSDF mesh
    std::string meshPath = outputDir + "/tsdf_mesh.ply";
    mainEngine->SaveSceneToMesh(meshPath.c_str());
    std::cout << "Saved mesh to: " << meshPath << "\n";

    // Cleanup
    std::cout << "\nShutting down...\n";
    cliEngine->Shutdown();
    delete mainEngine;
    delete settings;
    delete kinect;

    std::cout << "Done!\n";
    return 0;
}
