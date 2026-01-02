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
 *   --no-display           Disable live visualization windows
 */

#include <iostream>
#include <csignal>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <cmath>

#ifdef _WIN32
#include <windows.h>
#endif

#include <yaml-cpp/yaml.h>
#include <torch/torch.h>
#include <opencv2/opencv.hpp>

#include "InputSource/AzureKinectEngine.h"
#include "InputSource/ImageSourceEngine.h"
#include "ITMLib/Core/ITMBasicEngine.h"
#include "ITMLib/Utils/ITMLibSettings.h"
#include "ITMLib/Objects/Tracking/ITMTrackingState.h"
#include "ITMLib/Objects/Misc/ITMIMUMeasurement.h"

#include "TsdfFusion/CLIEngine.h"
#include "InfiniTAM_tools.h"
#include "slam_pipeline.h"  // includes slam_gs_model.h

using namespace InputSource;
using namespace ITMLib;
using namespace InfiniTAM::Engine;

// Global flag for graceful shutdown
std::atomic<bool> g_shouldStop(false);

#ifdef _WIN32
// Windows console control handler
BOOL WINAPI consoleHandler(DWORD signal) {
    if (signal == CTRL_C_EVENT || signal == CTRL_BREAK_EVENT || signal == CTRL_CLOSE_EVENT) {
        std::cout << "\nInterrupt signal received. Stopping...\n";
        g_shouldStop = true;
        return TRUE;
    }
    return FALSE;
}
#else
void signalHandler(int signum)
{
    std::cout << "\nInterrupt signal (" << signum << ") received. Stopping...\n";
    g_shouldStop = true;
}
#endif

void printUsage(const char* progName)
{
    std::cout << "Usage: " << progName << " [config.yaml] [options]\n\n"
              << "Options:\n"
              << "  --depth-mode <0-3>   Depth mode (default: 2=WFOV_UNBINNED)\n"
              << "  --color-res <0-5>    Color resolution (default: 1=1080P)\n"
              << "  --fps <5|15|30>      Frame rate (default: 30)\n"
              << "  --no-align           Don't align depth to color\n"
              << "  --no-imu             Disable IMU tracking\n"
              << "  --max-frames <N>     Stop after N frames (0 = unlimited)\n"
              << "  --output <dir>       Output directory\n"
              << "  --no-display         Disable live visualization\n"
              << "  --help               Show this help\n";
}

int main(int argc, char *argv[])
{
    // Register signal handler for graceful shutdown
#ifdef _WIN32
    SetConsoleCtrlHandler(consoleHandler, TRUE);
#else
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
#endif

    // Default parameters
    std::string configFile = "";
    std::string outputDir = "output_online";
    int depthMode = 2;      // WFOV_UNBINNED (1024x1024)
    int colorRes = 1;       // 1080P (1920x1080)
    int fps = 30;
    bool alignDepth = true;
    bool enableIMU = true;  // Use IMU if available
    int maxFrames = 0;      // 0 = unlimited
    bool showDisplay = true; // Show live visualization

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
        else if (arg == "--no-imu")
        {
            enableIMU = false;
        }
        else if (arg == "--no-display")
        {
            showDisplay = false;
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

    // Auto-adjust FPS for depth modes that don't support 30 FPS
    // WFOV_UNBINNED (mode 2) only supports 15 FPS max
    if (depthMode == 2 && fps > 15) {
        std::cout << "Note: WFOV_UNBINNED only supports 15 FPS, adjusting...\n";
        fps = 15;
    }

    std::cout << "=== GPS-SLAM Online Mode ===\n";
    std::cout << "Depth mode: " << depthMode << "\n";
    std::cout << "Color resolution: " << colorRes << "\n";
    std::cout << "FPS: " << fps << "\n";
    std::cout << "Align depth to color: " << (alignDepth ? "yes" : "no") << "\n";
    std::cout << "Max frames: " << (maxFrames == 0 ? "unlimited" : std::to_string(maxFrames)) << "\n";
    std::cout << "Output directory: " << outputDir << "\n";
    std::cout << "Live display: " << (showDisplay ? "enabled" : "disabled") << "\n";

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

    // Check IMU availability
    bool useIMU = enableIMU && kinect->hasIMU();
    std::cout << "  IMU: " << (kinect->hasIMU() ? "available" : "not available");
    if (kinect->hasIMU() && !enableIMU)
        std::cout << " (disabled via --no-imu)";
    std::cout << "\n";

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

    // Configure tracker based on IMU availability
    if (useIMU)
    {
        // Use extended IMU tracker for better tracking with IMU data
        // This fuses ICP depth tracking with IMU orientation prior
        settings->trackerConfig = "type=extendedimu,levels=rrbb,useDepth=1,minstep=1e-4,"
                                  "outlierSpaceC=0.1,outlierSpaceF=0.004,"
                                  "numiterC=20,numiterF=50,tukeyCutOff=8,"
                                  "framesToSkip=20,framesToWeight=50,failureDec=20.0";
        std::cout << "Using Extended IMU tracker (IMU + Depth)\n";
    }
    else
    {
        // Standard extended depth tracker without IMU
        settings->trackerConfig = "type=extended,levels=rrbb,useDepth=1,minstep=1e-4,"
                                  "outlierSpaceC=0.1,outlierSpaceF=0.004,"
                                  "numiterC=20,numiterF=50,tukeyCutOff=8,"
                                  "framesToSkip=20,framesToWeight=50,failureDec=20.0";
        std::cout << "Using Extended Depth tracker (no IMU)\n";
    }

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
    std::cout << "Controls:\n";
    std::cout << "  WASD         - Move free camera\n";
    std::cout << "  E/X          - Move up/down\n";
    std::cout << "  IJKL/Arrows  - Look around\n";
    std::cout << "  'r'          - Reset tracking (if lost)\n";
    std::cout << "  'c'          - Clear volume and restart\n";
    std::cout << "  'q'/ESC      - Quit and save mesh\n\n";

    // Create display windows if enabled
    ITMUChar4Image *raycastImage = nullptr;
    ITMUChar4Image *freeViewImage = nullptr;
    ORUtils::SE3Pose *freeViewPose = nullptr;

    // Free camera state (FPS-style)
    float freeCamX = 0.0f, freeCamY = 1.0f, freeCamZ = 2.0f;  // Position
    float freeCamYaw = 0.0f;    // Left/right rotation (radians)
    float freeCamPitch = -0.3f; // Up/down rotation (radians)
    float moveSpeed = 0.1f;
    float rotSpeed = 0.05f;

    // Mouse tracking
    int lastMouseX = -1, lastMouseY = -1;
    bool mouseControlEnabled = false;

    if (showDisplay)
    {
        cv::namedWindow("RGB", cv::WINDOW_NORMAL);
        cv::namedWindow("Depth", cv::WINDOW_NORMAL);
        cv::namedWindow("3D Reconstruction", cv::WINDOW_NORMAL);
        cv::namedWindow("Free View (arrows to orbit)", cv::WINDOW_NORMAL);
        cv::resizeWindow("RGB", 640, 360);
        cv::resizeWindow("Depth", 640, 360);
        cv::resizeWindow("3D Reconstruction", 640, 360);
        cv::resizeWindow("Free View (arrows to orbit)", 640, 360);

        // Allocate images for visualization
        raycastImage = new ITMUChar4Image(depthSize, true, false);
        freeViewImage = new ITMUChar4Image(depthSize, true, false);
        freeViewPose = new ORUtils::SE3Pose();
    }

    int frameCount = 0;
    auto startTime = std::chrono::high_resolution_clock::now();

    // Allocate input images for direct processing (with IMU support)
    ITMUChar4Image *inputRGBImage = new ITMUChar4Image(rgbSize, true, true);
    ITMShortImage *inputRawDepthImage = new ITMShortImage(depthSize, true, true);
    ITMIMUMeasurement *imuMeasurement = useIMU ? new ITMIMUMeasurement() : nullptr;

    // Main SLAM loop
    while (!g_shouldStop && kinect->hasMoreImages())
    {
        if (maxFrames > 0 && frameCount >= maxFrames)
        {
            std::cout << "Reached max frames limit.\n";
            break;
        }

        // Get images from sensor
        kinect->getImages(inputRGBImage, inputRawDepthImage);

        // Check if we got valid images (frame drop protection)
        if (!kinect->hasImagesNow())
        {
            // Frame dropped, skip processing but don't fail
            continue;
        }

        // Update IMU orientation (integrate gyroscope)
        if (useIMU)
        {
            kinect->updateIMU();
            Matrix3f R;
            if (kinect->getIMUOrientation(R))
            {
                imuMeasurement->R = R;
            }
        }

        // Process frame with IMU (TSDF fusion + tracking)
        mainEngine->ProcessFrame(inputRGBImage, inputRawDepthImage, imuMeasurement);

        frameCount++;

        // Get current pose and tracking status
        ITMTrackingState *trackingState = mainEngine->GetTrackingState();
        ORUtils::SE3Pose *pose = trackingState->pose_d;
        ITMTrackingState::TrackingResult trackingResult = trackingState->trackerResult;

        // Get tracking status string
        const char* trackingStatus = "GOOD";
        if (trackingResult == ITMTrackingState::TRACKING_POOR) trackingStatus = "POOR";
        else if (trackingResult == ITMTrackingState::TRACKING_FAILED) trackingStatus = "LOST";

        // Print progress every 30 frames
        if (frameCount % 30 == 0)
        {
            auto now = std::chrono::high_resolution_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();
            float avgFps = (float)frameCount / (elapsed / 1000.0f);

            Matrix4f M = pose->GetM();
            std::cout << "Frame " << frameCount
                      << " | FPS: " << std::fixed << std::setprecision(1) << avgFps
                      << " | Tracking: " << trackingStatus
                      << " | Pose: [" << M.m30 << ", " << M.m31 << ", " << M.m32 << "]\n";
        }

        // TODO: Add Gaussian splatting optimization at keyframes
        // if (frameCount % config["PIPE"]["local_opt_interval"].as<int>() == 0)
        // {
        //     pipeline.localOptimization(gsModel, ...);
        // }

        // Live visualization
        if (showDisplay)
        {
            if (inputRGBImage && inputRawDepthImage)
            {
                // Convert RGB (RGBA -> BGR for OpenCV)
                cv::Mat rgbMat(rgbSize.y, rgbSize.x, CV_8UC4, inputRGBImage->GetData(MEMORYDEVICE_CPU));
                cv::Mat bgrMat;
                cv::cvtColor(rgbMat, bgrMat, cv::COLOR_RGBA2BGR);

                // Convert depth to colorized visualization
                cv::Mat depthMat(depthSize.y, depthSize.x, CV_16UC1, inputRawDepthImage->GetData(MEMORYDEVICE_CPU));
                cv::Mat depthVis;
                depthMat.convertTo(depthVis, CV_8UC1, 255.0 / 5000.0);
                cv::applyColorMap(depthVis, depthVis, cv::COLORMAP_JET);

                // Get 3D reconstruction raycast (TSDF volume rendered from current viewpoint)
                mainEngine->GetImage(raycastImage, ITMMainEngine::InfiniTAM_IMAGE_COLOUR_FROM_VOLUME);
                cv::Mat raycastMat(depthSize.y, depthSize.x, CV_8UC4, raycastImage->GetData(MEMORYDEVICE_CPU));
                cv::Mat raycastBgr;
                cv::cvtColor(raycastMat, raycastBgr, cv::COLOR_RGBA2BGR);

                // Render FREE VIEW - FPS-style camera
                // Calculate forward/right vectors from yaw/pitch
                float cosYaw = cos(freeCamYaw), sinYaw = sin(freeCamYaw);
                float cosPitch = cos(freeCamPitch), sinPitch = sin(freeCamPitch);

                // Forward vector (where camera looks)
                Vector3f forward(-sinYaw * cosPitch, -sinPitch, -cosYaw * cosPitch);
                Vector3f right(cosYaw, 0.0f, -sinYaw);
                Vector3f up = ORUtils::cross(right, forward);

                // Build rotation matrix (camera orientation)
                Matrix3f R;
                R.m00 = right.x;   R.m10 = right.y;   R.m20 = right.z;
                R.m01 = up.x;      R.m11 = up.y;      R.m21 = up.z;
                R.m02 = -forward.x; R.m12 = -forward.y; R.m22 = -forward.z;

                Vector3f camPos(freeCamX, freeCamY, freeCamZ);
                freeViewPose->SetR(R);
                freeViewPose->SetT(camPos);

                mainEngine->GetImage(freeViewImage, ITMMainEngine::InfiniTAM_IMAGE_FREECAMERA_COLOUR_FROM_VOLUME,
                                    freeViewPose, &calib.intrinsics_d);
                cv::Mat freeViewMat(depthSize.y, depthSize.x, CV_8UC4, freeViewImage->GetData(MEMORYDEVICE_CPU));
                cv::Mat freeViewBgr;
                cv::cvtColor(freeViewMat, freeViewBgr, cv::COLOR_RGBA2BGR);

                // Show controls help
                cv::putText(freeViewBgr, "WASD=move QE=up/down Mouse=look", cv::Point(10, 30),
                           cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                cv::putText(freeViewBgr, cv::format("Pos: [%.1f, %.1f, %.1f]", freeCamX, freeCamY, freeCamZ),
                           cv::Point(10, 55), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);

                // Add frame info overlay with tracking status color
                cv::Scalar statusColor;
                if (trackingResult == ITMTrackingState::TRACKING_GOOD)
                    statusColor = cv::Scalar(0, 255, 0);  // Green
                else if (trackingResult == ITMTrackingState::TRACKING_POOR)
                    statusColor = cv::Scalar(0, 255, 255);  // Yellow
                else
                    statusColor = cv::Scalar(0, 0, 255);  // Red

                Matrix4f M = pose->GetM();
                std::string info = cv::format("Frame: %d | %s", frameCount, trackingStatus);
                std::string poseInfo = cv::format("Pose: [%.2f, %.2f, %.2f]", M.m30, M.m31, M.m32);
                cv::putText(bgrMat, info, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, statusColor, 2);
                cv::putText(bgrMat, poseInfo, cv::Point(10, 60), cv::FONT_HERSHEY_SIMPLEX, 0.6, statusColor, 2);

                std::string reconInfo = cv::format("3D Reconstruction | %s", trackingStatus);
                cv::putText(raycastBgr, reconInfo, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6, statusColor, 2);

                cv::imshow("RGB", bgrMat);
                cv::imshow("Depth", depthVis);
                cv::imshow("3D Reconstruction", raycastBgr);
                cv::imshow("Free View (arrows to orbit)", freeViewBgr);
            }

            // Handle keyboard input
            int key = cv::waitKey(1);
            if (key == 'q' || key == 'Q' || key == 27)  // 27 = ESC
            {
                std::cout << "\nQuit requested via keyboard.\n";
                g_shouldStop = true;
            }
            else if (key == 'r' || key == 'R')
            {
                // Reset tracking pose to identity
                std::cout << "\n*** Resetting tracking pose ***\n";
                trackingState->Reset();
                std::cout << "Tracking reset. Move camera slowly to re-acquire.\n";
            }
            else if (key == 'c' || key == 'C')
            {
                // Full reset - clear TSDF volume and tracking
                std::cout << "\n*** Clearing volume and resetting ***\n";
                auto* basicEngine = dynamic_cast<ITMBasicEngine<ITMVoxel, ITMVoxelIndex>*>(mainEngine);
                if (basicEngine) {
                    basicEngine->resetAll();
                    std::cout << "Volume cleared. Starting fresh scan.\n";
                }
            }
            // WASD + QE for free camera movement
            float cosYaw = cos(freeCamYaw), sinYaw = sin(freeCamYaw);
            if (key == 'w' || key == 'W')  // Forward
            {
                freeCamX -= sinYaw * moveSpeed;
                freeCamZ -= cosYaw * moveSpeed;
            }
            else if (key == 's' || key == 'S')  // Backward
            {
                freeCamX += sinYaw * moveSpeed;
                freeCamZ += cosYaw * moveSpeed;
            }
            else if (key == 'a' || key == 'A')  // Strafe left
            {
                freeCamX -= cosYaw * moveSpeed;
                freeCamZ += sinYaw * moveSpeed;
            }
            else if (key == 'd' || key == 'D')  // Strafe right
            {
                freeCamX += cosYaw * moveSpeed;
                freeCamZ -= sinYaw * moveSpeed;
            }
            else if (key == 'e' || key == 'E')  // Up
            {
                freeCamY += moveSpeed;
            }
            else if (key == 'x' || key == 'X')  // Down
            {
                freeCamY -= moveSpeed;
            }
            // Arrow keys for rotation
            else if (key == 2424832 || key == 81 || key == 'j' || key == 'J')  // Left - turn left
            {
                freeCamYaw -= rotSpeed;
            }
            else if (key == 2555904 || key == 83 || key == 'l' || key == 'L')  // Right - turn right
            {
                freeCamYaw += rotSpeed;
            }
            else if (key == 2490368 || key == 82 || key == 'i' || key == 'I')  // Up - look up
            {
                freeCamPitch = std::max(-1.5f, freeCamPitch - rotSpeed);
            }
            else if (key == 2621440 || key == 84 || key == 'k' || key == 'K')  // Down - look down
            {
                freeCamPitch = std::min(1.5f, freeCamPitch + rotSpeed);
            }
        }
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
    if (showDisplay)
    {
        cv::destroyAllWindows();
        if (raycastImage) delete raycastImage;
        if (freeViewImage) delete freeViewImage;
        if (freeViewPose) delete freeViewPose;
    }
    if (inputRGBImage) delete inputRGBImage;
    if (inputRawDepthImage) delete inputRawDepthImage;
    if (imuMeasurement) delete imuMeasurement;
    cliEngine->Shutdown();
    delete mainEngine;
    delete settings;
    delete kinect;

    std::cout << "Done!\n";
    return 0;
}
