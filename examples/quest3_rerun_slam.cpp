// Quest 3 + Rerun SLAM Example
// Demonstrates using Meta Quest 3 as input source with Rerun visualization
//
// Usage:
//   # On-device (Quest 3):
//   adb shell am start -n com.sap.slam/.MainActivity
//
//   # Streaming to PC:
//   quest3_rerun_slam --stream --host 192.168.1.100 --port 8888
//
//   # With recording:
//   quest3_rerun_slam --stream --record output/session.rrd

#include <iostream>
#include <memory>
#include <chrono>
#include <csignal>

#include "sap/core/frame.h"
#include "sap/core/config.h"
#include "sap/interfaces/input_source.h"
#include "sap/interfaces/slam_backend.h"

#include "plugins/input/quest_3.h"
#include "plugins/viewers/rerun_viewer.h"
#include "plugins/viewers/rerun_slam_logger.h"

// For argument parsing
#include <cxxopts.hpp>

using namespace sap;
using namespace sap::plugins;

// Global flag for graceful shutdown
static volatile bool g_running = true;

void signalHandler(int signum) {
    std::cout << "\nShutting down..." << std::endl;
    g_running = false;
}

int main(int argc, char** argv) {
    // Signal handling for graceful shutdown
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // Parse command line arguments
    cxxopts::Options options("quest3_rerun_slam", "Quest 3 SLAM with Rerun visualization");
    options.add_options()
        ("c,config", "Config file path", cxxopts::value<std::string>()->default_value("configs/quest_3.yaml"))
        ("s,stream", "Enable streaming mode (PC-side processing)")
        ("host", "Quest 3 IP address for streaming", cxxopts::value<std::string>()->default_value("192.168.1.100"))
        ("port", "Streaming port", cxxopts::value<int>()->default_value("8888"))
        ("r,record", "Record to .rrd file", cxxopts::value<std::string>())
        ("remote-rerun", "Connect to remote Rerun viewer", cxxopts::value<std::string>())
        ("no-viewer", "Disable Rerun viewer (recording only)")
        ("h,help", "Print help");

    auto args = options.parse(argc, argv);

    if (args.count("help")) {
        std::cout << options.help() << std::endl;
        return 0;
    }

    // ========================================================================
    // Initialize Quest 3 Input Source
    // ========================================================================
    std::cout << "Initializing Quest 3 input source..." << std::endl;

    auto quest_input = std::make_unique<Quest3InputSource>();

    Quest3Config quest_config;
    quest_config.camera_mode = Quest3Camera::STEREO;
    quest_config.resolution = Quest3Resolution::RES_1280;
    quest_config.target_fps = 30;
    quest_config.depth_mode = Quest3DepthMode::ENVIRONMENTAL;
    quest_config.enable_imu = true;
    quest_config.enable_hand_tracking = true;
    quest_config.enable_eye_tracking = false;  // Requires permission

    if (args.count("stream")) {
        quest_config.connection = Quest3Config::ConnectionMode::CUSTOM_STREAM;
        quest_config.host_address = args["host"].as<std::string>();
        quest_config.port = args["port"].as<int>();
        std::cout << "Streaming mode: connecting to Quest at "
                  << quest_config.host_address << ":" << quest_config.port << std::endl;
    } else {
        quest_config.connection = Quest3Config::ConnectionMode::ON_DEVICE;
        std::cout << "On-device mode" << std::endl;
    }

    // Load YAML config and merge with command line
    Config config;
    config.load(args["config"].as<std::string>());

    if (!quest_input->initialize(config)) {
        std::cerr << "Failed to initialize Quest 3 input" << std::endl;
        return 1;
    }

    quest_input->setConfig(quest_config);

    // ========================================================================
    // Initialize Rerun Logger
    // ========================================================================
    std::cout << "Initializing Rerun visualization..." << std::endl;

    std::unique_ptr<RerunSLAMLogger> logger;

    if (args.count("record")) {
        // Recording mode
        std::string record_path = args["record"].as<std::string>();
        logger = createRecordingRerunLogger(record_path, "Quest3_SLAM");
        std::cout << "Recording to: " << record_path << std::endl;
    } else if (args.count("remote-rerun")) {
        // Remote viewer mode
        std::string remote = args["remote-rerun"].as<std::string>();
        auto colon_pos = remote.find(':');
        std::string host = remote.substr(0, colon_pos);
        int port = std::stoi(remote.substr(colon_pos + 1));
        logger = createRemoteRerunLogger(host, port, "Quest3_SLAM");
        std::cout << "Connecting to remote Rerun viewer at: " << remote << std::endl;
    } else if (!args.count("no-viewer")) {
        // Local viewer mode
        logger = createLocalRerunLogger("Quest3_SLAM");
        std::cout << "Spawning local Rerun viewer" << std::endl;
    }

    if (logger) {
        logger->setLogLevel(RerunLogLevel::STANDARD);
    }

    // ========================================================================
    // Initialize SLAM System
    // ========================================================================
    std::cout << "Initializing SLAM system..." << std::endl;

    // TODO: Create SLAM system from config
    // auto slam_system = createSLAMSystem(config);

    // For now, we'll just visualize the Quest 3 data without full SLAM
    bool slam_initialized = false;
    torch::Tensor current_pose = torch::eye(4, torch::kFloat32);

    // ========================================================================
    // Main Processing Loop
    // ========================================================================
    std::cout << "Starting capture loop..." << std::endl;

    if (!quest_input->startCapture()) {
        std::cerr << "Failed to start Quest 3 capture" << std::endl;
        return 1;
    }

    int64_t frame_count = 0;
    auto start_time = std::chrono::high_resolution_clock::now();
    auto last_stats_time = start_time;

    while (g_running && quest_input->hasMoreFrames()) {
        // Get next frame from Quest 3
        Frame frame = quest_input->getNextFrame();
        frame.frame_id = frame_count++;

        // Get Quest's own tracking pose (if available)
        torch::Tensor quest_pose = quest_input->getQuestPose();

        // Get hand and eye tracking data
        auto [left_hand, right_hand] = quest_input->getLatestHandData();
        EyeTrackingData eye_data = quest_input->getLatestEyeData();

        // ====================================================================
        // SLAM Processing (placeholder)
        // ====================================================================
        TrackingResult tracking_result;
        tracking_result.status = TrackingStatus::OK;
        tracking_result.confidence = 1.0f;

        // Use Quest's tracking as initial estimate
        if (quest_pose.defined()) {
            tracking_result.pose = quest_pose;
            current_pose = quest_pose;
        } else {
            tracking_result.pose = current_pose;
        }

        // TODO: Run actual SLAM tracking
        // tracking_result = slam_system->track(frame);

        // TODO: Run mapping
        // slam_system->updateMap(frame, tracking_result.pose);

        // ====================================================================
        // Rerun Logging
        // ====================================================================
        if (logger) {
            // Log complete Quest 3 frame with all sensor data
            logger->logQuest3Frame(frame, left_hand, right_hand, eye_data, quest_pose);

            // Log tracking result
            logger->logTrackingIteration(frame, tracking_result, nullptr);
        }

        // ====================================================================
        // Statistics
        // ====================================================================
        auto now = std::chrono::high_resolution_clock::now();
        auto elapsed_since_stats = std::chrono::duration_cast<std::chrono::seconds>(
            now - last_stats_time);

        if (elapsed_since_stats.count() >= 5) {
            auto total_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                now - start_time);
            float fps = frame_count * 1000.0f / total_elapsed.count();

            auto stream_stats = quest_input->getStreamingStats();

            std::cout << "Frames: " << frame_count
                      << " | FPS: " << std::fixed << std::setprecision(1) << fps
                      << " | Latency: " << stream_stats.latency_ms << "ms"
                      << " | Hands: " << (left_hand.is_tracked ? "L" : "-")
                                      << (right_hand.is_tracked ? "R" : "-")
                      << std::endl;

            last_stats_time = now;
        }
    }

    // ========================================================================
    // Cleanup
    // ========================================================================
    std::cout << "Stopping capture..." << std::endl;
    quest_input->stopCapture();

    if (logger) {
        auto stats = logger->getStats();
        std::cout << "\n=== Session Summary ===" << std::endl;
        std::cout << "Total frames: " << stats.total_frames << std::endl;
        std::cout << "Keyframes: " << stats.keyframes << std::endl;
        std::cout << "Tracking failures: " << stats.tracking_failures << std::endl;
        std::cout << "Trajectory length: " << std::fixed << std::setprecision(2)
                  << stats.total_trajectory_length_m << " m" << std::endl;
        std::cout << "Loop closures: " << stats.loop_closures << std::endl;

        logger->endSession();
    }

    std::cout << "Done!" << std::endl;
    return 0;
}
