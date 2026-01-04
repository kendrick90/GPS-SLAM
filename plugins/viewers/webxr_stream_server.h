#pragma once
// WebXR Streaming Server
// Streams SLAM data to WebXR viewer running in Quest 3 browser
// Based on VideoDepthViewer3D pattern but optimized for SLAM data

#include "sap/interfaces/viewer.h"
#include <memory>
#include <functional>
#include <string>
#include <vector>

namespace sap::plugins {

// Configuration for WebXR streaming
struct WebXRStreamConfig {
    // Server settings
    int port = 6689;
    std::string bind_address = "0.0.0.0";

    // Streaming options
    bool serve_static_files = true;     // Serve the WebXR viewer HTML
    std::string static_file_path = "";   // Path to webxr/ directory

    // Data streaming settings
    bool stream_point_cloud = true;
    bool stream_gaussians = true;
    bool stream_trajectory = true;
    bool stream_hands = true;
    bool stream_poses = true;

    // Performance
    int point_cloud_stride = 4;         // Downsample points (1 = all)
    int max_points = 500000;            // Max points to stream
    int trajectory_max_points = 1000;   // Max trajectory points
    float update_rate_hz = 30.0f;       // Target update rate

    // Compression
    bool use_binary_protocol = true;    // Binary vs JSON (binary is faster)
    bool compress_data = false;         // zlib compression (TODO)
};

// WebXR streaming server - implements IViewer interface
class WebXRStreamServer : public IViewer {
public:
    WebXRStreamServer();
    ~WebXRStreamServer() override;

    // IPlugin interface
    bool initialize(const Config& config) override;
    void shutdown() override;
    std::string name() const override { return "webxr_stream"; }
    std::string version() const override { return "1.0.0"; }

    // IViewer interface
    void showFrame(const Frame& frame) override;
    void showRender(const RenderResult& render) override;
    void showComparison(const Frame& input, const RenderResult& render) override;
    void showImage(const std::string& window_name, const torch::Tensor& image) override;
    void showDepth(const std::string& window_name, const torch::Tensor& depth,
                   float min_depth = 0.0f, float max_depth = 10.0f) override;

    // 3D visualization
    void showScene(const SceneData& scene) override;
    void showTrajectory(const std::vector<torch::Tensor>& poses) override;
    void addCameraFrustum(const torch::Tensor& pose,
                          const CameraIntrinsics& intrinsics,
                          float scale = 0.1f) override;

    // Window management (different semantics for server)
    bool shouldClose() const override;
    void processEvents() override;
    int waitKey(int timeout_ms = 0) override { return -1; }

    // Not applicable for streaming server
    bool supportsInteractiveCamera() const override { return false; }
    std::optional<torch::Tensor> getInteractiveCameraPose() const override { return std::nullopt; }
    void setInteractiveCameraPose(const torch::Tensor& pose) override {}

    // Recording (streams to connected clients)
    bool startRecording(const std::string& path, int fps = 30) override { return false; }
    void stopRecording() override {}
    bool isRecording() const override { return false; }

    // ==================== WebXR Server Specific ====================

    // Start/stop server
    bool start();
    void stop();
    bool isRunning() const;

    // Configuration
    void setConfig(const WebXRStreamConfig& config) { config_ = config; }
    WebXRStreamConfig getConfig() const { return config_; }

    // Connection status
    int getConnectedClientCount() const;
    std::vector<std::string> getConnectedClients() const;

    // Manual data streaming
    void streamPointCloud(const torch::Tensor& points, const torch::Tensor& colors);
    void streamGaussians(const SceneData& gaussians);
    void streamPose(const torch::Tensor& pose, int64_t frame_id);
    void streamTrajectory(const std::vector<torch::Tensor>& poses);
    void streamHands(const torch::Tensor& left_keypoints, const torch::Tensor& right_keypoints);
    void streamClear();

    // Broadcast raw message
    void broadcast(const std::string& json_message);
    void broadcastBinary(const std::vector<uint8_t>& data);

    // Get server URL for connecting
    std::string getServerURL() const;
    std::string getViewerURL() const;  // URL to open WebXR viewer

    // Callbacks
    using ClientConnectedCallback = std::function<void(const std::string& client_id)>;
    using ClientDisconnectedCallback = std::function<void(const std::string& client_id)>;
    using MessageReceivedCallback = std::function<void(const std::string& client_id, const std::string& message)>;

    void setOnClientConnected(ClientConnectedCallback callback);
    void setOnClientDisconnected(ClientDisconnectedCallback callback);
    void setOnMessageReceived(MessageReceivedCallback callback);

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
    WebXRStreamConfig config_;
    bool running_ = false;

    // Rate limiting
    double last_point_cloud_time_ = 0;
    double last_trajectory_time_ = 0;
    double last_pose_time_ = 0;
};

// ============================================================================
// Binary Protocol Definition
// ============================================================================
//
// All messages start with:
//   [4 bytes: message_type (uint32_t)]
//   [4 bytes: payload_count (uint32_t)]
//   [payload...]
//
// Message types:
//   1 = Point cloud
//       payload: [count * 3 * float32 positions] [count * 3 * float32 colors]
//
//   2 = Gaussians (simplified)
//       payload: [count * 3 * float32 positions] [count * 3 * float32 scales]
//                [count * 4 * float32 rotations] [count * 3 * float32 colors]
//
//   3 = Trajectory
//       payload: [count * 3 * float32 positions]
//
//   4 = Pose
//       payload: [16 * float32 matrix] [int64 frame_id]
//
//   5 = Hands
//       payload: [21 * 3 * float32 left_keypoints] [21 * 3 * float32 right_keypoints]
//                [uint8 left_tracked] [uint8 right_tracked]
//
//   100 = Clear scene

namespace WebXRMessageType {
    constexpr uint32_t POINT_CLOUD = 1;
    constexpr uint32_t GAUSSIANS = 2;
    constexpr uint32_t TRAJECTORY = 3;
    constexpr uint32_t POSE = 4;
    constexpr uint32_t HANDS = 5;
    constexpr uint32_t CLEAR = 100;
}

// Helper to build binary messages
class WebXRBinaryMessage {
public:
    WebXRBinaryMessage(uint32_t type, uint32_t count = 0);

    void addFloat(float value);
    void addFloats(const float* data, size_t count);
    void addInt64(int64_t value);
    void addUint8(uint8_t value);

    // From tensor
    void addTensor(const torch::Tensor& tensor);

    std::vector<uint8_t> build() const;

private:
    std::vector<uint8_t> buffer_;
};

}  // namespace sap::plugins

// ============================================================================
// Usage Example
// ============================================================================
//
// // Create and start streaming server
// WebXRStreamServer server;
// WebXRStreamConfig config;
// config.port = 6689;
// config.stream_point_cloud = true;
// config.stream_gaussians = true;
// server.setConfig(config);
//
// if (!server.start()) {
//     std::cerr << "Failed to start WebXR server" << std::endl;
//     return 1;
// }
//
// std::cout << "Open in Quest 3 browser: " << server.getViewerURL() << std::endl;
//
// // In SLAM loop, stream data
// while (running) {
//     Frame frame = input_source->getNextFrame();
//     auto tracking_result = tracker->track(frame);
//
//     // Stream to all connected WebXR viewers
//     server.streamPose(tracking_result.pose, frame.frame_id);
//
//     if (frame.hasDepth()) {
//         auto points = depthToPoints(frame.depth.value(), frame.intrinsics);
//         server.streamPointCloud(points, colors);
//     }
//
//     // Stream Gaussian updates periodically
//     if (frame.frame_id % 30 == 0) {
//         auto gaussians = mapper->getGaussians();
//         server.streamGaussians(gaussians);
//     }
// }
//
// server.stop();
