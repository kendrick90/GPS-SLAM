// WebXR Streaming Server Implementation
// Uses Boost.Beast for WebSocket server + HTTP static file serving

#include "webxr_stream_server.h"
#include <iostream>
#include <thread>
#include <mutex>
#include <set>
#include <chrono>

// Boost.Beast / Asio for WebSocket server
#ifdef SAP_WITH_BOOST_BEAST
#include <boost/beast/core.hpp>
#include <boost/beast/websocket.hpp>
#include <boost/beast/http.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/asio/strand.hpp>
#include <fstream>
#include <filesystem>

namespace beast = boost::beast;
namespace http = beast::http;
namespace websocket = beast::websocket;
namespace net = boost::asio;
using tcp = net::ip::tcp;
#endif

namespace sap::plugins {

// ============================================================================
// WebXRBinaryMessage Implementation
// ============================================================================

WebXRBinaryMessage::WebXRBinaryMessage(uint32_t type, uint32_t count) {
    buffer_.reserve(1024);
    // Header: type + count
    addFloats(reinterpret_cast<const float*>(&type), 1);
    addFloats(reinterpret_cast<const float*>(&count), 1);
}

void WebXRBinaryMessage::addFloat(float value) {
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&value);
    buffer_.insert(buffer_.end(), bytes, bytes + sizeof(float));
}

void WebXRBinaryMessage::addFloats(const float* data, size_t count) {
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(data);
    buffer_.insert(buffer_.end(), bytes, bytes + count * sizeof(float));
}

void WebXRBinaryMessage::addInt64(int64_t value) {
    const uint8_t* bytes = reinterpret_cast<const uint8_t*>(&value);
    buffer_.insert(buffer_.end(), bytes, bytes + sizeof(int64_t));
}

void WebXRBinaryMessage::addUint8(uint8_t value) {
    buffer_.push_back(value);
}

void WebXRBinaryMessage::addTensor(const torch::Tensor& tensor) {
    if (!tensor.defined()) return;

    auto cpu_tensor = tensor.contiguous().cpu().to(torch::kFloat32);
    const float* data = cpu_tensor.data_ptr<float>();
    size_t count = cpu_tensor.numel();
    addFloats(data, count);
}

std::vector<uint8_t> WebXRBinaryMessage::build() const {
    return buffer_;
}

// ============================================================================
// WebXRStreamServer::Impl
// ============================================================================

class WebXRStreamServer::Impl {
public:
    Impl() = default;
    ~Impl() { stop(); }

    bool start(const WebXRStreamConfig& config) {
#ifdef SAP_WITH_BOOST_BEAST
        if (running_) return true;

        config_ = config;

        try {
            // Create IO context and acceptor
            ioc_ = std::make_unique<net::io_context>();
            acceptor_ = std::make_unique<tcp::acceptor>(*ioc_,
                tcp::endpoint(net::ip::make_address(config.bind_address), config.port));

            running_ = true;

            // Start accepting connections
            doAccept();

            // Run IO context in background thread
            io_thread_ = std::thread([this]() {
                ioc_->run();
            });

            std::cout << "[WebXR] Server started on port " << config.port << std::endl;
            return true;
        } catch (const std::exception& e) {
            std::cerr << "[WebXR] Failed to start server: " << e.what() << std::endl;
            return false;
        }
#else
        std::cerr << "[WebXR] Boost.Beast not available, cannot start server" << std::endl;
        return false;
#endif
    }

    void stop() {
#ifdef SAP_WITH_BOOST_BEAST
        if (!running_) return;

        running_ = false;

        if (ioc_) {
            ioc_->stop();
        }

        if (io_thread_.joinable()) {
            io_thread_.join();
        }

        acceptor_.reset();
        ioc_.reset();

        std::lock_guard<std::mutex> lock(clients_mutex_);
        clients_.clear();

        std::cout << "[WebXR] Server stopped" << std::endl;
#endif
    }

    bool isRunning() const { return running_; }

    int getConnectedClientCount() const {
        std::lock_guard<std::mutex> lock(clients_mutex_);
        return static_cast<int>(clients_.size());
    }

    void broadcast(const std::string& message) {
#ifdef SAP_WITH_BOOST_BEAST
        std::lock_guard<std::mutex> lock(clients_mutex_);
        for (auto& client : clients_) {
            try {
                client->text(true);
                client->write(net::buffer(message));
            } catch (...) {
                // Client disconnected
            }
        }
#endif
    }

    void broadcastBinary(const std::vector<uint8_t>& data) {
#ifdef SAP_WITH_BOOST_BEAST
        std::lock_guard<std::mutex> lock(clients_mutex_);
        for (auto& client : clients_) {
            try {
                client->binary(true);
                client->write(net::buffer(data));
            } catch (...) {
                // Client disconnected
            }
        }
#endif
    }

    // Callbacks
    ClientConnectedCallback on_connected_;
    ClientDisconnectedCallback on_disconnected_;
    MessageReceivedCallback on_message_;

private:
#ifdef SAP_WITH_BOOST_BEAST
    void doAccept() {
        acceptor_->async_accept(
            net::make_strand(*ioc_),
            [this](beast::error_code ec, tcp::socket socket) {
                if (!ec && running_) {
                    handleConnection(std::move(socket));
                }
                if (running_) {
                    doAccept();
                }
            });
    }

    void handleConnection(tcp::socket socket) {
        // First, check if this is HTTP or WebSocket
        beast::flat_buffer buffer;
        http::request<http::string_body> req;

        // Read the HTTP request
        beast::error_code ec;
        http::read(socket, buffer, req, ec);
        if (ec) return;

        // Check if it's a WebSocket upgrade request
        if (websocket::is_upgrade(req)) {
            // Upgrade to WebSocket
            auto ws = std::make_shared<websocket::stream<tcp::socket>>(std::move(socket));
            ws->accept(req, ec);
            if (ec) return;

            // Add to clients
            {
                std::lock_guard<std::mutex> lock(clients_mutex_);
                clients_.insert(ws);
            }

            std::string client_id = ws->next_layer().remote_endpoint().address().to_string();
            if (on_connected_) {
                on_connected_(client_id);
            }

            // Read loop
            std::thread([this, ws, client_id]() {
                try {
                    beast::flat_buffer buffer;
                    while (running_) {
                        ws->read(buffer);
                        std::string msg = beast::buffers_to_string(buffer.data());
                        buffer.consume(buffer.size());

                        if (on_message_) {
                            on_message_(client_id, msg);
                        }
                    }
                } catch (...) {
                    // Client disconnected
                }

                {
                    std::lock_guard<std::mutex> lock(clients_mutex_);
                    clients_.erase(ws);
                }

                if (on_disconnected_) {
                    on_disconnected_(client_id);
                }
            }).detach();
        } else {
            // Serve static files
            serveStaticFile(socket, req);
        }
    }

    void serveStaticFile(tcp::socket& socket, const http::request<http::string_body>& req) {
        if (!config_.serve_static_files) {
            sendNotFound(socket);
            return;
        }

        std::string path = req.target().to_string();
        if (path == "/") path = "/index.html";

        // Build file path
        std::filesystem::path file_path;
        if (!config_.static_file_path.empty()) {
            file_path = std::filesystem::path(config_.static_file_path) / path.substr(1);
        } else {
            // Default to webxr/ directory relative to executable
            file_path = std::filesystem::path("plugins/viewers/webxr") / path.substr(1);
        }

        // Read and serve file
        if (std::filesystem::exists(file_path)) {
            std::ifstream file(file_path, std::ios::binary);
            std::string content((std::istreambuf_iterator<char>(file)),
                                std::istreambuf_iterator<char>());

            // Determine content type
            std::string content_type = "text/plain";
            if (path.ends_with(".html")) content_type = "text/html";
            else if (path.ends_with(".js")) content_type = "application/javascript";
            else if (path.ends_with(".css")) content_type = "text/css";
            else if (path.ends_with(".json")) content_type = "application/json";

            http::response<http::string_body> res{http::status::ok, req.version()};
            res.set(http::field::content_type, content_type);
            res.set(http::field::access_control_allow_origin, "*");
            res.body() = content;
            res.prepare_payload();

            beast::error_code ec;
            http::write(socket, res, ec);
        } else {
            sendNotFound(socket);
        }
    }

    void sendNotFound(tcp::socket& socket) {
        http::response<http::string_body> res{http::status::not_found, 11};
        res.set(http::field::content_type, "text/plain");
        res.body() = "Not Found";
        res.prepare_payload();

        beast::error_code ec;
        http::write(socket, res, ec);
    }

    std::unique_ptr<net::io_context> ioc_;
    std::unique_ptr<tcp::acceptor> acceptor_;
    std::thread io_thread_;
    bool running_ = false;

    mutable std::mutex clients_mutex_;
    std::set<std::shared_ptr<websocket::stream<tcp::socket>>> clients_;
    WebXRStreamConfig config_;
#else
    bool running_ = false;
#endif
};

// ============================================================================
// WebXRStreamServer Implementation
// ============================================================================

WebXRStreamServer::WebXRStreamServer()
    : impl_(std::make_unique<Impl>()) {
}

WebXRStreamServer::~WebXRStreamServer() {
    shutdown();
}

bool WebXRStreamServer::initialize(const Config& config) {
    // Extract WebXR config from generic config
    // TODO: Parse config
    return start();
}

void WebXRStreamServer::shutdown() {
    stop();
}

bool WebXRStreamServer::start() {
    if (running_) return true;
    running_ = impl_->start(config_);
    return running_;
}

void WebXRStreamServer::stop() {
    impl_->stop();
    running_ = false;
}

bool WebXRStreamServer::isRunning() const {
    return running_;
}

int WebXRStreamServer::getConnectedClientCount() const {
    return impl_->getConnectedClientCount();
}

std::vector<std::string> WebXRStreamServer::getConnectedClients() const {
    // TODO: Track client IDs
    return {};
}

bool WebXRStreamServer::shouldClose() const {
    return !running_;
}

void WebXRStreamServer::processEvents() {
    // Events are processed in background thread
}

// ==================== Data Streaming ====================

void WebXRStreamServer::showFrame(const Frame& frame) {
    if (!running_ || impl_->getConnectedClientCount() == 0) return;

    // Send frame notification
    std::string msg = R"({"type":"frame","id":)" + std::to_string(frame.frame_id) + "}";
    impl_->broadcast(msg);
}

void WebXRStreamServer::showRender(const RenderResult& render) {
    // TODO: Stream rendered image if needed
}

void WebXRStreamServer::showComparison(const Frame& input, const RenderResult& render) {
    // TODO: Side-by-side comparison
}

void WebXRStreamServer::showImage(const std::string& window_name, const torch::Tensor& image) {
    // TODO: Stream image as base64 or binary
}

void WebXRStreamServer::showDepth(const std::string& window_name, const torch::Tensor& depth,
                                   float min_depth, float max_depth) {
    // TODO: Stream depth as image
}

void WebXRStreamServer::showScene(const SceneData& scene) {
    if (!running_ || impl_->getConnectedClientCount() == 0) return;

    // Stream Gaussians if available
    if (config_.stream_gaussians && scene.means.defined()) {
        streamGaussians(scene);
    }
}

void WebXRStreamServer::showTrajectory(const std::vector<torch::Tensor>& poses) {
    if (!running_ || impl_->getConnectedClientCount() == 0) return;

    if (config_.stream_trajectory) {
        streamTrajectory(poses);
    }
}

void WebXRStreamServer::addCameraFrustum(const torch::Tensor& pose,
                                          const CameraIntrinsics& intrinsics,
                                          float scale) {
    if (!running_) return;

    // Stream pose with camera info
    // Simplified: just stream the pose
    auto now = std::chrono::high_resolution_clock::now().time_since_epoch();
    auto frame_id = std::chrono::duration_cast<std::chrono::milliseconds>(now).count();
    streamPose(pose, frame_id);
}

// ==================== Manual Streaming ====================

void WebXRStreamServer::streamPointCloud(const torch::Tensor& points, const torch::Tensor& colors) {
    if (!running_ || impl_->getConnectedClientCount() == 0) return;
    if (!config_.stream_point_cloud) return;

    // Rate limiting
    auto now = std::chrono::high_resolution_clock::now().time_since_epoch().count() / 1e9;
    if (now - last_point_cloud_time_ < 1.0 / config_.update_rate_hz) return;
    last_point_cloud_time_ = now;

    auto pts = points.contiguous().cpu().to(torch::kFloat32);
    int64_t num_points = pts.size(0);

    // Downsample if needed
    if (config_.point_cloud_stride > 1 || num_points > config_.max_points) {
        int stride = config_.point_cloud_stride;
        if (num_points / stride > config_.max_points) {
            stride = static_cast<int>(num_points / config_.max_points) + 1;
        }

        std::vector<int64_t> indices;
        for (int64_t i = 0; i < num_points; i += stride) {
            indices.push_back(i);
        }
        auto idx_tensor = torch::tensor(indices, torch::kLong);
        pts = pts.index_select(0, idx_tensor);
        num_points = pts.size(0);
    }

    if (config_.use_binary_protocol) {
        WebXRBinaryMessage msg(WebXRMessageType::POINT_CLOUD, static_cast<uint32_t>(num_points));
        msg.addTensor(pts);

        if (colors.defined()) {
            auto cols = colors.contiguous().cpu().to(torch::kFloat32);
            if (cols.size(0) == num_points) {
                msg.addTensor(cols);
            } else {
                // Generate white colors
                auto white = torch::ones({num_points, 3}, torch::kFloat32) * 0.8f;
                msg.addTensor(white);
            }
        } else {
            auto white = torch::ones({num_points, 3}, torch::kFloat32) * 0.8f;
            msg.addTensor(white);
        }

        impl_->broadcastBinary(msg.build());
    } else {
        // JSON fallback (slower)
        // TODO: Implement JSON point cloud streaming
    }
}

void WebXRStreamServer::streamGaussians(const SceneData& gaussians) {
    if (!running_ || impl_->getConnectedClientCount() == 0) return;
    if (!config_.stream_gaussians) return;

    if (!gaussians.means.defined()) return;

    int64_t num_gaussians = gaussians.means.size(0);
    if (num_gaussians == 0) return;

    // For WebXR, we simplify Gaussians to colored points for now
    // Full Gaussian splatting in WebXR would require a custom shader

    if (config_.use_binary_protocol) {
        WebXRBinaryMessage msg(WebXRMessageType::GAUSSIANS, static_cast<uint32_t>(num_gaussians));

        // Positions
        msg.addTensor(gaussians.means);

        // Scales (simplified)
        if (gaussians.scales.defined()) {
            msg.addTensor(gaussians.scales);
        } else {
            auto scales = torch::ones({num_gaussians, 3}, torch::kFloat32) * 0.01f;
            msg.addTensor(scales);
        }

        // Rotations
        if (gaussians.quats.defined()) {
            msg.addTensor(gaussians.quats);
        } else {
            auto quats = torch::zeros({num_gaussians, 4}, torch::kFloat32);
            quats.select(1, 0).fill_(1.0f);  // Identity quaternion
            msg.addTensor(quats);
        }

        // Colors (from SH DC)
        if (gaussians.features_dc.defined()) {
            auto colors = gaussians.features_dc.squeeze(-1).clamp(0.0f, 1.0f);
            msg.addTensor(colors);
        } else {
            auto colors = torch::ones({num_gaussians, 3}, torch::kFloat32) * 0.5f;
            msg.addTensor(colors);
        }

        impl_->broadcastBinary(msg.build());
    }
}

void WebXRStreamServer::streamPose(const torch::Tensor& pose, int64_t frame_id) {
    if (!running_ || impl_->getConnectedClientCount() == 0) return;
    if (!config_.stream_poses) return;

    if (!pose.defined()) return;

    if (config_.use_binary_protocol) {
        WebXRBinaryMessage msg(WebXRMessageType::POSE, 1);
        msg.addTensor(pose);
        msg.addInt64(frame_id);
        impl_->broadcastBinary(msg.build());
    } else {
        auto p = pose.contiguous().cpu().to(torch::kFloat32);
        std::vector<float> data(p.data_ptr<float>(), p.data_ptr<float>() + 16);

        std::string json = "{\"type\":\"pose\",\"pose\":[";
        for (size_t i = 0; i < data.size(); i++) {
            json += std::to_string(data[i]);
            if (i < data.size() - 1) json += ",";
        }
        json += "],\"frame_id\":" + std::to_string(frame_id) + "}";

        impl_->broadcast(json);
    }
}

void WebXRStreamServer::streamTrajectory(const std::vector<torch::Tensor>& poses) {
    if (!running_ || impl_->getConnectedClientCount() == 0) return;
    if (!config_.stream_trajectory) return;

    // Rate limiting
    auto now = std::chrono::high_resolution_clock::now().time_since_epoch().count() / 1e9;
    if (now - last_trajectory_time_ < 0.5) return;  // Max 2Hz for trajectory
    last_trajectory_time_ = now;

    // Extract positions from poses
    std::vector<float> positions;
    int count = 0;
    int stride = 1;
    if (poses.size() > static_cast<size_t>(config_.trajectory_max_points)) {
        stride = static_cast<int>(poses.size() / config_.trajectory_max_points) + 1;
    }

    for (size_t i = 0; i < poses.size(); i += stride) {
        if (poses[i].defined()) {
            auto p = poses[i].contiguous().cpu().to(torch::kFloat32);
            auto data = p.data_ptr<float>();
            // Extract translation (column 3)
            positions.push_back(data[3]);   // x
            positions.push_back(data[7]);   // y
            positions.push_back(data[11]);  // z
            count++;
        }
    }

    if (count == 0) return;

    if (config_.use_binary_protocol) {
        WebXRBinaryMessage msg(WebXRMessageType::TRAJECTORY, static_cast<uint32_t>(count));
        msg.addFloats(positions.data(), positions.size());
        impl_->broadcastBinary(msg.build());
    }
}

void WebXRStreamServer::streamHands(const torch::Tensor& left_keypoints, const torch::Tensor& right_keypoints) {
    if (!running_ || impl_->getConnectedClientCount() == 0) return;
    if (!config_.stream_hands) return;

    bool left_tracked = left_keypoints.defined() && left_keypoints.size(0) == 21;
    bool right_tracked = right_keypoints.defined() && right_keypoints.size(0) == 21;

    if (!left_tracked && !right_tracked) return;

    if (config_.use_binary_protocol) {
        WebXRBinaryMessage msg(WebXRMessageType::HANDS, 2);

        if (left_tracked) {
            msg.addTensor(left_keypoints);
        } else {
            auto zeros = torch::zeros({21, 3}, torch::kFloat32);
            msg.addTensor(zeros);
        }

        if (right_tracked) {
            msg.addTensor(right_keypoints);
        } else {
            auto zeros = torch::zeros({21, 3}, torch::kFloat32);
            msg.addTensor(zeros);
        }

        msg.addUint8(left_tracked ? 1 : 0);
        msg.addUint8(right_tracked ? 1 : 0);

        impl_->broadcastBinary(msg.build());
    }
}

void WebXRStreamServer::streamClear() {
    if (!running_) return;

    if (config_.use_binary_protocol) {
        WebXRBinaryMessage msg(WebXRMessageType::CLEAR, 0);
        impl_->broadcastBinary(msg.build());
    } else {
        impl_->broadcast(R"({"type":"clear"})");
    }
}

void WebXRStreamServer::broadcast(const std::string& json_message) {
    impl_->broadcast(json_message);
}

void WebXRStreamServer::broadcastBinary(const std::vector<uint8_t>& data) {
    impl_->broadcastBinary(data);
}

std::string WebXRStreamServer::getServerURL() const {
    return "ws://" + config_.bind_address + ":" + std::to_string(config_.port);
}

std::string WebXRStreamServer::getViewerURL() const {
    // Return HTTP URL for the viewer page
    return "http://" + config_.bind_address + ":" + std::to_string(config_.port) + "/";
}

void WebXRStreamServer::setOnClientConnected(ClientConnectedCallback callback) {
    impl_->on_connected_ = callback;
}

void WebXRStreamServer::setOnClientDisconnected(ClientDisconnectedCallback callback) {
    impl_->on_disconnected_ = callback;
}

void WebXRStreamServer::setOnMessageReceived(MessageReceivedCallback callback) {
    impl_->on_message_ = callback;
}

}  // namespace sap::plugins
