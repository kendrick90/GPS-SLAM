// Nanobind module for Spatial AI Playground
// Exposes C++ SLAM and sensor classes to Python

#include <nanobind/nanobind.h>
#include <nanobind/ndarray.h>
#include <nanobind/stl/string.h>
#include <nanobind/stl/vector.h>
#include <nanobind/stl/optional.h>
#include <nanobind/eigen/dense.h>

#include "../sensors/camera.h"
#include "../sensors/webcam.h"
#include "../slam/tracker.h"

namespace nb = nanobind;
using namespace nb::literals;

namespace sap {

// Helper to convert numpy array to cv::Mat (zero-copy)
cv::Mat ndarray_to_mat(nb::ndarray<uint8_t, nb::c_contig, nb::device::cpu> arr) {
    if (arr.ndim() < 2) return cv::Mat();
    int rows = static_cast<int>(arr.shape(0));
    int cols = static_cast<int>(arr.shape(1));
    int channels = (arr.ndim() >= 3) ? static_cast<int>(arr.shape(2)) : 1;
    int type = (channels == 3) ? CV_8UC3 : (channels == 1) ? CV_8UC1 : CV_8UC4;
    return cv::Mat(rows, cols, type, reinterpret_cast<void*>(const_cast<uint8_t*>(arr.data())));
}

}  // namespace sap

NB_MODULE(_core, m) {
    m.doc() = "Spatial AI Playground C++ Core";

    using namespace sap;

    // ========== Camera Intrinsics ==========
    nb::class_<CameraIntrinsics>(m, "CameraIntrinsics")
        .def(nb::init<>())
        .def_rw("fx", &CameraIntrinsics::fx)
        .def_rw("fy", &CameraIntrinsics::fy)
        .def_rw("cx", &CameraIntrinsics::cx)
        .def_rw("cy", &CameraIntrinsics::cy)
        .def_rw("width", &CameraIntrinsics::width)
        .def_rw("height", &CameraIntrinsics::height)
        .def("K", &CameraIntrinsics::K, "Get 3x3 intrinsic matrix");

    // ========== IMU Sample ==========
    nb::class_<IMUSample>(m, "IMUSample")
        .def(nb::init<>())
        .def_rw("timestamp", &IMUSample::timestamp)
        .def_rw("accel", &IMUSample::accel)
        .def_rw("gyro", &IMUSample::gyro);

    // ========== Frame Data ==========
    nb::class_<FrameData>(m, "FrameData")
        .def(nb::init<>())
        .def_rw("timestamp_ns", &FrameData::timestamp_ns)
        .def_rw("frame_id", &FrameData::frame_id)
        .def_rw("intrinsics", &FrameData::intrinsics)
        .def("valid", &FrameData::valid)
        .def("get_rgb_copy", [](const FrameData& f) -> std::vector<uint8_t> {
            // Return as flat vector (to be reshaped in Python)
            if (f.rgb.empty()) return {};
            std::vector<uint8_t> data(f.rgb.total() * f.rgb.elemSize());
            std::memcpy(data.data(), f.rgb.data, data.size());
            return data;
        })
        .def("rgb_shape", [](const FrameData& f) -> std::vector<size_t> {
            if (f.rgb.empty()) return {};
            return {static_cast<size_t>(f.rgb.rows), static_cast<size_t>(f.rgb.cols), 3};
        })
        .def_prop_ro("has_depth", [](const FrameData& f) { return f.depth.has_value(); });

    // ========== Camera ==========
    nb::class_<Camera>(m, "Camera")
        .def("open", &Camera::open, "config"_a = "")
        .def("close", &Camera::close)
        .def("is_open", &Camera::isOpen)
        .def("get_frame", &Camera::getFrame)
        .def("get_intrinsics", &Camera::getIntrinsics)
        .def("has_depth", &Camera::hasDepth)
        .def("has_imu", &Camera::hasIMU)
        .def("name", &Camera::name);

    // ========== Webcam Camera ==========
    nb::class_<WebcamCamera, Camera>(m, "WebcamCamera")
        .def(nb::init<>())
        .def("open", &WebcamCamera::open, "config"_a = "")
        .def("close", &WebcamCamera::close)
        .def("is_open", &WebcamCamera::isOpen)
        .def("get_frame", &WebcamCamera::getFrame)
        .def("get_intrinsics", &WebcamCamera::getIntrinsics)
        .def("name", &WebcamCamera::name);

    // ========== Tracking Status ==========
    nb::enum_<TrackingStatus>(m, "TrackingStatus")
        .value("GOOD", TrackingStatus::GOOD)
        .value("POOR", TrackingStatus::POOR)
        .value("LOST", TrackingStatus::LOST)
        .value("INITIALIZING", TrackingStatus::INITIALIZING);

    // ========== Tracking Result ==========
    nb::class_<TrackingResult>(m, "TrackingResult")
        .def(nb::init<>())
        .def_rw("pose", &TrackingResult::pose)
        .def_rw("status", &TrackingResult::status)
        .def_rw("confidence", &TrackingResult::confidence)
        .def_rw("is_keyframe", &TrackingResult::is_keyframe)
        .def_rw("frame_id", &TrackingResult::frame_id);

    // ========== Map Point ==========
    nb::class_<MapPoint>(m, "MapPoint")
        .def(nb::init<>())
        .def_rw("position", &MapPoint::position)
        .def_rw("color", &MapPoint::color)
        .def_rw("normal", &MapPoint::normal)
        .def_rw("confidence", &MapPoint::confidence);

    // ========== Tracker ==========
    nb::class_<Tracker>(m, "Tracker")
        .def("track", nb::overload_cast<const FrameData&>(&Tracker::track))
        .def("reset", &Tracker::reset)
        .def("set_pose", &Tracker::setPose)
        .def("get_current_pose", &Tracker::getCurrentPose)
        .def("get_trajectory", &Tracker::getTrajectory)
        .def("get_keyframes", &Tracker::getKeyframes)
        .def("get_point_cloud", &Tracker::getPointCloud)
        .def("requires_depth", &Tracker::requiresDepth)
        .def("supports_imu", &Tracker::supportsIMU)
        .def("name", &Tracker::name)
        .def_static("create", &Tracker::create, "type"_a = "default");

    // ========== Simple VO ==========
    nb::class_<SimpleVO, Tracker>(m, "SimpleVO")
        .def(nb::init<>());

    // ========== Utility Functions ==========
    m.def("create_webcam", []() { return std::make_unique<WebcamCamera>(); },
          "Create a webcam camera instance");

    m.def("create_tracker", [](const std::string& type) { return Tracker::create(type); },
          "type"_a = "default", "Create a tracker instance");
}
