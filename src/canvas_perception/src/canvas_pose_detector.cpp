/**
 * canvas_pose_detector.cpp  --  Member 3 canvas pose node  (v13)
 *
 * New in v13:
 *   - 3D RPY arc markers in RViz (/canvas/markers, namespace "rpy_arcs")
 *     Roll arc  = red   LINE_STRIP sweeping around canvas X axis (Y-Z plane)
 *     Pitch arc = green LINE_STRIP sweeping around canvas Y axis (X-Z plane)
 *     Yaw arc   = blue  LINE_STRIP sweeping around canvas Z axis (X-Y plane)
 *   - Radial spoke + angle label at each arc tip
 *   - View in RViz2: Fixed Frame = camera_color_optical_frame,
 *     Add MarkerArray -> /canvas/markers
 *
 * New in v12:
 *   - XYZ + RPY (deg) terminal log every 500ms (throttled)
 *   - RViz TEXT_VIEW_FACING marker at canvas centre showing live XYZ + RPY
 *   - RPY computed via standard ZYX extrinsic formula (ROS convention)
 *
 * New in v11:
 *   - TF frame broadcast for each of the 4 corner markers
 *     (canvas_marker_TL, canvas_marker_TR, canvas_marker_BR, canvas_marker_BL)
 *   - Per-marker XYZ axis arrows in RViz MarkerArray (red=X, green=Y, blue=Z)
 *   - Canvas pose uses cmsg->header.stamp so it updates live as canvas moves
 *   - Reduced debug image text sizes for cleaner display
 *
 * Physical setup:
 *     [ID:0] ─────────────── [ID:1]
 *       │      A4 Canvas        │
 *     [ID:3] ─────────────── [ID:2]
 *
 *   Print AprilTag 36h11 at: https://chev.me/arucogen/
 *   Size: 50mm  |  IDs: 0(TL) 1(TR) 2(BR) 3(BL)
 */

#include <array>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>
#include <Eigen/Dense>

// ============================================================================
// Helpers
// ============================================================================

static float sampleDepth(const cv::Mat & d16, int u, int v, int half)
{
  std::vector<float> vals;
  vals.reserve((2*half+1)*(2*half+1));
  for (int r = std::max(0,v-half); r <= std::min(d16.rows-1,v+half); ++r)
    for (int c = std::max(0,u-half); c <= std::min(d16.cols-1,u+half); ++c) {
      uint16_t raw = d16.at<uint16_t>(r,c);
      if (raw > 0) vals.push_back(raw * 0.001f);
    }
  if (vals.empty()) return 0.f;
  std::nth_element(vals.begin(), vals.begin()+vals.size()/2, vals.end());
  return vals[vals.size()/2];
}

static Eigen::Vector3d backProject(double u, double v, double z,
  double fx, double fy, double cx, double cy)
{ return {(u-cx)*z/fx, (v-cy)*z/fy, z}; }

static geometry_msgs::msg::Point toGPoint(const Eigen::Vector3d & v)
{ geometry_msgs::msg::Point p; p.x=v.x(); p.y=v.y(); p.z=v.z(); return p; }

static void canvasDimensions(const std::vector<Eigen::Vector3d> & c,
  double & width, double & height)
{
  width  = ((c[1]-c[0]).norm() + (c[2]-c[3]).norm()) / 2.0;
  height = ((c[3]-c[0]).norm() + (c[2]-c[1]).norm()) / 2.0;
}

// Convert quaternion to RPY (roll/pitch/yaw) in radians -- ZYX extrinsic (ROS convention)
static void quaternionToRPY(const Eigen::Quaterniond & q,
  double & roll, double & pitch, double & yaw)
{
  Eigen::Matrix3d R = q.toRotationMatrix();
  // Standard ZYX extrinsic decomposition
  pitch = std::asin(-R(2, 0));
  if (std::abs(R(2, 0)) < 0.9999) {
    roll = std::atan2(R(2, 1), R(2, 2));
    yaw  = std::atan2(R(1, 0), R(0, 0));
  } else {
    // Gimbal lock -- yaw absorbs all rotation
    roll = 0.0;
    yaw  = std::atan2(-R(0, 1), R(1, 1));
  }
}

// solvePnP for canvas orientation using 4 corner centres
static Eigen::Quaterniond solvePnPOrientation(
  const std::vector<cv::Point2f> & img_pts,
  double canvas_w, double canvas_h,
  const cv::Mat & K, const cv::Mat & D)
{
  std::vector<cv::Point3f> obj = {
    {(float)(-canvas_w/2), (float)( canvas_h/2), 0.f},  // TL
    {(float)( canvas_w/2), (float)( canvas_h/2), 0.f},  // TR
    {(float)( canvas_w/2), (float)(-canvas_h/2), 0.f},  // BR
    {(float)(-canvas_w/2), (float)(-canvas_h/2), 0.f},  // BL
  };
  cv::Vec3d rvec, tvec;
  if (!cv::solvePnP(obj, img_pts, K, D, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE))
    return Eigen::Quaterniond::Identity();
  cv::Mat R; cv::Rodrigues(rvec, R);
  Eigen::Matrix3d Re;
  for (int i=0; i<3; ++i) for (int j=0; j<3; ++j) Re(i,j)=R.at<double>(i,j);
  return Eigen::Quaterniond(Re).normalized();
}

// solvePnP for individual marker orientation (used for per-marker TF)
static Eigen::Quaterniond markerOrientation(
  const std::vector<cv::Point2f> & corners,  // 4 corners of one marker
  double marker_size,
  const cv::Mat & K, const cv::Mat & D)
{
  float h = (float)(marker_size / 2.0);
  std::vector<cv::Point3f> obj = {
    {-h,  h, 0.f}, { h,  h, 0.f},
    { h, -h, 0.f}, {-h, -h, 0.f},
  };
  cv::Vec3d rvec, tvec;
  if (!cv::solvePnP(obj, corners, K, D, rvec, tvec, false, cv::SOLVEPNP_IPPE_SQUARE))
    return Eigen::Quaterniond::Identity();
  cv::Mat R; cv::Rodrigues(rvec, R);
  Eigen::Matrix3d Re;
  for (int i=0; i<3; ++i) for (int j=0; j<3; ++j) Re(i,j)=R.at<double>(i,j);
  return Eigen::Quaterniond(Re).normalized();
}

// ============================================================================
// RPY arc helper
// ============================================================================

/**
 * Build a LINE_STRIP MarkerArray entry representing one RPY arc.
 *
 * The arc sweeps angle_rad radians around rot_axis_world, starting from
 * u_axis_world. v_axis_world is perpendicular to both (right-hand rule).
 * Points lie on a circle of radius arc_r centred at origin.
 *
 * Colour: roll=red, pitch=green, yaw=blue (matches RViz axis convention)
 */
static visualization_msgs::msg::Marker makeRPYArc(
  const std::string & frame_id,
  const builtin_interfaces::msg::Time & stamp,
  const std::string & ns,
  int id,
  const Eigen::Vector3d & origin,
  const Eigen::Vector3d & u_axis,   // arc start direction
  const Eigen::Vector3d & v_axis,   // arc 90-deg direction
  double angle_rad,
  double arc_r,
  float r, float g, float b,
  const rclcpp::Duration & life,
  int n_segs = 48)
{
  namespace VM = visualization_msgs::msg;
  VM::Marker m;
  m.header.frame_id = frame_id;
  m.header.stamp    = stamp;
  m.ns              = ns;
  m.id              = id;
  m.type            = VM::Marker::LINE_STRIP;
  m.action          = VM::Marker::ADD;
  m.scale.x         = 0.004f;   // line width 4mm
  m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = 1.f;
  m.lifetime        = life;
  m.pose.orientation.w = 1.0;

  // Sweep from 0 to angle_rad
  // If angle is near zero skip to avoid degenerate markers
  if (std::abs(angle_rad) < 1e-4) {
    // Still add two identical points so the marker is valid
    geometry_msgs::msg::Point p;
    p.x = origin.x() + arc_r * u_axis.x();
    p.y = origin.y() + arc_r * u_axis.y();
    p.z = origin.z() + arc_r * u_axis.z();
    m.points.push_back(p);
    m.points.push_back(p);
    return m;
  }

  for (int k = 0; k <= n_segs; ++k) {
    double t  = angle_rad * k / n_segs;
    Eigen::Vector3d pt = origin
      + arc_r * (std::cos(t) * u_axis + std::sin(t) * v_axis);
    geometry_msgs::msg::Point p;
    p.x = pt.x(); p.y = pt.y(); p.z = pt.z();
    m.points.push_back(p);
  }
  return m;
}

// ============================================================================
// Node
// ============================================================================

class CanvasPoseDetector : public rclcpp::Node
{
public:
  CanvasPoseDetector() : Node("canvas_pose_detector")
  {
    declare_parameter("marker_id_tl",         0);
    declare_parameter("marker_id_tr",         1);
    declare_parameter("marker_id_br",         2);
    declare_parameter("marker_id_bl",         3);
    declare_parameter("marker_size_m",        0.050);
    declare_parameter("aruco_dict",           std::string("DICT_APRILTAG_36h11"));
    declare_parameter("clahe_clip",           4.0);
    declare_parameter("clahe_tile",           8);
    declare_parameter("depth_sample_half",    12);
    declare_parameter("max_depth_m",          3.0);
    declare_parameter("missing_frames_tol",   5);
    declare_parameter("publish_debug",        true);
    declare_parameter("debug_scale",          0.5);
    declare_parameter("debug_fps",            10.0);
    // RPY display offsets -- subtracted from computed RPY before logging and
    // RViz markers. Does NOT affect the published /canvas/pose quaternion.
    // Set these to your observed flat-on baseline values so display reads 0.
    declare_parameter("rpy_offset_roll",      133.0);   // observed flat-on roll
    declare_parameter("rpy_offset_pitch",     0.0);
    declare_parameter("rpy_offset_yaw",       -176.8);  // observed flat-on yaw

    // ── ArUco detector tuning (performance-critical) ───────────────────────
    // adaptiveThreshWinSizeMax is the dominant cost.
    // Each step = one full adaptive threshold pass over the image.
    // Steps = (max - min) / step_size  ->  fewer steps = faster detection.
    //
    //   Default (fast, 15-30fps): min=3 max=23 step=10  -> 3 passes  ~5ms
    //   Robust (slow, ~0.2fps):   min=3 max=100 step=4  -> 25 passes ~3500ms
    //   Balanced (recommended):   min=3 max=53  step=10 -> 6 passes  ~30ms
    //
    // Raise max only if markers fail to detect at distance or low contrast.
    declare_parameter("aruco_win_min",        3);
    declare_parameter("aruco_win_max",        53);    // was 100 -- 25x faster
    declare_parameter("aruco_win_step",       10);    // was 4
    declare_parameter("aruco_thresh_const",   1);
    declare_parameter("aruco_error_rate",     0.6);
    declare_parameter("sync_queue_size",      30);    // was 10

    // Physical A4 canvas dimensions for partial marker reconstruction.
    // 0.0 = auto-measured from 4-marker detection.
    // Set to 0.297 / 0.210 for guaranteed A4 landscape geometry even with partial detection.
    declare_parameter("canvas_physical_w_m",  0.297);
    declare_parameter("canvas_physical_h_m",  0.210);

    marker_id_tl_       = get_parameter("marker_id_tl").as_int();
    marker_id_tr_       = get_parameter("marker_id_tr").as_int();
    marker_id_br_       = get_parameter("marker_id_br").as_int();
    marker_id_bl_       = get_parameter("marker_id_bl").as_int();
    marker_size_m_      = get_parameter("marker_size_m").as_double();
    clahe_clip_         = get_parameter("clahe_clip").as_double();
    clahe_tile_         = get_parameter("clahe_tile").as_int();
    depth_sample_half_  = get_parameter("depth_sample_half").as_int();
    max_depth_m_        = get_parameter("max_depth_m").as_double();
    missing_frames_tol_ = get_parameter("missing_frames_tol").as_int();
    publish_debug_      = get_parameter("publish_debug").as_bool();
    debug_scale_        = get_parameter("debug_scale").as_double();
    debug_fps_          = get_parameter("debug_fps").as_double();
    rpy_offset_roll_    = get_parameter("rpy_offset_roll").as_double();
    rpy_offset_pitch_   = get_parameter("rpy_offset_pitch").as_double();
    rpy_offset_yaw_     = get_parameter("rpy_offset_yaw").as_double();
    canvas_physical_w_  = get_parameter("canvas_physical_w_m").as_double();
    canvas_physical_h_  = get_parameter("canvas_physical_h_m").as_double();

    // ArUco / AprilTag dictionary
    auto dict_id = cv::aruco::DICT_APRILTAG_36h11;
    const auto dict_str = get_parameter("aruco_dict").as_string();
    if      (dict_str == "DICT_4X4_50")         dict_id = cv::aruco::DICT_4X4_50;
    else if (dict_str == "DICT_5X5_100")        dict_id = cv::aruco::DICT_5X5_100;
    else if (dict_str == "DICT_6X6_250")        dict_id = cv::aruco::DICT_6X6_250;
    else if (dict_str == "DICT_APRILTAG_36h11") dict_id = cv::aruco::DICT_APRILTAG_36h11;
    else if (dict_str == "DICT_APRILTAG_36h10") dict_id = cv::aruco::DICT_APRILTAG_36h10;
    else if (dict_str == "DICT_APRILTAG_25h9")  dict_id = cv::aruco::DICT_APRILTAG_25h9;
    else if (dict_str == "DICT_APRILTAG_16h5")  dict_id = cv::aruco::DICT_APRILTAG_16h5;
    else RCLCPP_WARN(get_logger(),
      "Unknown dict '%s', using DICT_APRILTAG_36h11", dict_str.c_str());

    dictionary_ = cv::aruco::getPredefinedDictionary(dict_id);
    params_     = cv::aruco::DetectorParameters::create();
    params_->adaptiveThreshWinSizeMin    = get_parameter("aruco_win_min").as_int();
    params_->adaptiveThreshWinSizeMax    = get_parameter("aruco_win_max").as_int();
    params_->adaptiveThreshWinSizeStep   = get_parameter("aruco_win_step").as_int();
    params_->adaptiveThreshConstant      = get_parameter("aruco_thresh_const").as_int();
    params_->minMarkerPerimeterRate      = 0.01;
    params_->maxMarkerPerimeterRate      = 4.0;
    params_->polygonalApproxAccuracyRate = 0.08;
    params_->cornerRefinementMethod      = cv::aruco::CORNER_REFINE_CONTOUR;
    params_->errorCorrectionRate         = get_parameter("aruco_error_rate").as_double();

    int sync_q = get_parameter("sync_queue_size").as_int();
    RCLCPP_INFO(get_logger(),
      "ArUco: win=[%d..%d step %d]  sync_queue=%d  -> ~%d passes/frame",
      params_->adaptiveThreshWinSizeMin,
      params_->adaptiveThreshWinSizeMax,
      params_->adaptiveThreshWinSizeStep,
      sync_q,
      (params_->adaptiveThreshWinSizeMax - params_->adaptiveThreshWinSizeMin)
        / params_->adaptiveThreshWinSizeStep + 1);

    color_sub_.subscribe(this, "/camera/camera/color/image_raw");
    depth_sub_.subscribe(this, "/camera/camera/aligned_depth_to_color/image_raw");
    sync_ = std::make_shared<Sync>(SyncPolicy(sync_q), color_sub_, depth_sub_);
    sync_->registerCallback(std::bind(&CanvasPoseDetector::cb, this,
      std::placeholders::_1, std::placeholders::_2));

    info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
      "/camera/camera/color/camera_info", 1,
      [this](const sensor_msgs::msg::CameraInfo::SharedPtr m){
        if (got_info_) return;
        fx_=m->k[0]; fy_=m->k[4]; cx_=m->k[2]; cy_=m->k[5];
        camera_matrix_ = (cv::Mat_<double>(3,3) <<
          fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
        dist_coeffs_ = cv::Mat::zeros(1, 5, CV_64F);
        got_info_ = true;
        RCLCPP_INFO(get_logger(),
          "Intrinsics: fx=%.2f fy=%.2f cx=%.2f cy=%.2f", fx_,fy_,cx_,cy_);
      });

    pose_pub_   = create_publisher<geometry_msgs::msg::PoseStamped>("/canvas/pose", 10);
    debug_pub_  = create_publisher<sensor_msgs::msg::Image>("/canvas/debug", 5);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/canvas/markers", 5);
    angles_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/canvas/viewing_angles", 10);

    // TF broadcaster for per-marker frames
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    corner_ids_ = {marker_id_tl_, marker_id_tr_, marker_id_br_, marker_id_bl_};

    RCLCPP_INFO(get_logger(),
      "Canvas pose detector v12 ready.\n"
      "  IDs: TL=%d TR=%d BR=%d BL=%d  dict=%s\n"
      "  TF frames: canvas_marker_TL/TR/BR/BL + canvas_centre\n"
      "  Pose log: XYZ (m) + RPY (deg) every 500ms\n"
      "  RViz: XYZ+RPY text label at canvas_centre\n"
      "  Print AprilTag 36h11: https://chev.me/arucogen/",
      marker_id_tl_, marker_id_tr_, marker_id_br_, marker_id_bl_,
      dict_str.c_str());
  }

private:
  using SyncPolicy = message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::Image, sensor_msgs::msg::Image>;
  using Sync = message_filters::Synchronizer<SyncPolicy>;

  message_filters::Subscriber<sensor_msgs::msg::Image> color_sub_, depth_sub_;
  std::shared_ptr<Sync> sync_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr info_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr debug_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr angles_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  cv::Ptr<cv::aruco::Dictionary>         dictionary_;
  cv::Ptr<cv::aruco::DetectorParameters> params_;

  double  fx_{0}, fy_{0}, cx_{0}, cy_{0};
  cv::Mat camera_matrix_, dist_coeffs_;
  bool    got_info_{false};

  int    marker_id_tl_, marker_id_tr_, marker_id_br_, marker_id_bl_;
  double marker_size_m_;
  double clahe_clip_;
  int    clahe_tile_;
  int    depth_sample_half_;
  double max_depth_m_;
  int    missing_frames_tol_;
  bool   publish_debug_;
  double debug_scale_;
  double debug_fps_;
  double rpy_offset_roll_;
  double rpy_offset_pitch_;
  double rpy_offset_yaw_;
  double canvas_physical_w_;   // physical A4 width  (metres)
  double canvas_physical_h_;   // physical A4 height (metres)

  rclcpp::Time              last_debug_stamp_{0, 0, RCL_ROS_TIME};
  std::vector<int>          corner_ids_;
  std::map<int,int>         missing_frames_;
  std::map<int,cv::Point2f> last_centres_;

  // Frame names for each corner
  const std::array<std::string,4> corner_frame_names_{
    "canvas_marker_TL", "canvas_marker_TR",
    "canvas_marker_BR", "canvas_marker_BL"};

  // ==========================================================================
  // Main callback
  // ==========================================================================
  void cb(const sensor_msgs::msg::Image::ConstSharedPtr & cmsg,
          const sensor_msgs::msg::Image::ConstSharedPtr & dmsg)
  {
    if (!got_info_) return;

    cv::Mat color, depth16;
    try {
      color   = cv_bridge::toCvShare(cmsg, "bgr8")->image.clone();
      depth16 = cv_bridge::toCvShare(dmsg,
        sensor_msgs::image_encodings::TYPE_16UC1)->image.clone();
    } catch (const cv_bridge::Exception & e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge: %s", e.what()); return;
    }

    // CLAHE
    cv::Mat gray, enhanced;
    cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
    auto clahe = cv::createCLAHE(clahe_clip_, cv::Size(clahe_tile_, clahe_tile_));
    clahe->apply(gray, enhanced);

    // ArUco / AprilTag detection
    std::vector<std::vector<cv::Point2f>> marker_corners;
    std::vector<int> marker_ids;
    std::vector<std::vector<cv::Point2f>> rejected;
    cv::aruco::detectMarkers(
      enhanced, dictionary_, marker_corners, marker_ids, params_, rejected);

    // Diagnostics
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
      "[DIAG] %dx%d  CLAHE=%.1f  detected=%zu  rejected=%zu  need IDs:%d %d %d %d",
      color.cols, color.rows, clahe_clip_,
      marker_ids.size(), rejected.size(),
      marker_id_tl_, marker_id_tr_, marker_id_br_, marker_id_bl_);

    for (size_t i = 0; i < marker_ids.size(); ++i) {
      float perim = cv::arcLength(marker_corners[i], true);
      cv::Point2f c(0,0);
      for (const auto & pt : marker_corners[i]) c += pt;
      c *= 0.25f;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000,
        "[DIAG]   ID=%d  centre=(%.0f,%.0f)  perimeter=%.0fpx",
        marker_ids[i], c.x, c.y, perim);
    }
    if (rejected.size() > 0 && marker_ids.empty()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "[DIAG] %zu rejected -- squares found but ID decode failed. "
        "Wrong dictionary?", rejected.size());
    } else if (rejected.empty() && marker_ids.empty()) {
      cv::Scalar mv, sv; cv::meanStdDev(enhanced, mv, sv);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "[DIAG] No candidates. CLAHE mean=%.0f std=%.0f  "
        "(if mean<50: too dark -- raise clahe_clip)", mv[0], sv[0]);
    }

    // Build id -> corners map (for per-marker TF)
    std::map<int, std::vector<cv::Point2f>> id_to_corners;
    std::map<int, cv::Point2f> detected;
    for (size_t i = 0; i < marker_ids.size(); ++i) {
      cv::Point2f c(0,0);
      for (const auto & pt : marker_corners[i]) c += pt;
      detected[marker_ids[i]]      = c * 0.25f;
      id_to_corners[marker_ids[i]] = marker_corners[i];
    }

    // Persistence
    std::map<int, cv::Point2f> centres;
    bool all_present = true;
    for (int id : corner_ids_) {
      if (detected.count(id)) {
        centres[id] = detected[id];
        last_centres_[id] = detected[id];
        missing_frames_[id] = 0;
      } else {
        missing_frames_[id]++;
        if (missing_frames_[id] <= missing_frames_tol_ && last_centres_.count(id))
          centres[id] = last_centres_[id];
        else
          all_present = false;
      }
    }

    if (!all_present) {
      // ── Partial detection: reconstruct missing marker centres from geometry ──
      //
      // Strategy (in 2D image space, depth handled afterwards by existing pipeline):
      //
      //   3 markers  →  parallelogram rule.
      //     For any rectangle ABCD: TL + BR = TR + BL (diagonals bisect each other).
      //     Missing TL = TR + BL - BR
      //     Missing TR = TL + BR - BL
      //     Missing BR = TR + BL - TL
      //     Missing BL = TL + BR - TR
      //
      //   2 adjacent markers (same edge)  →  A4 aspect ratio reconstruction.
      //     The known edge vector scaled by the canvas aspect ratio gives the
      //     perpendicular edge.  Two perpendicular directions exist; we pick the
      //     one that makes geometric sense (positive image Y = downward for top
      //     edge, negative for bottom edge).
      //     Aspect ratio: A4 landscape = 297 / 210 = 1.4143
      //     h_edge → v_edge = rotate90CW(h_edge) * (H/W)   (top edge case)
      //     v_edge → h_edge = rotate90CCW(v_edge) * (W/H)  (left/right edge case)
      //
      //   2 diagonal markers or 1 marker  →  too ambiguous, return early.
      //
      // After reconstruction the code falls through to normal depth+pose pipeline.
      // A status flag is logged and shown in the debug image.

      // Count detected markers and find which are present/missing
      int tl = marker_id_tl_, tr = marker_id_tr_,
          br = marker_id_br_, bl = marker_id_bl_;

      bool has_tl = centres.count(tl), has_tr = centres.count(tr),
           has_br = centres.count(br), has_bl = centres.count(bl);
      int n_present = (int)has_tl + (int)has_tr + (int)has_br + (int)has_bl;

      bool reconstructed = false;
      std::string reconstruct_method;

      if (n_present == 3) {
        // ── 3-marker: parallelogram rule ─────────────────────────────────────
        // TL+BR=TR+BL, so missing = adj1 + adj2 - diagonal_of_missing
        if (!has_tl && has_tr && has_br && has_bl) {
          centres[tl] = centres[tr] + centres[bl] - centres[br];
          reconstruct_method = "3-marker: TL estimated";
        } else if (has_tl && !has_tr && has_br && has_bl) {
          centres[tr] = centres[tl] + centres[br] - centres[bl];
          reconstruct_method = "3-marker: TR estimated";
        } else if (has_tl && has_tr && !has_br && has_bl) {
          centres[br] = centres[tr] + centres[bl] - centres[tl];
          reconstruct_method = "3-marker: BR estimated";
        } else if (has_tl && has_tr && has_br && !has_bl) {
          centres[bl] = centres[tl] + centres[br] - centres[tr];
          reconstruct_method = "3-marker: BL estimated";
        }
        reconstructed = true;

      } else if (n_present == 2) {
        // ── 2-marker: adjacent pair only ─────────────────────────────────────
        // Identify adjacent pairs (share an edge). Diagonal pairs are skipped.
        //
        // A4 landscape aspect ratio: W/H = 297/210, H/W = 210/297
        // For image coords (x right, y down):
        //   Rotate 2D vector (dx,dy) by 90° CW  → ( dy, -dx)
        //   Rotate 2D vector (dx,dy) by 90° CCW → (-dy,  dx)
        //
        // Top edge (TL→TR): v_edge goes downward → use CW rotation of h_edge
        // Bottom edge (BL→BR): v_edge goes upward → use CCW rotation
        // Left edge (TL→BL): h_edge goes rightward → use CCW rotation of v_edge
        // Right edge (TR→BR): h_edge goes leftward → use CW rotation of v_edge

        double aspect_h_over_w = canvas_physical_h_ / canvas_physical_w_;
        double aspect_w_over_h = canvas_physical_w_ / canvas_physical_h_;

        auto rot_cw  = [](cv::Point2f v) { return cv::Point2f( v.y, -v.x); };
        auto rot_ccw = [](cv::Point2f v) { return cv::Point2f(-v.y,  v.x); };

        if (has_tl && has_tr && !has_br && !has_bl) {
          // Top edge: h = TR-TL, v = rot_cw(h)*H/W (downward)
          cv::Point2f h = centres[tr] - centres[tl];
          cv::Point2f v = rot_cw(h) * (float)aspect_h_over_w;
          centres[bl] = centres[tl] + v;
          centres[br] = centres[tr] + v;
          reconstruct_method = "2-marker: BL+BR from TL+TR (top edge)";
          reconstructed = true;

        } else if (has_bl && has_br && !has_tl && !has_tr) {
          // Bottom edge: h = BR-BL, v = rot_ccw(h)*H/W (upward = CCW)
          cv::Point2f h = centres[br] - centres[bl];
          cv::Point2f v = rot_ccw(h) * (float)aspect_h_over_w;
          centres[tl] = centres[bl] + v;
          centres[tr] = centres[br] + v;
          reconstruct_method = "2-marker: TL+TR from BL+BR (bottom edge)";
          reconstructed = true;

        } else if (has_tl && has_bl && !has_tr && !has_br) {
          // Left edge: v = BL-TL, h = rot_ccw(v)*W/H (rightward)
          cv::Point2f v = centres[bl] - centres[tl];
          cv::Point2f h = rot_ccw(v) * (float)aspect_w_over_h;
          centres[tr] = centres[tl] + h;
          centres[br] = centres[bl] + h;
          reconstruct_method = "2-marker: TR+BR from TL+BL (left edge)";
          reconstructed = true;

        } else if (has_tr && has_br && !has_tl && !has_bl) {
          // Right edge: v = BR-TR, h = rot_cw(v)*W/H (leftward)
          cv::Point2f v = centres[br] - centres[tr];
          cv::Point2f h = rot_cw(v) * (float)aspect_w_over_h;
          centres[tl] = centres[tr] + h;
          centres[bl] = centres[br] + h;
          reconstruct_method = "2-marker: TL+BL from TR+BR (right edge)";
          reconstructed = true;

        } else {
          // Diagonal pair -- cannot reconstruct without orientation
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
            "Diagonal marker pair detected -- cannot reconstruct. "
            "Need at least 2 markers on the same edge.");
        }
      }

      if (!reconstructed) {
        std::string miss;
        for (int id : corner_ids_)
          if (!centres.count(id)) miss += std::to_string(id) + " ";
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Partial detection failed (%d/4 markers). Missing: [%s]",
          n_present, miss.c_str());
        publishDebug(color, enhanced, marker_corners, marker_ids,
          {}, {}, 0, 0, cmsg->header, "missing");
        deleteMarkers(cmsg->header);
        return;
      }

      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "[PARTIAL] %s  (%d/4 markers detected)",
        reconstruct_method.c_str(), n_present);
    }

    std::vector<cv::Point2f> corners_2d = {
      centres[marker_id_tl_], centres[marker_id_tr_],
      centres[marker_id_br_], centres[marker_id_bl_]
    };

    // Depth -> 3D corners
    std::vector<Eigen::Vector3d> pts3d;
    bool ok = true;
    for (const auto & c : corners_2d) {
      int u=(int)std::round(c.x), v=(int)std::round(c.y);
      float z = sampleDepth(depth16, u, v, depth_sample_half_);
      if (z < 0.05f || z > (float)max_depth_m_)
        z = sampleDepth(depth16, u, v, depth_sample_half_ * 2);
      if (z < 0.05f || z > (float)max_depth_m_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
          "Invalid depth at (%.0f,%.0f). Increase depth_sample_half.", c.x, c.y);
        ok = false; break;
      }
      pts3d.push_back(backProject(c.x, c.y, z, fx_, fy_, cx_, cy_));
    }

    if (!ok) {
      publishDebug(color, enhanced, marker_corners, marker_ids,
        corners_2d, {}, 0, 0, cmsg->header, "bad_depth");
      return;
    }

    Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
    for (const auto & p : pts3d) centroid += p;
    centroid /= 4.0;

    double canvas_w, canvas_h;
    canvasDimensions(pts3d, canvas_w, canvas_h);

    // Canvas orientation (solvePnP on 4 centres)
    Eigen::Quaterniond q = solvePnPOrientation(
      corners_2d, canvas_w, canvas_h, camera_matrix_, dist_coeffs_);

    // -----------------------------------------------------------------------
    // Viewing geometry -- canvas normal approach
    //
    // REASON FOR THIS APPROACH:
    // When camera translates sideways (canvas fixed), two effects occur:
    //   1. Camera rotation (~7 deg) -- captured by q_rel
    //   2. Parallax from translation (~30+ deg at close range) -- NOT in q_rel
    // A protractor measurement captures BOTH effects (total geometric angle).
    // q_rel-based RPY only captures (1), so pitch reads ~7 instead of ~39.
    //
    // FIX: compute viewing angles directly from the canvas normal vector in
    // camera frame. This captures the full geometric angle (rotation + parallax)
    // and matches physical protractor measurements.
    //
    //   H (horizontal) = how far camera is left/right of canvas centre line
    //                    negative = camera is to the right of canvas
    //   V (vertical)   = how far camera is above/below canvas centre line
    //                    positive = camera is above canvas
    //   Roll           = canvas in-plane rotation (from q_rel, unaffected by translation)
    //
    // /canvas/pose quaternion is still raw q (Member 2 needs real values).
    // -----------------------------------------------------------------------
    const double R2D = 180.0 / M_PI;

    // -----------------------------------------------------------------------
    // H, V, Roll: computed ENTIRELY from 3D corner positions (pts3d).
    //
    // No q_baseline, no solvePnP convention offsets involved.
    // The depth sensor gives correct 3D geometry directly.
    //
    // Corner order: pts3d[0]=TL, pts3d[1]=TR, pts3d[2]=BR, pts3d[3]=BL
    //
    // h_edge: canvas horizontal direction in 3D (left to right)
    // v_edge: canvas vertical direction in 3D (bottom to top)
    // normal = h_edge x v_edge, forced toward camera (-Z)
    //
    // H    = atan2(normal.x, -normal.z)
    //        When flat-on, normal=(0,0,-1) so H=atan2(0,1)=0  naturally zeroed
    //        positive H = camera to LEFT of canvas
    //        negative H = camera to RIGHT of canvas
    //
    // V    = atan2(-normal.y, -normal.z)
    //        When flat-on, normal=(0,0,-1) so V=atan2(0,1)=0  naturally zeroed
    //        positive V = camera ABOVE canvas
    //        negative V = camera BELOW canvas
    //
    // Roll = atan2(h_edge.y, h_edge.x)
    //        When flat-on, h_edge points along +X so Roll=atan2(0,1)=0  naturally zeroed
    //        positive Roll = canvas top tilted to the right (clockwise)
    //        negative Roll = canvas top tilted to the left (anticlockwise)
    //
    // Total = angle between n_depth and (0,0,-1) -- always positive scalar
    //
    // /canvas/pose quaternion is still raw q (Member 2 needs real values).
    // -----------------------------------------------------------------------
    Eigen::Vector3d mid_right  = (pts3d[1] + pts3d[2]) / 2.0;  // TR, BR midpoint
    Eigen::Vector3d mid_left   = (pts3d[0] + pts3d[3]) / 2.0;  // TL, BL midpoint
    Eigen::Vector3d mid_top    = (pts3d[0] + pts3d[1]) / 2.0;  // TL, TR midpoint
    Eigen::Vector3d mid_bottom = (pts3d[3] + pts3d[2]) / 2.0;  // BL, BR midpoint

    Eigen::Vector3d h_edge = mid_right - mid_left;            // canvas horizontal (+X)
    Eigen::Vector3d v_edge = mid_top   - mid_bottom;          // canvas vertical   (+Y up)

    // Canvas face normal -- force toward camera (negative Z in optical frame)
    Eigen::Vector3d n_depth = h_edge.cross(v_edge).normalized();
    if (n_depth.z() > 0.0) n_depth = -n_depth;

    // -----------------------------------------------------------------------
    // Decoupled Roll, H, V computation
    //
    // PROBLEM with naive approach:
    //   H = atan2(n.x, -n.z) and V = atan2(-n.y, -n.z) are measured in the
    //   camera frame. When camera rolls, its Y axis is no longer world-vertical,
    //   so a pure camera roll contaminates V, and a pure vertical movement
    //   slightly contaminates Roll. They couple.
    //
    // FIX:
    //   Step 1 -- Roll: extract from h_edge angle in camera XY plane.
    //             This is the rotation of the canvas horizontal edge around
    //             the optical axis. Naturally 0 when h_edge is along camera +X.
    //
    //   Step 2 -- Unroll the normal: rotate n_depth by -Roll around the Z
    //             (optical) axis. This brings the normal into a "level" frame
    //             where the canvas horizontal edge is perfectly horizontal.
    //             Rotation matrix around Z by angle a:
    //               [cos(a)  -sin(a)  0]
    //               [sin(a)   cos(a)  0]
    //               [0        0       1]
    //
    //   Step 3 -- H and V from the unrolled normal. Now they are independent:
    //             H = atan2(n_u.x, -n_u.z)   -- purely left/right offset
    //             V = atan2(-n_u.y, -n_u.z)  -- purely up/down offset
    //
    // All three are naturally 0 when camera faces canvas flat-on and level.
    // No hardcoded parameters or calibration offsets required.
    // -----------------------------------------------------------------------

    // Step 1: Roll -- in-plane rotation of canvas horizontal edge
    double roll_rad  = std::atan2(h_edge.y(), h_edge.x());
    double roll_disp = roll_rad * R2D;

    // Step 2: Unroll normal by rotating -roll_rad around optical Z axis
    double cos_r = std::cos(-roll_rad);
    double sin_r = std::sin(-roll_rad);
    Eigen::Vector3d n_unrolled(
      cos_r * n_depth.x() - sin_r * n_depth.y(),
      sin_r * n_depth.x() + cos_r * n_depth.y(),
      n_depth.z()
    );

    // Step 3: H and V from unrolled normal -- fully decoupled from Roll
    double horiz_disp = std::atan2( n_unrolled.x(), -n_unrolled.z()) * R2D;
    double vert_disp  = std::atan2(-n_unrolled.y(), -n_unrolled.z()) * R2D;

    // Total angular deviation from flat-on (scalar, always positive)
    Eigen::Vector3d n_flat(0.0, 0.0, -1.0);
    double total_disp = std::acos(
      std::max(-1.0, std::min(1.0, n_depth.dot(n_flat)))) * R2D;

    // Map to pitch/yaw slots for publishMarkers + visualiser compatibility
    double pitch_disp = horiz_disp;
    double yaw_disp   = vert_disp;

    // Terminal log -- throttled to 500ms
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
      "[POSE] XYZ: (x=%.3f  y=%.3f  z=%.3f) m"
      "  |  Roll=%.1f  H=%.1f  V=%.1f deg  Total=%.1f deg",
      centroid.x(), centroid.y(), centroid.z(),
      roll_disp, horiz_disp, vert_disp, total_disp);

    // Publish raw pose (quaternion unchanged -- Member 2 needs real values)
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp    = cmsg->header.stamp;
    pose.header.frame_id = "camera_color_optical_frame";
    pose.pose.position   = toGPoint(centroid);
    pose.pose.orientation.x = q.x(); pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z(); pose.pose.orientation.w = q.w();
    pose_pub_->publish(pose);

    // Publish computed viewing angles for visualiser
    // x = Roll (in-plane), y = H (horizontal, neg=cam right), z = V (vertical, pos=cam above)
    geometry_msgs::msg::Vector3Stamped angles;
    angles.header = pose.header;
    angles.vector.x = roll_disp;
    angles.vector.y = horiz_disp;
    angles.vector.z = vert_disp;
    angles_pub_->publish(angles);

    broadcastMarkerTFs(cmsg->header.stamp, pts3d, id_to_corners, q);

    Eigen::Vector3d normal = q * Eigen::Vector3d(0,0,1);
    publishMarkers(pose, pts3d, normal, centroid,
      canvas_w, canvas_h, id_to_corners, q,
      roll_disp, pitch_disp, yaw_disp);

    publishDebug(color, enhanced, marker_corners, marker_ids,
      corners_2d, pts3d, canvas_w, canvas_h, cmsg->header, "ok");
  }

  // ==========================================================================
  // Broadcast TF for each corner marker + canvas centre
  // ==========================================================================
  void broadcastMarkerTFs(
    const builtin_interfaces::msg::Time & stamp,
    const std::vector<Eigen::Vector3d> & pts3d,
    const std::map<int, std::vector<cv::Point2f>> & id_to_corners,
    const Eigen::Quaterniond & canvas_q)
  {
    const std::string parent = "camera_color_optical_frame";
    std::vector<geometry_msgs::msg::TransformStamped> tfs;

    // Per-corner marker TF
    const std::array<int,4> ids{marker_id_tl_, marker_id_tr_,
                                 marker_id_br_, marker_id_bl_};
    for (int i = 0; i < 4; ++i) {
      geometry_msgs::msg::TransformStamped ts;
      ts.header.stamp    = stamp;
      ts.header.frame_id = parent;
      ts.child_frame_id  = corner_frame_names_[i];

      ts.transform.translation.x = pts3d[i].x();
      ts.transform.translation.y = pts3d[i].y();
      ts.transform.translation.z = pts3d[i].z();

      Eigen::Quaterniond mq = canvas_q;
      if (id_to_corners.count(ids[i])) {
        mq = markerOrientation(
          id_to_corners.at(ids[i]), marker_size_m_, camera_matrix_, dist_coeffs_);
      }
      ts.transform.rotation.x = mq.x();
      ts.transform.rotation.y = mq.y();
      ts.transform.rotation.z = mq.z();
      ts.transform.rotation.w = mq.w();
      tfs.push_back(ts);
    }

    // Canvas centre TF
    {
      Eigen::Vector3d cen = Eigen::Vector3d::Zero();
      for (const auto & p : pts3d) cen += p;
      cen /= 4.0;

      geometry_msgs::msg::TransformStamped ts;
      ts.header.stamp    = stamp;
      ts.header.frame_id = parent;
      ts.child_frame_id  = "canvas_centre";
      ts.transform.translation.x = cen.x();
      ts.transform.translation.y = cen.y();
      ts.transform.translation.z = cen.z();
      ts.transform.rotation.x = canvas_q.x();
      ts.transform.rotation.y = canvas_q.y();
      ts.transform.rotation.z = canvas_q.z();
      ts.transform.rotation.w = canvas_q.w();
      tfs.push_back(ts);
    }

    tf_broadcaster_->sendTransform(tfs);
  }

  // ==========================================================================
  // RViz MarkerArray
  // ==========================================================================
  void publishMarkers(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::vector<Eigen::Vector3d> & corners3d,
    const Eigen::Vector3d & normal,
    const Eigen::Vector3d & centroid,
    double canvas_w, double canvas_h,
    const std::map<int, std::vector<cv::Point2f>> & id_to_corners,
    const Eigen::Quaterniond & q,
    double roll_deg, double pitch_deg, double yaw_deg)
  {
    namespace VM = visualization_msgs::msg;
    VM::MarkerArray arr;
    const auto & fr   = pose.header.frame_id;
    const auto & st   = pose.header.stamp;
    const auto   life = rclcpp::Duration::from_seconds(0.5);
    auto mk = [&](const std::string & ns, int id, int type) {
      VM::Marker m; m.header.frame_id=fr; m.header.stamp=st;
      m.ns=ns; m.id=id; m.type=type; m.action=VM::Marker::ADD;
      return m;
    };

    // Canvas plane (cyan)
    { auto m=mk("canvas",0,VM::Marker::CUBE);
      m.pose=pose.pose;
      m.scale.x=canvas_w; m.scale.y=canvas_h; m.scale.z=0.002;
      m.color.r=0.f; m.color.g=0.8f; m.color.b=1.f; m.color.a=0.3f;
      m.lifetime=life; arr.markers.push_back(m); }

    // Canvas normal arrow (green)
    { auto m=mk("canvas",1,VM::Marker::ARROW);
      m.points.push_back(toGPoint(centroid));
      m.points.push_back(toGPoint(centroid + normal*0.10));
      m.scale.x=0.006f; m.scale.y=0.012f; m.scale.z=0.015f;
      m.color.r=0.f; m.color.g=1.f; m.color.b=0.f; m.color.a=1.f;
      m.lifetime=life; arr.markers.push_back(m); }

    // Canvas size label
    { auto m=mk("canvas",2,VM::Marker::TEXT_VIEW_FACING);
      m.pose.position=toGPoint(centroid+Eigen::Vector3d(0,-0.06,0));
      m.pose.orientation.w=1.0; m.scale.z=0.025f;
      m.text=cv::format("%.3fx%.3fm", canvas_w, canvas_h);
      m.color.r=1.f; m.color.g=1.f; m.color.b=0.f; m.color.a=1.f;
      m.lifetime=life; arr.markers.push_back(m); }

    // -------------------------------------------------------------------------
    // XYZ + RPY overlay -- floating white text above canvas centre
    // Two separate markers so they can be different sizes / positions
    // -------------------------------------------------------------------------
    {
      // Line 1: XYZ translation
      auto m=mk("canvas_pose_label", 10, VM::Marker::TEXT_VIEW_FACING);
      m.pose.position=toGPoint(centroid + Eigen::Vector3d(0, -0.11, 0));
      m.pose.orientation.w=1.0;
      m.scale.z=0.022f;
      m.text=cv::format(
        "X=%+.3f  Y=%+.3f  Z=%.3fm",
        centroid.x(), centroid.y(), centroid.z());
      m.color.r=0.9f; m.color.g=0.9f; m.color.b=0.9f; m.color.a=1.f;
      m.lifetime=life; arr.markers.push_back(m);
    }
    {
      // Line 2: RPY rotation
      auto m=mk("canvas_pose_label", 11, VM::Marker::TEXT_VIEW_FACING);
      m.pose.position=toGPoint(centroid + Eigen::Vector3d(0, -0.14, 0));
      m.pose.orientation.w=1.0;
      m.scale.z=0.022f;
      m.text=cv::format(
        "R=%+.1f  P=%+.1f  Y=%+.1fdeg",
        roll_deg, pitch_deg, yaw_deg);
      m.color.r=0.6f; m.color.g=1.0f; m.color.b=0.6f; m.color.a=1.f;
      m.lifetime=life; arr.markers.push_back(m);
    }

    // Corner spheres + labels
    const std::array<std::string,4> lbl{"TL","TR","BR","BL"};
    const std::array<int,4> ids{marker_id_tl_, marker_id_tr_,
                                 marker_id_br_, marker_id_bl_};
    for (int i=0; i<4; ++i) {
      auto s=mk("canvas_corners",i,VM::Marker::SPHERE);
      s.pose.position=toGPoint(corners3d[i]); s.pose.orientation.w=1.0;
      s.scale.x=s.scale.y=s.scale.z=0.015f;
      s.color.r=1.f; s.color.g=0.2f; s.color.b=0.2f; s.color.a=1.f;
      s.lifetime=life; arr.markers.push_back(s);

      auto t=mk("canvas_labels",i,VM::Marker::TEXT_VIEW_FACING);
      t.pose.position=toGPoint(corners3d[i]+Eigen::Vector3d(0,-0.02,0));
      t.pose.orientation.w=1.0; t.scale.z=0.018f;
      t.text=lbl[i]+cv::format(" ID:%d %.2fm", ids[i], corners3d[i].z());
      t.color.r=t.color.g=t.color.b=t.color.a=1.f;
      t.lifetime=life; arr.markers.push_back(t);

      // Per-marker XYZ axis arrows
      Eigen::Quaterniond mq = Eigen::Quaterniond::Identity();
      if (id_to_corners.count(ids[i]))
        mq = markerOrientation(
          id_to_corners.at(ids[i]), marker_size_m_, camera_matrix_, dist_coeffs_);

      // X -- red
      { auto m=mk("marker_axes", i*3+0, VM::Marker::ARROW);
        m.points.push_back(toGPoint(corners3d[i]));
        m.points.push_back(toGPoint(corners3d[i] + mq*Eigen::Vector3d(0.04,0,0)));
        m.scale.x=0.004f; m.scale.y=0.008f; m.scale.z=0.010f;
        m.color.r=1.f; m.color.g=0.f; m.color.b=0.f; m.color.a=1.f;
        m.lifetime=life; arr.markers.push_back(m); }

      // Y -- green
      { auto m=mk("marker_axes", i*3+1, VM::Marker::ARROW);
        m.points.push_back(toGPoint(corners3d[i]));
        m.points.push_back(toGPoint(corners3d[i] + mq*Eigen::Vector3d(0,0.04,0)));
        m.scale.x=0.004f; m.scale.y=0.008f; m.scale.z=0.010f;
        m.color.r=0.f; m.color.g=1.f; m.color.b=0.f; m.color.a=1.f;
        m.lifetime=life; arr.markers.push_back(m); }

      // Z -- blue
      { auto m=mk("marker_axes", i*3+2, VM::Marker::ARROW);
        m.points.push_back(toGPoint(corners3d[i]));
        m.points.push_back(toGPoint(corners3d[i] + mq*Eigen::Vector3d(0,0,0.04)));
        m.scale.x=0.004f; m.scale.y=0.008f; m.scale.z=0.010f;
        m.color.r=0.f; m.color.g=0.f; m.color.b=1.f; m.color.a=1.f;
        m.lifetime=life; arr.markers.push_back(m); }
    }

    // -------------------------------------------------------------------------
    // RPY arc markers -- 3D LINE_STRIP arcs in canvas frame axes
    //
    // Each arc sweeps the actual RPY angle around the corresponding canvas
    // frame axis, starting from a perpendicular reference direction.
    // Arc radius 8cm -- visible at 0.5-2m range without cluttering the scene.
    //
    // Convention (ZYX extrinsic, ROS standard):
    //   Roll  arc (red)   sweeps around canvas X axis in Y-Z plane
    //   Pitch arc (green) sweeps around canvas Y axis in X-Z plane
    //   Yaw   arc (blue)  sweeps around canvas Z axis in X-Y plane
    // -------------------------------------------------------------------------
    {
      const double arc_r = 0.08;   // 8cm radius
      const double D2R   = M_PI / 180.0;

      // Canvas frame axes in world (camera) coordinates
      Eigen::Vector3d ax_x = q * Eigen::Vector3d(1, 0, 0);  // canvas X
      Eigen::Vector3d ax_y = q * Eigen::Vector3d(0, 1, 0);  // canvas Y
      Eigen::Vector3d ax_z = q * Eigen::Vector3d(0, 0, 1);  // canvas Z

      // --- Roll arc (red) -- sweeps roll_deg around ax_x ---
      // Start from ax_y, sweep toward ax_z
      {
        auto arc = makeRPYArc(fr, st, "rpy_arcs", 20,
          centroid, ax_y, ax_z,
          roll_deg * D2R, arc_r,
          1.f, 0.f, 0.f, life);
        arr.markers.push_back(arc);

        // Radial spoke from origin to arc start
        auto spoke = mk("rpy_arcs", 21, VM::Marker::ARROW);
        spoke.points.push_back(toGPoint(centroid));
        spoke.points.push_back(toGPoint(centroid + arc_r * ax_y));
        spoke.scale.x=0.003f; spoke.scale.y=0.006f; spoke.scale.z=0.0f;
        spoke.color.r=1.f; spoke.color.g=0.f; spoke.color.b=0.f;
        spoke.color.a=0.5f; spoke.lifetime=life;
        arr.markers.push_back(spoke);

        // Label at arc tip
        double t_tip = roll_deg * D2R;
        Eigen::Vector3d tip = centroid
          + arc_r * (std::cos(t_tip)*ax_y + std::sin(t_tip)*ax_z);
        auto lbl = mk("rpy_arcs", 22, VM::Marker::TEXT_VIEW_FACING);
        lbl.pose.position = toGPoint(tip + Eigen::Vector3d(0, -0.015, 0));
        lbl.pose.orientation.w = 1.0; lbl.scale.z = 0.020f;
        lbl.text = cv::format("R=%+.1fdeg", roll_deg);
        lbl.color.r=1.f; lbl.color.g=0.5f; lbl.color.b=0.5f;
        lbl.color.a=1.f; lbl.lifetime=life;
        arr.markers.push_back(lbl);
      }

      // --- Pitch arc (green) -- sweeps pitch_deg around ax_y ---
      // Start from ax_x, sweep toward ax_z
      {
        auto arc = makeRPYArc(fr, st, "rpy_arcs", 23,
          centroid, ax_x, ax_z,
          pitch_deg * D2R, arc_r,
          0.f, 1.f, 0.f, life);
        arr.markers.push_back(arc);

        auto spoke = mk("rpy_arcs", 24, VM::Marker::ARROW);
        spoke.points.push_back(toGPoint(centroid));
        spoke.points.push_back(toGPoint(centroid + arc_r * ax_x));
        spoke.scale.x=0.003f; spoke.scale.y=0.006f; spoke.scale.z=0.0f;
        spoke.color.r=0.f; spoke.color.g=1.f; spoke.color.b=0.f;
        spoke.color.a=0.5f; spoke.lifetime=life;
        arr.markers.push_back(spoke);

        double t_tip = pitch_deg * D2R;
        Eigen::Vector3d tip = centroid
          + arc_r * (std::cos(t_tip)*ax_x + std::sin(t_tip)*ax_z);
        auto lbl = mk("rpy_arcs", 25, VM::Marker::TEXT_VIEW_FACING);
        lbl.pose.position = toGPoint(tip + Eigen::Vector3d(0, -0.015, 0));
        lbl.pose.orientation.w = 1.0; lbl.scale.z = 0.020f;
        lbl.text = cv::format("P=%+.1fdeg", pitch_deg);
        lbl.color.r=0.5f; lbl.color.g=1.f; lbl.color.b=0.5f;
        lbl.color.a=1.f; lbl.lifetime=life;
        arr.markers.push_back(lbl);
      }

      // --- Yaw arc (blue) -- sweeps yaw_deg around ax_z ---
      // Start from ax_x, sweep toward ax_y
      {
        auto arc = makeRPYArc(fr, st, "rpy_arcs", 26,
          centroid, ax_x, ax_y,
          yaw_deg * D2R, arc_r,
          0.2f, 0.5f, 1.f, life);
        arr.markers.push_back(arc);

        auto spoke = mk("rpy_arcs", 27, VM::Marker::ARROW);
        spoke.points.push_back(toGPoint(centroid));
        spoke.points.push_back(toGPoint(centroid + arc_r * ax_x));
        spoke.scale.x=0.003f; spoke.scale.y=0.006f; spoke.scale.z=0.0f;
        spoke.color.r=0.2f; spoke.color.g=0.5f; spoke.color.b=1.f;
        spoke.color.a=0.5f; spoke.lifetime=life;
        arr.markers.push_back(spoke);

        double t_tip = yaw_deg * D2R;
        Eigen::Vector3d tip = centroid
          + arc_r * (std::cos(t_tip)*ax_x + std::sin(t_tip)*ax_y);
        auto lbl = mk("rpy_arcs", 28, VM::Marker::TEXT_VIEW_FACING);
        lbl.pose.position = toGPoint(tip + Eigen::Vector3d(0, -0.015, 0));
        lbl.pose.orientation.w = 1.0; lbl.scale.z = 0.020f;
        lbl.text = cv::format("Y=%+.1fdeg", yaw_deg);
        lbl.color.r=0.5f; lbl.color.g=0.7f; lbl.color.b=1.f;
        lbl.color.a=1.f; lbl.lifetime=life;
        arr.markers.push_back(lbl);
      }
    }

    marker_pub_->publish(arr);
  }

  void deleteMarkers(const std_msgs::msg::Header & h)
  {
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker m;
    m.header=h; m.action=visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(m); marker_pub_->publish(arr);
  }

  // ==========================================================================
  // Debug image
  // ==========================================================================
  void publishDebug(
    cv::Mat & color,
    const cv::Mat & enhanced,
    const std::vector<std::vector<cv::Point2f>> & marker_corners,
    const std::vector<int> & marker_ids,
    const std::vector<cv::Point2f> & canvas_corners,
    const std::vector<Eigen::Vector3d> & pts3d,
    double canvas_w, double canvas_h,
    const std_msgs::msg::Header & header,
    const std::string & status)
  {
    if (!publish_debug_) return;
    rclcpp::Time now = this->get_clock()->now();
    if ((now - last_debug_stamp_).seconds() < 1.0 / debug_fps_) return;
    last_debug_stamp_ = now;

    cv::Mat dbg;
    if (debug_scale_ < 0.99)
      cv::resize(color, dbg, {}, debug_scale_, debug_scale_, cv::INTER_LINEAR);
    else
      dbg = color.clone();
    const float s = (float)debug_scale_;

    std::vector<cv::Point2f> sc;
    for (const auto & c : canvas_corners) sc.emplace_back(c.x*s, c.y*s);

    std::vector<std::vector<cv::Point2f>> sm;
    for (const auto & mc : marker_corners) {
      std::vector<cv::Point2f> row;
      for (const auto & p : mc) row.emplace_back(p.x*s, p.y*s);
      sm.push_back(row);
    }

    // CLAHE inset
    { int iw=dbg.cols/4, ih=dbg.rows/4;
      cv::Mat ins, inb;
      cv::resize(enhanced, ins, {iw,ih});
      cv::cvtColor(ins, inb, cv::COLOR_GRAY2BGR);
      cv::Rect roi(dbg.cols-iw-4,4,iw,ih);
      inb.copyTo(dbg(roi));
      cv::rectangle(dbg,roi,{255,255,0},1);
      cv::putText(dbg,"CLAHE",{dbg.cols-iw,ih+14},
        cv::FONT_HERSHEY_SIMPLEX,0.35,{255,255,0},1); }

    if (!marker_ids.empty())
      cv::aruco::drawDetectedMarkers(dbg, sm, marker_ids);

    if (status == "ok" && sc.size() == 4) {
      const std::vector<std::string> lbl{"TL","TR","BR","BL"};
      for (int i=0; i<4; ++i) {
        cv::line(dbg, sc[i], sc[(i+1)%4], {0,255,0}, 1);
        cv::circle(dbg, sc[i], 4, {0,255,255}, -1);
        std::string label = lbl[i];
        if (!pts3d.empty()) label += cv::format(" %.2fm", pts3d[i].z());
        cv::putText(dbg, label, {(int)sc[i].x+5,(int)sc[i].y-5},
          cv::FONT_HERSHEY_SIMPLEX, 0.35, {0,255,255}, 1);
        if (!pts3d.empty()) {
          double elen = (pts3d[(i+1)%4]-pts3d[i]).norm();
          cv::Point2f mid = (sc[i]+sc[(i+1)%4])*0.5f;
          cv::putText(dbg, cv::format("%.2fm",elen),
            {(int)mid.x+3,(int)mid.y-3},
            cv::FONT_HERSHEY_SIMPLEX, 0.30, {0,255,255}, 1);
        }
      }
      cv::putText(dbg,
        cv::format("DETECTED %.2fx%.2fm", canvas_w, canvas_h),
        {10,22}, cv::FONT_HERSHEY_SIMPLEX, 0.55, {0,255,0}, 1);

    } else if (status == "missing") {
      cv::putText(dbg,
        cv::format("Missing IDs: %d %d %d %d",
          marker_id_tl_,marker_id_tr_,marker_id_br_,marker_id_bl_),
        {10,22}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,165,255}, 1);
      cv::putText(dbg,
        cv::format("Detected %zu -- raise clahe_clip if dark",marker_ids.size()),
        {10,42}, cv::FONT_HERSHEY_SIMPLEX, 0.45, {0,165,255}, 1);
    } else if (status == "bad_depth") {
      cv::putText(dbg, "Markers found but depth invalid",
        {10,22}, cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,165,255}, 1);
      cv::putText(dbg, "Increase depth_sample_half or move closer",
        {10,42}, cv::FONT_HERSHEY_SIMPLEX, 0.45, {0,165,255}, 1);
    }

    debug_pub_->publish(*cv_bridge::CvImage(header,"bgr8",dbg).toImageMsg());
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanvasPoseDetector>());
  rclcpp::shutdown();
  return 0;
}