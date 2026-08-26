#include "ROSutils.hpp"

#include <algorithm>
#include <deque>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace ros2wrap {

using geometry_msgs::msg::PoseWithCovarianceStamped;
using fast_limo::srv::SendPointCloud;


class RelocaWrapper : public rclcpp::Node
{
public:
  RelocaWrapper()
  : Node("fast_limo_reloca",
         rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
  {
    // Unique Relocator
    Relocator& RELOCA = Relocator::getInstance();

    // Load params
    loadConfig(&cfg_);

    // Downsample filter
    voxel_filter.setLeafSize(cfg_.downsample_leaf, cfg_.downsample_leaf, cfg_.downsample_leaf);

    // Subscribers
    rclcpp::SubscriptionOptions lidar_opt, state_opt, init_opt;
    lidar_opt.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    state_opt.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    init_opt.callback_group  = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/fast_limo/final_raw", 1,
      std::bind(&RelocaWrapper::lidar_callback, this, std::placeholders::_1),
      lidar_opt);

    state_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/fast_limo/state", 10,
      std::bind(&RelocaWrapper::state_callback, this, std::placeholders::_1),
      state_opt);

    initialpose_sub_ = create_subscription<PoseWithCovarianceStamped>(
      "/initialpose", 1,
      std::bind(&RelocaWrapper::initialpose_callback, this, std::placeholders::_1),
      init_opt);

    // Publishers
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
    qos.best_effort();
    full_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("/fast_limo/full_map", qos);
    
    // TF buffer and listener
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Dynamic TF for dummy (identity) map -> odom transform (published until Relocator is relocated)
    dynamic_tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

    // The map -> odom transform is immutable after this one-shot relocation.
    static_tf_broadcaster_ =
      std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);
    

    // Service client -> send map to main node once relocated
    pc_client_ = create_client<SendPointCloud>("/fast_limo/send_pointcloud");

    // Init RELOCA
    RELOCA.init(cfg_);

    map_timer_ = create_wall_timer(
      std::chrono::milliseconds(5000),
      std::bind(&RelocaWrapper::publish_map, this));

    map_sent_ = false;
    tf_sent_ = false;
  }

private:
  // ============================ Callbacks ============================
  void lidar_callback(const sensor_msgs::msg::PointCloud2 & msg)
  {
    if(map_sent_) return; 

    pcl::PointCloud<PointType>::Ptr pc (fast_limo::make_shared<pcl::PointCloud<PointType>>());
    pcl::fromROSMsg(msg, *pc);

    auto& reloca = Relocator::getInstance();
    if (!reloca.is_relocated()) {
      reloca.updateCloud(pc);
    }

    if (!map_sent_ && reloca.is_relocated()) {
      publish_static_tf();
      call_send_pointcloud_service();
    }else{
      publish_dynamic_tf();
    }

  }

  void state_callback(const nav_msgs::msg::Odometry & msg)
  {
    if(map_sent_) return; 

    store_odom_pose(msg);

    fast_limo::State st;
    fromROStoLimo(msg, st);
    auto& reloca = Relocator::getInstance();
    reloca.updateState(st);

  }

  void initialpose_callback(const PoseWithCovarianceStamped & msg)
  {
    if (map_sent_) return;

    // Convert the initial pose to the expected map frame if necessary.
    PoseWithCovarianceStamped initial_pose = msg;

    if (msg.header.frame_id.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "Ignoring /initialpose: frame_id is empty");
      return;
    }

    if (msg.header.frame_id != map_frame_) {
      try {
        initial_pose = tf_buffer_->transform(
          msg,
          map_frame_,
          tf2::durationFromSec(initialpose_tf_timeout_s_));

        RCLCPP_INFO(
          get_logger(),
          "Transformed /initialpose from '%s' to '%s'",
          msg.header.frame_id.c_str(),
          map_frame_.c_str());
      } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(
          get_logger(),
          "Ignoring /initialpose: could not transform from '%s' to '%s': %s",
          msg.header.frame_id.c_str(),
          map_frame_.c_str(),
          ex.what());
        return;
      }
    }

    Eigen::Matrix4f odom_to_base = Eigen::Matrix4f::Identity();

    if (!interpolate_odom_pose(
        rclcpp::Time(msg.header.stamp), odom_to_base))
    {
      RCLCPP_WARN(
        get_logger(),
        "Ignoring /initialpose: no synchronized odometry pose within %.3f s",
        initialpose_sync_tolerance_s_);
      return;
    }

    auto& reloca = Relocator::getInstance();

    const Eigen::Vector3f map_position(
      static_cast<float>(initial_pose.pose.pose.position.x),
      static_cast<float>(initial_pose.pose.pose.position.y),
      static_cast<float>(initial_pose.pose.pose.position.z));

    Eigen::Quaternionf map_orientation(
      static_cast<float>(initial_pose.pose.pose.orientation.w),
      static_cast<float>(initial_pose.pose.pose.orientation.x),
      static_cast<float>(initial_pose.pose.pose.orientation.y),
      static_cast<float>(initial_pose.pose.pose.orientation.z));

    if (map_orientation.norm() < 1.0e-6f) {
      RCLCPP_ERROR(
        get_logger(),
        "Ignoring /initialpose: invalid zero-norm quaternion");
      return;
    }

    map_orientation.normalize();

    Eigen::Matrix4f map_to_base = Eigen::Matrix4f::Identity();
    map_to_base.block<3, 3>(0, 0) = map_orientation.toRotationMatrix();
    map_to_base.block<3, 1>(0, 3) = map_position;

    reloca.updateInitialPose(map_to_base, odom_to_base);

    RCLCPP_INFO(
      get_logger(),
      "Synchronized pose prior received at map position [%.3f, %.3f, %.3f]",
      map_position.x(),
      map_position.y(),
      map_position.z());
  }

  // ============================ Service call ============================
  void call_send_pointcloud_service()
  {
    if (!pc_client_->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_WARN(get_logger(), "send_pointcloud service not available yet");
      return;
    }

    auto& reloca = Relocator::getInstance();

    pcl::PointCloud<PointType>::Ptr map(new pcl::PointCloud<PointType>);
    reloca.get_full_map_transformed(map);

    auto req = std::make_shared<SendPointCloud::Request>();
    pcl::toROSMsg(*map, req->pointcloud);
    req->pointcloud.header.frame_id = map_frame_;
    req->pointcloud.header.stamp = now();

    auto future = pc_client_->async_send_request(req);
    try {
      auto res = future.get();
      if (res->success) {
        RCLCPP_INFO(get_logger(), "Relocation completed successfully :)");
        map_sent_ = true;
      } else {
        RCLCPP_WARN(get_logger(), "Map sent but server reported failure.");
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to call send_pointcloud: %s", e.what());
    }
  }

  // ============================ TF + full map ============================
  void publish_dynamic_tf()
  {
    if (tf_sent_) {
      return;  // Static TF has already been published
    }

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now();
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id = world_frame_;

    tf_msg.transform.translation.x = 0.0;
    tf_msg.transform.translation.y = 0.0;
    tf_msg.transform.translation.z = 0.0;

    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = 0.0;
    tf_msg.transform.rotation.w = 1.0;

    dynamic_tf_broadcaster_->sendTransform(tf_msg);
  }

  void publish_static_tf()
  {
    if (tf_sent_) return;

    auto& reloca = Relocator::getInstance();

    Eigen::Vector3f p = reloca.get_pose();
    Eigen::Quaternionf q = reloca.get_orientation();

    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp = now();
    tf_msg.header.frame_id = map_frame_;
    tf_msg.child_frame_id = world_frame_;
    tf_msg.transform.translation.x = p.x();
    tf_msg.transform.translation.y = p.y();
    tf_msg.transform.translation.z = p.z();
    tf_msg.transform.rotation.x = q.x();
    tf_msg.transform.rotation.y = q.y();
    tf_msg.transform.rotation.z = q.z();
    tf_msg.transform.rotation.w = q.w();

    static_tf_broadcaster_->sendTransform(tf_msg);
    tf_sent_ = true;
    RCLCPP_INFO(
      get_logger(),
      "Published static TF %s -> %s after accepted relocation",
      map_frame_.c_str(),
      world_frame_.c_str());
  }

  void publish_map()
  {
    auto& reloca = Relocator::getInstance();

    // Publish full map (untransformed)
    pcl::PointCloud<PointType>::Ptr full_map(new pcl::PointCloud<PointType>);
    reloca.get_full_map(full_map);

    voxel_filter.setInputCloud(full_map);
    voxel_filter.filter(*full_map);

    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*full_map, msg);
    msg.header.frame_id = map_frame_;
    msg.header.stamp = now();
    full_map_pub_->publish(msg);
  }

  // ============================ Utils ============================
  void fromROStoLimo(const nav_msgs::msg::Odometry& in, fast_limo::State& out)
  {
    // Only pose & twist are needed by Relocator to count distance
    out.time = rclcpp::Time(in.header.stamp).seconds();

    out.p = Eigen::Vector3f(
      static_cast<float>(in.pose.pose.position.x),
      static_cast<float>(in.pose.pose.position.y),
      static_cast<float>(in.pose.pose.position.z));

    Eigen::Quaterniond qd(
      in.pose.pose.orientation.w,
      in.pose.pose.orientation.x,
      in.pose.pose.orientation.y,
      in.pose.pose.orientation.z);
    out.q = qd.cast<float>();

    out.v = Eigen::Vector3f(
      static_cast<float>(in.twist.twist.linear.x),
      static_cast<float>(in.twist.twist.linear.y),
      static_cast<float>(in.twist.twist.linear.z));

    out.w = Eigen::Vector3f(
      static_cast<float>(in.twist.twist.angular.x),
      static_cast<float>(in.twist.twist.angular.y),
      static_cast<float>(in.twist.twist.angular.z));
  }

  struct TimedOdomPose
  {
    int64_t stamp_ns = 0;
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
  };

  void store_odom_pose(const nav_msgs::msg::Odometry& msg)
  {
    TimedOdomPose pose;
    pose.stamp_ns = rclcpp::Time(msg.header.stamp).nanoseconds();
    if (pose.stamp_ns == 0) pose.stamp_ns = now().nanoseconds();

    pose.position = Eigen::Vector3f(
      static_cast<float>(msg.pose.pose.position.x),
      static_cast<float>(msg.pose.pose.position.y),
      static_cast<float>(msg.pose.pose.position.z));
    pose.orientation = Eigen::Quaternionf(
      static_cast<float>(msg.pose.pose.orientation.w),
      static_cast<float>(msg.pose.pose.orientation.x),
      static_cast<float>(msg.pose.pose.orientation.y),
      static_cast<float>(msg.pose.pose.orientation.z));
    if (pose.orientation.norm() < 1.0e-6f) {
      pose.orientation = Eigen::Quaternionf::Identity();
    } else {
      pose.orientation.normalize();
    }

    std::lock_guard<std::mutex> lock(odom_history_mutex_);

    auto insertion_point = std::upper_bound(
      odom_history_.begin(),
      odom_history_.end(),
      pose.stamp_ns,
      [](int64_t stamp, const TimedOdomPose& item) {
        return stamp < item.stamp_ns;
      });
    odom_history_.insert(insertion_point, pose);

    const int64_t oldest_allowed =
      pose.stamp_ns - static_cast<int64_t>(odom_history_duration_s_ * 1.0e9);
    while (!odom_history_.empty() &&
           odom_history_.front().stamp_ns < oldest_allowed)
    {
      odom_history_.pop_front();
    }
  }

  bool interpolate_odom_pose(
    const rclcpp::Time& requested_stamp,
    Eigen::Matrix4f& odom_to_base)
  {
    std::lock_guard<std::mutex> lock(odom_history_mutex_);
    if (odom_history_.empty()) return false;

    int64_t requested_ns = requested_stamp.nanoseconds();
    if (requested_ns == 0) requested_ns = odom_history_.back().stamp_ns;

    auto upper = std::lower_bound(
      odom_history_.begin(),
      odom_history_.end(),
      requested_ns,
      [](const TimedOdomPose& item, int64_t stamp) {
        return item.stamp_ns < stamp;
      });

    TimedOdomPose synchronized;
    const int64_t tolerance_ns =
      static_cast<int64_t>(initialpose_sync_tolerance_s_ * 1.0e9);

    if (upper == odom_history_.begin()) {
      if (std::abs(upper->stamp_ns - requested_ns) > tolerance_ns) return false;
      synchronized = *upper;
    } else if (upper == odom_history_.end()) {
      const TimedOdomPose& latest = odom_history_.back();
      if (std::abs(latest.stamp_ns - requested_ns) > tolerance_ns) return false;
      synchronized = latest;
    } else {
      const TimedOdomPose& after = *upper;
      const TimedOdomPose& before = *(upper - 1);
      const int64_t before_gap = requested_ns - before.stamp_ns;
      const int64_t after_gap = after.stamp_ns - requested_ns;
      if (before_gap > tolerance_ns || after_gap > tolerance_ns) return false;

      const double interval =
        static_cast<double>(after.stamp_ns - before.stamp_ns);
      const float alpha = interval > 0.0
        ? static_cast<float>(before_gap / interval)
        : 0.0f;

      synchronized.stamp_ns = requested_ns;
      synchronized.position =
        (1.0f - alpha) * before.position + alpha * after.position;
      synchronized.orientation =
        before.orientation.slerp(alpha, after.orientation).normalized();
    }

    odom_to_base = Eigen::Matrix4f::Identity();
    odom_to_base.block<3, 3>(0, 0) =
      synchronized.orientation.toRotationMatrix();
    odom_to_base.block<3, 1>(0, 3) = synchronized.position;
    return true;
  }

// ============================ Params ============================
  template <typename T>
  T get_or_declare_parameter(const std::string& name, const T& default_value)
  {
    if (!this->has_parameter(name)) {
      this->declare_parameter<T>(name, default_value);
    }

    T value;
    this->get_parameter(name, value);
    return value;
  }

  void loadConfig(fast_limo::RelocaConfig* cfg)
  {
    // Reloca-specific
    cfg->mode = get_or_declare_parameter<bool>(
      "mode",
      false);

    cfg->map_path = get_or_declare_parameter<std::string>(
      "map_path",
      "");

    cfg->distance_threshold = static_cast<float>(
      get_or_declare_parameter<double>("distance_threshold", 10.0));

    cfg->downsample_leaf = static_cast<float>(
      get_or_declare_parameter<double>("downsample_leaf", 0.5));

    cfg->inliers_threshold = get_or_declare_parameter<int>(
      "inliers_threshold",
      5);

    cfg->score = get_or_declare_parameter<double>(
      "score",
      10000.0);

    cfg->prior_distance_threshold = static_cast<float>(
      get_or_declare_parameter<double>("prior.distance_threshold", 2.5));
    cfg->prior_crop_margin = static_cast<float>(
      get_or_declare_parameter<double>("prior.crop_margin", 6.0));
    cfg->prior_voxel = static_cast<float>(
      get_or_declare_parameter<double>("prior.voxel", 0.3));
    cfg->prior_max_correspondence = static_cast<float>(
      get_or_declare_parameter<double>("prior.max_correspondence", 1.0));
    cfg->prior_max_iterations = get_or_declare_parameter<int>(
      "prior.max_iterations",
      64);
    cfg->prior_max_fitness_score = get_or_declare_parameter<double>(
      "prior.max_fitness_score",
      1.0);

    initialpose_sync_tolerance_s_ = get_or_declare_parameter<double>(
      "initialpose_sync_tolerance",
      0.25);
    odom_history_duration_s_ = get_or_declare_parameter<double>(
      "odom_history_duration",
      30.0);

    // Frames
    map_frame_ = get_or_declare_parameter<std::string>(
      "frames.map",
      "map");

    world_frame_ = get_or_declare_parameter<std::string>(
      "frames.world",
      "odom");
  }
  

  // ============================ Members ============================
  fast_limo::RelocaConfig cfg_;

  // I/O
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       state_sub_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr     initialpose_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    full_map_pub_;
  rclcpp::Client<SendPointCloud>::SharedPtr                      pc_client_;

  std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> dynamic_tf_broadcaster_;

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Timer
  rclcpp::TimerBase::SharedPtr map_timer_;

  // Downsampling
  pcl::VoxelGrid<PointType> voxel_filter;

  // Params / names
  std::string map_frame_, world_frame_;

  std::deque<TimedOdomPose> odom_history_;
  std::mutex odom_history_mutex_;
  double initialpose_sync_tolerance_s_ = 0.25;
  double odom_history_duration_s_ = 30.0;

  double initialpose_tf_timeout_s_{2.0};

  bool map_sent_;
  bool tf_sent_;
};

} // namespace ros2wrap

int main(int argc, char** argv) { 
  rclcpp::init(argc, argv); 
  auto node = std::make_shared<ros2wrap::RelocaWrapper>(); 
  rclcpp::executors::MultiThreadedExecutor ex; 
  ex.add_node(node); 
  ex.spin(); 
  rclcpp::shutdown(); 
  return 0;
}
