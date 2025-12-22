#include "ROSutils.hpp"

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

    // Subscribers
    rclcpp::SubscriptionOptions lidar_opt, state_opt, init_opt;
    lidar_opt.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    state_opt.callback_group = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    init_opt.callback_group  = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    lidar_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_, 1,
      std::bind(&RelocaWrapper::lidar_callback, this, std::placeholders::_1),
      lidar_opt);

    state_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      state_topic_, 10,
      std::bind(&RelocaWrapper::state_callback, this, std::placeholders::_1),
      state_opt);

    initialpose_sub_ = create_subscription<PoseWithCovarianceStamped>(
      "/initialpose", 1,
      std::bind(&RelocaWrapper::initialpose_callback, this, std::placeholders::_1),
      init_opt);

    // Publishers
    full_map_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("full_map", 1);

    // TF
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    // Service client -> send map to main node once relocated
    pc_client_ = create_client<SendPointCloud>(send_pc_srv_name_);

    // Init RELOCA
    RELOCA.init(cfg_);

    // 10 Hz timer: broadcast TF + publish full map
    timer_ = create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&RelocaWrapper::tick, this));

    map_sent_ = false;
  }

private:
  // ============================ Callbacks ============================
  void lidar_callback(const sensor_msgs::msg::PointCloud2 & msg)
  {
    if(map_sent_) return; 

    pcl::PointCloud<PointType>::Ptr pc (std::make_shared<pcl::PointCloud<PointType>>());
    pcl::fromROSMsg(msg, *pc);

    auto& reloca = Relocator::getInstance();
    reloca.updateCloud(pc);

    if (!map_sent_ && reloca.is_relocated()) {
      call_send_pointcloud_service();
    }

  }

  void state_callback(const nav_msgs::msg::Odometry & msg)
  {
    if(map_sent_) return; 

    fast_limo::State st;
    fromROStoLimo(msg, st);
    auto& reloca = Relocator::getInstance();
    reloca.updateState(st);

  }

  void initialpose_callback(const PoseWithCovarianceStamped & msg)
  {

    if(map_sent_) return; 

    auto& reloca = Relocator::getInstance();
    std::vector<double> init_state = {
      msg.pose.pose.position.x,
      msg.pose.pose.position.y,
      msg.pose.pose.position.z,
      0.0, 0.0, 0.0
    };
    reloca.updateInitialPose(init_state);
  }

  // ============================ Periodic ============================
  void tick()
  {
    tf_broadcast_and_publish_full_map();
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
  void tf_broadcast_and_publish_full_map()
  {
    auto& reloca = Relocator::getInstance();

    Eigen::Vector3f p = reloca.get_pose();
    Eigen::Quaternionf q = reloca.get_orientation();

    // Build fast_limo::State from pose (as in ROS1)
    Eigen::Matrix4f T = Eigen::Matrix4f::Identity();
    T.block<3,3>(0,0) = q.toRotationMatrix();
    T.block<3,1>(0,3) = p;
    fast_limo::State state(T);

    // Broadcast TF (map_frame_ -> world_frame_)
    broadcastTF(state, map_frame_, world_frame_, true);

    // Publish full map (untransformed)
    pcl::PointCloud<PointType>::Ptr full_map(new pcl::PointCloud<PointType>);
    reloca.get_full_map(full_map);
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

  void broadcastTF(const fast_limo::State& in, const std::string& parent, const std::string& child, bool use_now)
  {
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header.stamp    = use_now ? now() : rclcpp::Time(in.time);
    tf_msg.header.frame_id = parent;
    tf_msg.child_frame_id  = child;

    tf_msg.transform.translation.x = in.p.cast<double>()[0];
    tf_msg.transform.translation.y = in.p.cast<double>()[1];
    tf_msg.transform.translation.z = in.p.cast<double>()[2];

    Eigen::Quaterniond qd = in.q.cast<double>();
    tf_msg.transform.rotation.x = qd.x();
    tf_msg.transform.rotation.y = qd.y();
    tf_msg.transform.rotation.z = qd.z();
    tf_msg.transform.rotation.w = qd.w();

    tf_broadcaster_->sendTransform(tf_msg);
  }

  // ============================ Params ============================
  void loadConfig(fast_limo::RelocaConfig* cfg)
  {
    // Reloca-specific
    cfg->mode               = get_parameter("mode").as_bool();
    cfg->map_path           = get_parameter("map_path").as_string();
    cfg->distance_threshold = static_cast<float>(get_parameter("distance_threshold").as_double());
    cfg->inliers_threshold  = get_parameter("inliers_threshold").as_int();
    cfg->score              = get_parameter("score").as_double();

    // Topics (all taken from fast_limo main namespace to keep compatibility)
    lidar_topic_  = declare_parameter<std::string>("topics.input.lidar", "/fast_limo/final_raw");
    state_topic_  = declare_parameter<std::string>("topics.state", "/fast_limo/state");
    send_pc_srv_name_ = declare_parameter<std::string>("services.send_pointcloud", "/fast_limo/send_pointcloud");

    // Frames
    map_frame_   = declare_parameter<std::string>("frames.map",   "ona2/map");
    world_frame_ = declare_parameter<std::string>("frames.world", "ona2/odom");
  }

  // ============================ Members ============================
  fast_limo::RelocaConfig cfg_;

  // I/O
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr       state_sub_;
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr     initialpose_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr    full_map_pub_;
  rclcpp::Client<SendPointCloud>::SharedPtr                      pc_client_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Timer
  rclcpp::TimerBase::SharedPtr timer_;

  // Params / names
  std::string lidar_topic_, state_topic_, send_pc_srv_name_;
  std::string map_frame_, world_frame_;

  bool map_sent_;
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

