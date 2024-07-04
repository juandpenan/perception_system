/*
Copyright (c) 2024 José Miguel Guerrero Hernández

Licensed under the Attribution-ShareAlike 4.0 International (CC BY-SA 4.0) License;
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    https://creativecommons.org/licenses/by-sa/4.0/

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#ifndef PERCEPTION_SYSTEM__PERCEPTION_LISTENER_HPP_
#define PERCEPTION_SYSTEM__PERCEPTION_LISTENER_HPP_

#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

#include "perception_system_interfaces/msg/detection.hpp"
#include "perception_system_interfaces/msg/detection_array.hpp"
#include "leg_tracker_ros2/msg/leg_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "state_observers/kalman_filter.hpp"


namespace perception_system
{

using ApproximateSyncPolicy = message_filters::sync_policies::ApproximateTime<
  perception_system_interfaces::msg::DetectionArray,
  leg_tracker_ros2::msg::LegArray>;

struct PerceptionData
{
  std::string type;
  perception_system_interfaces::msg::Detection msg;
  rclcpp::Time time;
};

struct PerceptionInterest
{
  bool status;
  rclcpp::Time time;
};

struct HumanTracked
{
  // std::string id;
  // std::string type;
  double x, y, z, confidence;
  rclcpp::Time time;
  Eigen::MatrixXd A{6, 6};
  Eigen::MatrixXd B;
  Eigen::MatrixXd C;
  Eigen::MatrixXd Q;
  Eigen::MatrixXd R;
  double dT{30.0};
  std::shared_ptr<state_observer::KalmanFilter> kf;
  perception_system_interfaces::msg::Detection msg;
  // leg_tracker_ros2::msg::LegArray leg_msg;
  
  HumanTracked(perception_system_interfaces::msg::Detection msg,
               double dT, 
               rclcpp::Time now) :                
                time(now),
                dT(dT)
  {
    A <<  1, 0, 0, dT, 0, 0,  // x
          0, 1, 0, 0, dT, 0,  // y
          0, 0, 1, 0, 0, dT,  // z
          0, 0, 0, 1, 0, 0,  // x_dot
          0, 0, 0, 0, 1, 0,  // y_dot
          0, 0, 0, 0, 0, 1;  // z_dot
    B = Eigen::MatrixXd::Zero(6, 1);
    C = Eigen::MatrixXd::Zero(3, 6);
    C.diagonal().setOnes();
    Eigen::MatrixXd::Identity(6, 6);
    Eigen::MatrixXd::Identity(3, 3);

    Q = Eigen::MatrixXd::Identity(6, 6);
    R = Eigen::MatrixXd::Identity(3, 3);

    kf = std::make_shared<state_observer::KalmanFilter>(A, B, C, Q, R);
    Eigen::VectorXd x0(6);
    x = msg.center3d.position.x;
    y = msg.center3d.position.y;
    z = msg.center3d.position.z;
    x0 << x, y, z, 0, 0, 0;
    kf->initialize(x0);
  }
  void update(perception_system_interfaces::msg::Detection msg, rclcpp::Time now)
  {
    Eigen::VectorXd measurement(3);
    measurement << msg.center3d.position.x, msg.center3d.position.y, msg.center3d.position.z;
    kf->update(measurement);  
    this->x = kf->get_output()(0);
    this->y = kf->get_output()(1);
    this->z = kf->get_output()(2);
    this->msg.center3d.position.x = msg.center3d.position.x;
    this->msg.center3d.position.y = msg.center3d.position.y;
    this->msg.center3d.position.z = msg.center3d.position.z;
    time = now;
  }

};

class PerceptionListener
{
public:
  static std::shared_ptr<PerceptionListener> getInstance(
    std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> parent_node)
  {
    if (uniqueInstance_ == nullptr) {
      uniqueInstance_ = std::make_shared<PerceptionListener>(parent_node);
    }
    return uniqueInstance_;
  }

  explicit PerceptionListener(std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> parent_node);

  virtual ~PerceptionListener() {}

  void update(double hz = 30);

  void set_interest(const std::string & type, bool status = true);
  std::vector<perception_system_interfaces::msg::Detection> get_by_id(const std::string & id);
  std::vector<perception_system_interfaces::msg::Detection> get_by_type(const std::string & type);
  // directly publish the TF
  int publicTF(
    const perception_system_interfaces::msg::Detection & detected_object,
    const std::string & custom_suffix = "");
  void publicTFinterest();
  // public tfs with custom sorting
  void publicSortedTFinterest(
    std::function<bool(const perception_system_interfaces::msg::Detection &,
    const perception_system_interfaces::msg::Detection &)> comp = [] (const perception_system_interfaces::msg::Detection & a, const perception_system_interfaces::msg::Detection & b) {
      // Default sorting behavior
      return a.center3d.position.z < b.center3d.position.z;
    });


private:
  static std::shared_ptr<PerceptionListener> uniqueInstance_;
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> parent_node_;

  rclcpp::Subscription<perception_system_interfaces::msg::DetectionArray>::SharedPtr percept_sub_;
  message_filters::Subscriber<leg_tracker_ros2::msg::LegArray,
    rclcpp_lifecycle::LifecycleNode> leg_sub_;
  message_filters::Subscriber<perception_system_interfaces::msg::DetectionArray,
    rclcpp_lifecycle::LifecycleNode> detection_sub_;
  std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> sync_;

  std::map<std::string, PerceptionInterest> interests_;
  std::map<std::string, PerceptionData> perceptions_;
  std::map<std::string, HumanTracked> humans_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  perception_system_interfaces::msg::DetectionArray::UniquePtr last_msg_;

  void perception_callback(perception_system_interfaces::msg::DetectionArray::UniquePtr msg);
  void sync_cb(
    const perception_system_interfaces::msg::DetectionArray::ConstSharedPtr & detection_msg,
    const leg_tracker_ros2::msg::LegArray::ConstSharedPtr & leg_msg);
  void classical_update_(double hz = 30);
  void update_with_yolo_(double hz = 30);
  void initialize_filter_(
    const geometry_msgs::msg::PointStamped & entity);

  geometry_msgs::msg::PointStamped _update_filter(
    const geometry_msgs::msg::PointStamped & entity);

  double max_time_perception_;
  double max_time_interest_;
  rclcpp::Time last_update_;

  std::string tf_frame_camera_;
  std::string tf_frame_map_;
  bool use_people_filter_, use_multiple_sources_filter_;
  std::string source_topic_;

  std::shared_ptr<state_observer::KalmanFilter> kf_;

  double lambda_, dT_{30.0};
  bool state_obs_initialized_ = false;
  bool update_dt_ = false;
};

}  // namespace perception_system

std::shared_ptr<perception_system::PerceptionListener> perception_system::PerceptionListener::
uniqueInstance_;


#endif  // PERCEPTION_SYSTEM__PERCEPTION_LISTENER_HPP_
