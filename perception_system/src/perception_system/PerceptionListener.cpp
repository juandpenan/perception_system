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


#include "rclcpp/rclcpp.hpp"
#include "rclcpp_cascade_lifecycle/rclcpp_cascade_lifecycle.hpp"

#include "perception_system_interfaces/msg/detection.hpp"
#include "perception_system_interfaces/msg/detection_array.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <tf2/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "perception_system/PerceptionListener.hpp"


namespace perception_system
{
using namespace std::chrono_literals;

PerceptionListener::PerceptionListener(
  std::shared_ptr<rclcpp_cascade_lifecycle::CascadeLifecycleNode> parent_node)
: parent_node_(parent_node)
{
  parent_node_->declare_parameter("max_time_perception", 0.01);
  parent_node_->declare_parameter("max_time_interest", 0.01);
  parent_node_->declare_parameter("debug", false);
  parent_node_->declare_parameter("tf_frame_camera", "head_front_camera_link_color_optical_frame");
  parent_node_->declare_parameter("tf_frame_map", "base_footprint");
  parent_node_->declare_parameter("use_people_filter", false);
  parent_node_->declare_parameter("use_multiple_sources_filter", false);
  parent_node_->declare_parameter("source_topic", "detected_leg_clusters");

  parent_node_->get_parameter("max_time_perception", max_time_perception_);  
  parent_node_->get_parameter("max_time_interest", max_time_interest_);
  parent_node_->get_parameter("tf_frame_camera", tf_frame_camera_);
  parent_node_->get_parameter("tf_frame_map", tf_frame_map_);
  parent_node_->get_parameter("use_people_filter", use_people_filter_);
  parent_node_->get_parameter("use_multiple_sources_filter", use_multiple_sources_filter_);
  parent_node_->get_parameter("source_topic", source_topic_);
  if (parent_node_->get_parameter("debug").as_bool()) {
    parent_node_->add_activation("yolov8_debug_node");
  }

  last_update_ = rclcpp::Clock(RCL_STEADY_TIME).now();

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(parent_node_->get_clock(), 120ms); // 30 Hz liveliness so it is always updated

  tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(parent_node_.get());

  std::string topic_name = "all_perceptions";
  if (!use_people_filter_ && !use_multiple_sources_filter_) {

    percept_sub_ = parent_node_->create_subscription<perception_system_interfaces::msg::DetectionArray>(
      topic_name, 10,
      std::bind(&PerceptionListener::perception_callback, this, std::placeholders::_1));

  } else if (use_people_filter_ && !use_multiple_sources_filter_) {

    percept_sub_ = parent_node_->create_subscription<perception_system_interfaces::msg::DetectionArray>(
      topic_name, 10,
      std::bind(&PerceptionListener::perception_callback, this, std::placeholders::_1));

    // Eigen::MatrixXd A(6, 6);
    // A <<  1, 0, 0, dT_, 0, 0,  // x
    //       0, 1, 0, 0, dT_, 0,  // y
    //       0, 0, 1, 0, 0, dT_,  // z
    //       0, 0, 0, 1, 0, 0,  // x_dot
    //       0, 0, 0, 0, 1, 0,  // y_dot
    //       0, 0, 0, 0, 0, 1;  // z_dot
    // Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6, 1);
    // Eigen::MatrixXd C = Eigen::MatrixXd::Zero(3, 6);
    // C.diagonal().setOnes();

    // Eigen::MatrixXd Q = Eigen::MatrixXd::Identity(6, 6);
    // Eigen::MatrixXd R = Eigen::MatrixXd::Identity(3, 3);

    // kf_ = std::make_shared<state_observer::KalmanFilter>(A, B, C, Q, R);

  } else if (use_people_filter_ && use_multiple_sources_filter_) { 

    leg_sub_.subscribe(parent_node_->shared_from_this(), source_topic_);
    detection_sub_.subscribe(parent_node_->shared_from_this(), topic_name);

    sync_ = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(20);
    sync_->connectInput(detection_sub_, leg_sub_);
    sync_->registerCallback(
      std::bind(&PerceptionListener::sync_cb, this, std::placeholders::_1, std::placeholders::_2));

  }
}

void PerceptionListener::perception_callback(
  perception_system_interfaces::msg::DetectionArray::UniquePtr msg)
{
  last_msg_ = std::move(msg);
}

void PerceptionListener::sync_cb(
  const perception_system_interfaces::msg::DetectionArray::ConstSharedPtr & detection_msg,
  const leg_tracker_ros2::msg::LegArray::ConstSharedPtr & leg_msg)
{
  //todo
}

// Check if last_perceptions_ elements are too old, and remove it
// Check if interests are too old, and call set_interest( , false)
void PerceptionListener::update(double hz)
{
  if (use_people_filter_ && !use_multiple_sources_filter_) {

    update_with_yolo_(hz);
  }
  else if (use_people_filter_ && use_multiple_sources_filter_) {
    //todo
  }
  else {
    classical_update_(hz);
  }

}

// if status == true, add to interests_ and call add_activation
// if status == false, remove to interests_ and call remove_activation
void PerceptionListener::set_interest(const std::string & id, bool status)
{
  if (status) {
    if (interests_.find(id) == interests_.end()) {
      rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      interests_.insert(
        std::pair<std::string, PerceptionInterest>(
          id,
          {status, steady_clock.now()}));
      if (id.find("person") != std::string::npos) {
        parent_node_->add_activation("perception_people_detection");
      } else {
        parent_node_->add_activation("perception_objects_detection");
      }
      RCLCPP_INFO(parent_node_->get_logger(), "Added interest: %s, %d", id.c_str(), status);
    } else {
      rclcpp::Clock steady_clock(RCL_STEADY_TIME);
      interests_.find(id)->second.time = steady_clock.now();
    }
  } else {
    interests_.erase(id);
    if (id.find("person") != std::string::npos) {
      parent_node_->remove_activation("perception_people_detection");
    } else {
      parent_node_->remove_activation("perception_objects_detection");
    }
    RCLCPP_DEBUG(parent_node_->get_logger(), "Removed interest: %s, %d", id.c_str(), status);
  }

}

// Create a vector from perceptions_ whose id match
std::vector<perception_system_interfaces::msg::Detection>
PerceptionListener::get_by_id(const std::string & id)
{
  std::vector<perception_system_interfaces::msg::Detection> result;

  if (use_people_filter_ && !use_multiple_sources_filter_) 
  {
    for (auto & perception : humans_) {
      if (perception.first == id) {
        result.push_back(perception.second.msg);
      }
    }
    return result;
  }
  else if (use_people_filter_ && use_multiple_sources_filter_) 
  {
    //todo
  }
  else {
  for (auto & perception : perceptions_) {
    if (perception.first == id) {
      result.push_back(perception.second.msg);
    }
  }
  }

  return result;

}

// Create a vector from perceptions_ whose type match
std::vector<perception_system_interfaces::msg::Detection>
PerceptionListener::get_by_type(const std::string & type)
{
  std::vector<perception_system_interfaces::msg::Detection> result;

  if (use_people_filter_ && !use_multiple_sources_filter_) 
  {
    for (auto & perception : humans_) {
      if (type.empty() || perception.second.msg.type == type) {
        result.push_back(perception.second.msg);
      }
    }
    return result;
  }
  else if (use_people_filter_ && use_multiple_sources_filter_) 
  {
    //todo
  }
  else {
  for (auto & perception : perceptions_) {
    if (type.empty() || perception.second.type == type) {
      result.push_back(perception.second.msg);
    }
  }
  }

  return result;
}

// Create a transform from map to object and send it
int
PerceptionListener::publicTF(
  const perception_system_interfaces::msg::Detection & detected_object,
  const std::string & custom_suffix)
{
  geometry_msgs::msg::TransformStamped map2camera_msg;
  try {
    map2camera_msg = tf_buffer_->lookupTransform(
      tf_frame_map_, tf_frame_camera_,
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO(
      parent_node_->get_logger(), "Could not transform %s to %s: %s",
      tf_frame_map_.c_str(), tf_frame_camera_.c_str(), ex.what());
    return -1;
  }

  tf2::Transform camera2object;
  camera2object.setOrigin(
    tf2::Vector3(
      detected_object.center3d.position.x,
      detected_object.center3d.position.y,
      detected_object.center3d.position.z));
  camera2object.setRotation(tf2::Quaternion(0.0, 0.0, 0.0, 1.0));

  tf2::Transform map2camera;
  tf2::fromMsg(map2camera_msg.transform, map2camera);

  tf2::Transform map2object = map2camera * camera2object;

  // create a transform message from tf2::Transform
  geometry_msgs::msg::TransformStamped map2object_msg;
  map2object_msg.header.stamp = detected_object.header.stamp;
  map2object_msg.header.frame_id = tf_frame_map_;
  map2object_msg.child_frame_id =
    (custom_suffix.empty()) ? detected_object.unique_id : (detected_object.class_name + "_" +
    custom_suffix);
  map2object_msg.transform = tf2::toMsg(map2object);
  
  tf_broadcaster_->sendTransform(map2object_msg);
  return 0;
}

// Public all the transforms in interests_ whose status is true
void PerceptionListener::publicTFinterest()
{
  for (auto & interest : interests_) {
    if (interest.second.status) {
      auto detections = get_by_type(interest.first);
      for (auto & detection : detections) {
        publicTF(detection);
      }
    }
  }
}

void PerceptionListener::publicSortedTFinterest(
  std::function<bool(
    const perception_system_interfaces::msg::Detection &,
    const perception_system_interfaces::msg::Detection &)> comp)
{
  for (auto & interest : interests_) {
    if (interest.second.status) {
      auto detections = get_by_type(interest.first);
      std::sort(detections.begin(), detections.end(), comp);
      int entity_counter = 0;
      for (auto & detection : detections) {
        publicTF(detection, std::to_string(entity_counter));
        entity_counter++;
      }
    }
  }
}

void PerceptionListener::initialize_filter_(
  const geometry_msgs::msg::PointStamped & entity)
{
  if (use_people_filter_ && !use_multiple_sources_filter_)
  {
    Eigen::VectorXd x0(6);
    x0 << entity.point.x, entity.point.y, entity.point.z, 0, 0, 0;
    kf_->initialize(x0);
  }

  else if (use_people_filter_ && use_multiple_sources_filter_)
  {
    /* code */
  }
}

void PerceptionListener::classical_update_(double hz)
{
  // Check if last_update_ is less than hz_callback, and return
  auto now_update = rclcpp::Clock(RCL_STEADY_TIME).now();
  auto diff_update = now_update - last_update_;
  auto seconds = 1.0 / hz;
  if (diff_update.seconds() < seconds) {
    return;
  }

  if (last_msg_ != nullptr) {
    for (auto & detection : last_msg_->detections) {
      // If it is in last_perceptions_ update, if not, create a new element in the vector
      auto det = perceptions_.find(detection.unique_id);
      if (det != perceptions_.end()) {
        det->second.msg = detection;

        det->second.time = rclcpp::Clock(RCL_STEADY_TIME).now();
      } else {
        perceptions_.emplace(
          std::pair<std::string, PerceptionData>(
            detection.unique_id,
            {detection.type, detection, rclcpp::Clock(RCL_STEADY_TIME).now()}));
      }
    }
  }
  
  // Check if last_perceptions_ elements are too old, and remove it
  std::vector<std::string> to_remove_percetions;
  for (auto & perception : perceptions_) {
    auto now = rclcpp::Clock(RCL_STEADY_TIME).now();
    auto diff = now - perception.second.time;
    if (diff.seconds() > max_time_perception_) {
      to_remove_percetions.push_back(perception.first);
    }
  }

  for (auto & id : to_remove_percetions) {
    perceptions_.erase(id);
  }

  // Check if interests are too old, and call set_interest( , false)
  std::vector<std::string> to_remove_interests;
  for (auto & interest : interests_) {
    auto now = rclcpp::Clock(RCL_STEADY_TIME).now();
    auto diff = now - interest.second.time;
    if (diff.seconds() > max_time_interest_) {
      to_remove_interests.push_back(interest.first);
    }
  }

  for (auto & id : to_remove_interests) {
    set_interest(id, false);
  }

  last_update_ = rclcpp::Clock(RCL_STEADY_TIME).now();
}

void PerceptionListener::update_with_yolo_(
  double hz)
{  
  auto now_update = rclcpp::Clock(RCL_STEADY_TIME).now();
  auto diff_update = now_update - last_update_;
  auto seconds = 1.0 / hz;
  if (diff_update.seconds() < seconds) {
    return;
  }

  if (last_msg_ != nullptr) 
  {
    for (const auto & detection : last_msg_->detections) {
      // If it is in last_perceptions_ update, if not, create a new element in the vector
      auto det = humans_.find(detection.unique_id);
      if (det != humans_.end()) {
        det->second.update(detection, rclcpp::Clock(RCL_STEADY_TIME).now());
 
      } 
      else {
        for (auto & human : humans_) 
        {
          const auto& variances = human.second.kf->get_state_variance();

          // Calculate Mahalanobis distance 
          double dx = detection.center3d.position.x - human.second.x;
          double dy = detection.center3d.position.y - human.second.y;
          double dz = detection.center3d.position.z - human.second.z;

          double mahalanobis_dist = std::sqrt(
              (dx * dx / variances[0]) +
              (dy * dy / variances[1]) +
              (dz * dz / variances[2])
          );
          //  scipy.stats.norm.ppf (1.0 - (1.0-self.confidence_percentile)/2., 0, 1.0) 0.9 percentile
          if (mahalanobis_dist < 1.6448536269514722)
          {
            human.second.update(detection, rclcpp::Clock(RCL_STEADY_TIME).now());
          }
          else 
          {
            humans_.emplace(
              std::make_pair(
                detection.unique_id,
                HumanTracked(detection,
                             hz,                      
                             rclcpp::Clock(RCL_STEADY_TIME).now())));
          }          
        }
        humans_.emplace(
          std::make_pair(
            detection.unique_id,
            HumanTracked(detection,
                         hz,
                         rclcpp::Clock(RCL_STEADY_TIME).now())));                      
      }
    }
  }

  // Check if last_perceptions_ elements are too old, and remove it
  std::vector<std::string> to_remove_percetions;
  for (auto & perception : humans_) {
    auto now = rclcpp::Clock(RCL_STEADY_TIME).now();
    auto diff = now - perception.second.time;
    if (diff.seconds() > max_time_perception_) {
      to_remove_percetions.push_back(perception.first);
    }
  }

  for (auto & id : to_remove_percetions) {
    humans_.erase(id);
  }

  // Check if interests are too old, and call set_interest( , false)
  std::vector<std::string> to_remove_interests;
  for (auto & interest : interests_) {
    auto now = rclcpp::Clock(RCL_STEADY_TIME).now();
    auto diff = now - interest.second.time;
    if (diff.seconds() > max_time_interest_) {
      to_remove_interests.push_back(interest.first);
    }
  }

  for (auto & id : to_remove_interests) {
    set_interest(id, false);
  }

  last_update_ = rclcpp::Clock(RCL_STEADY_TIME).now();
}

}  // namespace perception_system
