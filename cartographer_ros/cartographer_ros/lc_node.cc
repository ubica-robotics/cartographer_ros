/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "absl/memory/memory.h"
#include "cartographer/mapping/map_builder.h"
#include "cartographer_ros/lc_node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"
#include <chrono>
#include <string>
#include <vector>
#include "Eigen/Core"
#include "cartographer/common/configuration_file_resolver.h"
#include "cartographer/common/lua_parameter_dictionary.h"
#include "cartographer/common/port.h"
#include "cartographer/common/time.h"
#include "cartographer/mapping/pose_graph_interface.h"
#include "cartographer/mapping/proto/submap_visualization.pb.h"
#include "cartographer/metrics/register.h"
#include "cartographer/sensor/point_cloud.h"
#include "cartographer/transform/rigid_transform.h"
#include "cartographer/transform/transform.h"
#include "cartographer_ros/metrics/family_factory.h"
#include "cartographer_ros/msg_conversion.h"
#include "cartographer_ros/sensor_bridge.h"
#include "cartographer_ros/tf_bridge.h"
#include "cartographer_ros/time_conversion.h"
#include "cartographer_ros_msgs/msg/status_code.hpp"
#include "cartographer_ros_msgs/msg/status_response.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "glog/logging.h"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_eigen/tf2_eigen.h"
#include "visualization_msgs/msg/marker_array.hpp"

// Log messages
#define DEBUG (std::cout<<"\033[36;1m")
#define END "\033[0m" << std::endl

namespace cartographer_ros_lifecycle {

namespace carto = ::cartographer;

using carto::transform::Rigid3d;
using TrajectoryState =
    ::cartographer::mapping::PoseGraphInterface::TrajectoryState;

namespace {
// Subscribes to the 'topic' for 'trajectory_id' using the 'node_handle' and
// calls 'handler' on the 'node' to handle messages. Returns the subscriber.
template <typename MessageType> ::rclcpp::SubscriptionBase::SharedPtr SubscribeWithHandler(
    void (Node::*handler)(int, const std::string&,
                          const typename MessageType::ConstSharedPtr&),
  const int trajectory_id, const std::string& topic,
   Node* const node) {
  return node->create_subscription<MessageType>(
      topic, rclcpp::SensorDataQoS(),
      [node, handler, trajectory_id, topic](const typename MessageType::ConstSharedPtr msg) {
            (node->*handler)(trajectory_id, topic, msg);
          });
}

std::string TrajectoryStateToString(const TrajectoryState trajectory_state) {
  switch (trajectory_state) {
    case TrajectoryState::ACTIVE:
      return "ACTIVE";
    case TrajectoryState::FINISHED:
      return "FINISHED";
    case TrajectoryState::FROZEN:
      return "FROZEN";
    case TrajectoryState::DELETED:
      return "DELETED";
  }
  return "";
}

}  // namespace

Node::Node() : nav2_util::LifecycleNode("cartographer_lc_node","",false){

  declare_parameter<std::string>("configuration_directory",cartographer_ros_lifecycle::configuration_directory);
  declare_parameter<std::string>("configuration_basename",cartographer_ros_lifecycle::configuration_basename);
  declare_parameter<bool>("collect_metrics",cartographer_ros_lifecycle::collect_metrics);
  declare_parameter<bool>("start_trajectory_with_default_topics", cartographer_ros_lifecycle::start_trajectory_with_default_topics);

}

Node::~Node() {
  submap_list_timer_.reset();
  local_trajectory_data_timer_.reset();
  trajectory_node_list_timer_.reset();
  landmark_pose_list_timer_.reset();
  constrain_list_timer_.reset();
  maybe_warn_about_topic_mismatch_timer_.reset();

  map_builder_bridge_.reset();
  tf_buffer.reset();
  tf_listener.reset();
  tf_broadcaster_.reset();

  metrics_registry_.reset();

  run_final_optimization_server.reset();
  load_state_server.reset();
  load_options_server.reset();
  finish_all_trajectories_server.reset();
  start_trajectory_with_default_topics_server.reset();

  submap_query_server_.reset();
  trajectory_query_server.reset();
  start_trajectory_server_.reset();
  finish_trajectory_server_.reset();
  write_state_server_.reset();
  get_trajectory_states_server_.reset();
  read_metrics_server_.reset();

  submap_list_publisher_.reset();
  trajectory_node_list_publisher_.reset();
  landmark_poses_list_publisher_.reset();
  constraint_list_publisher_.reset();
  tracked_pose_publisher_.reset();
  scan_matched_point_cloud_publisher_.reset();
}

nav2_util::CallbackReturn Node::on_configure(const rclcpp_lifecycle::State & /*state*/){

  get_parameter("configuration_directory",configuration_directory);
  get_parameter("configuration_basename",configuration_basename);
  get_parameter_or("collect_metrics",collect_metrics,false);
  get_parameter_or("start_trajectory_with_default_topics",start_trajectory_with_default_topics,true);

  DEBUG<<configuration_directory<<END;
  DEBUG<<configuration_basename<<END;

  CHECK(!configuration_directory.empty())
      << "-configuration_directory is missing.";
  CHECK(!configuration_basename.empty())
      << "-configuration_basename is missing.";

  submap_list_publisher_ =
      create_publisher<::cartographer_ros_msgs::msg::SubmapList>(
        cartographer_ros::kSubmapListTopic, 10);
  trajectory_node_list_publisher_ =
      create_publisher<::visualization_msgs::msg::MarkerArray>(
        cartographer_ros::kTrajectoryNodeListTopic, 10);
  landmark_poses_list_publisher_ =
      create_publisher<::visualization_msgs::msg::MarkerArray>(
        cartographer_ros::kLandmarkPosesListTopic, 10);
  constraint_list_publisher_ =
      create_publisher<::visualization_msgs::msg::MarkerArray>(
        cartographer_ros::kConstraintListTopic, 10);
  if (node_options_.publish_tracked_pose) {
    tracked_pose_publisher_ =
        create_publisher<::geometry_msgs::msg::PoseStamped>(
          cartographer_ros::kTrackedPoseTopic, 10);
  }

  scan_matched_point_cloud_publisher_ =
      create_publisher<sensor_msgs::msg::PointCloud2>(
        cartographer_ros::kScanMatchedPointCloudTopic, 10);

  // NOTE
  // Now that we are editing the src of carto, I'm planning to create some custom services convenient for us
  // With the objective of simplifying the map and remap routines
  run_final_optimization_server = create_service<std_srvs::srv::Trigger>(
        cartographer_ros::kRunFinalOptimizationServiceName,
        std::bind(
          &Node::handleRunFinalOptimization, this, std::placeholders::_1, std::placeholders::_2));
  load_state_server = create_service<cartographer_ros_msgs::srv::WriteState>(
        cartographer_ros::kLoadStateServiceName,
        std::bind(
          &Node::handleLoadState, this, std::placeholders::_1, std::placeholders::_2));
  load_options_server = create_service<cartographer_ros_msgs::srv::LoadOptions>(
        cartographer_ros::kLoadOptionsServiceName,
        std::bind(
          &Node::handleLoadOptions, this, std::placeholders::_1, std::placeholders::_2));
  start_trajectory_with_default_topics_server = create_service<std_srvs::srv::Trigger>(
        cartographer_ros::kStartTrajectoryWithDefaultTopicsServiceName,
        std::bind(&Node::handleStartTrajectoryWithDefaultTopics,this,std::placeholders::_1, std::placeholders::_2));
  finish_all_trajectories_server = create_service<std_srvs::srv::Trigger>(
        cartographer_ros::kFinishAllTrajectoriesServiceName,
        std::bind(&Node::handleFinishAllTrajectories,this,std::placeholders::_1, std::placeholders::_2));

  submap_query_server_ = create_service<cartographer_ros_msgs::srv::SubmapQuery>(
        cartographer_ros::kSubmapQueryServiceName,
        std::bind(
          &Node::handleSubmapQuery, this, std::placeholders::_1, std::placeholders::_2));
  trajectory_query_server = create_service<cartographer_ros_msgs::srv::TrajectoryQuery>(
        cartographer_ros::kTrajectoryQueryServiceName,
        std::bind(
          &Node::handleTrajectoryQuery, this, std::placeholders::_1, std::placeholders::_2));
  start_trajectory_server_ = create_service<cartographer_ros_msgs::srv::StartTrajectory>(
        cartographer_ros::kStartTrajectoryServiceName,
        std::bind(
          &Node::handleStartTrajectory, this, std::placeholders::_1, std::placeholders::_2));
  finish_trajectory_server_ = create_service<cartographer_ros_msgs::srv::FinishTrajectory>(
        cartographer_ros::kFinishTrajectoryServiceName,
        std::bind(
          &Node::handleFinishTrajectory, this, std::placeholders::_1, std::placeholders::_2));
  write_state_server_ = create_service<cartographer_ros_msgs::srv::WriteState>(
        cartographer_ros::kWriteStateServiceName,
        std::bind(
          &Node::handleWriteState, this, std::placeholders::_1, std::placeholders::_2));
  get_trajectory_states_server_ = create_service<cartographer_ros_msgs::srv::GetTrajectoryStates>(
        cartographer_ros::kGetTrajectoryStatesServiceName,
        std::bind(
          &Node::handleGetTrajectoryStates, this, std::placeholders::_1, std::placeholders::_2));
  read_metrics_server_ = create_service<cartographer_ros_msgs::srv::ReadMetrics>(
        cartographer_ros::kReadMetricsServiceName,
        std::bind(
          &Node::handleReadMetrics, this, std::placeholders::_1, std::placeholders::_2));

  // Always registering metrics!
  if (collect_metrics){
    metrics_registry_ = absl::make_unique<cartographer_ros::metrics::FamilyFactory>();
    carto::metrics::RegisterAllMetrics(metrics_registry_.get());
  }

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn Node::on_activate(const rclcpp_lifecycle::State & /*state*/){
  DEBUG<<"1"<<END;
  std::tie(node_options_, trajectory_options_) =
      cartographer_ros::LoadOptions(configuration_directory, configuration_basename);
  DEBUG<<"2"<<END;
  auto map_builder =
    cartographer::mapping::CreateMapBuilder(node_options_.map_builder_options);
  DEBUG<<"3"<<END;
  constexpr double kTfBufferCacheTimeInSeconds = 10.;
  tf_buffer = std::make_shared<tf2_ros::Buffer>(get_clock(),tf2::durationFromSec(kTfBufferCacheTimeInSeconds));
  tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);
  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
  DEBUG<<"4"<<END;
  absl::MutexLock lock(&mutex_);
  map_builder_bridge_.reset(new cartographer_ros::MapBuilderBridge(node_options_, std::move(map_builder), tf_buffer.get()));
DEBUG<<"5"<<END;
  submap_list_publisher_->on_activate();
  trajectory_node_list_publisher_->on_activate();
  landmark_poses_list_publisher_->on_activate();
  constraint_list_publisher_->on_activate();
  if (node_options_.publish_tracked_pose) {
    tracked_pose_publisher_->on_activate();}
  scan_matched_point_cloud_publisher_->on_activate();
DEBUG<<"6"<<END;
  submap_list_timer_ = create_wall_timer(
    std::chrono::milliseconds(int(node_options_.submap_publish_period_sec * 1000)),
    [this]() {
      PublishSubmapList();
    });
  if (node_options_.pose_publish_period_sec > 0) {
    local_trajectory_data_timer_ = create_wall_timer(
      std::chrono::milliseconds(int(node_options_.pose_publish_period_sec * 1000)),
      [this]() {
        PublishLocalTrajectoryData();
      });
  }
  trajectory_node_list_timer_ = create_wall_timer(
    std::chrono::milliseconds(int(node_options_.trajectory_publish_period_sec * 1000)),
    [this]() {
      PublishTrajectoryNodeList();
    });
  landmark_pose_list_timer_ = create_wall_timer(
    std::chrono::milliseconds(int(node_options_.trajectory_publish_period_sec * 1000)),
    [this]() {
      PublishLandmarkPosesList();
    });
  constrain_list_timer_ = create_wall_timer(
    std::chrono::milliseconds(int(cartographer_ros::kConstraintPublishPeriodSec * 1000)),
    [this]() {
      PublishConstraintList();
    });
DEBUG<<"7"<<END;
  //if (start_trajectory_with_default_topics){
    //DEBUG<<"7.1"<<END;
    //StartTrajectoryWithDefaultTopics(trajectory_options_);
  //}
DEBUG<<"8"<<END;
  createBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn Node::on_deactivate(const rclcpp_lifecycle::State & /*state*/){

  submap_list_timer_->cancel();
  local_trajectory_data_timer_->cancel();
  trajectory_node_list_timer_->cancel();
  landmark_pose_list_timer_->cancel();
  constrain_list_timer_->cancel();
  maybe_warn_about_topic_mismatch_timer_->cancel();

  submap_list_publisher_->on_deactivate();
  trajectory_node_list_publisher_->on_deactivate();
  landmark_poses_list_publisher_->on_deactivate();
  constraint_list_publisher_->on_deactivate();
  if (node_options_.publish_tracked_pose) {
    tracked_pose_publisher_->on_deactivate();}
  scan_matched_point_cloud_publisher_->on_deactivate();

  tf_buffer->clear();

  extrapolators_.clear();
  last_published_tf_stamps_.clear();
  sensor_samplers_.clear();
  subscribers_.clear();
  subscribed_topics_.clear();
  trajectories_scheduled_for_finish_.clear();

  node_options_=cartographer_ros::NodeOptions();
  trajectory_options_=cartographer_ros::TrajectoryOptions();

  destroyBond();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn Node::on_cleanup(const rclcpp_lifecycle::State & /*state*/){

  submap_list_timer_.reset();
  local_trajectory_data_timer_.reset();
  trajectory_node_list_timer_.reset();
  landmark_pose_list_timer_.reset();
  constrain_list_timer_.reset();
  maybe_warn_about_topic_mismatch_timer_.reset();

  absl::MutexLock lock(&mutex_);
  map_builder_bridge_.reset();
  tf_buffer.reset();
  tf_listener.reset();
  tf_broadcaster_.reset();

  metrics_registry_.reset();

  run_final_optimization_server.reset();
  load_state_server.reset();
  load_options_server.reset();
  start_trajectory_with_default_topics_server.reset();
  finish_all_trajectories_server.reset();


  submap_query_server_.reset();
  trajectory_query_server.reset();
  start_trajectory_server_.reset();
  finish_trajectory_server_.reset();
  write_state_server_.reset();
  get_trajectory_states_server_.reset();
  read_metrics_server_.reset();

  submap_list_publisher_.reset();
  trajectory_node_list_publisher_.reset();
  landmark_poses_list_publisher_.reset();
  constraint_list_publisher_.reset();
  tracked_pose_publisher_.reset();
  scan_matched_point_cloud_publisher_.reset();

  return nav2_util::CallbackReturn::SUCCESS;
}

nav2_util::CallbackReturn Node::on_shutdown(const rclcpp_lifecycle::State & /*state*/){

  return nav2_util::CallbackReturn::SUCCESS;
}

// NOTE
// Now that we are editing the src of carto, I'm planning to create some custom services convenient for us
// With the objective of simplifying the map and remap routines
bool Node::handleRunFinalOptimization(const std::shared_ptr<std_srvs::srv::Trigger::Request> , std::shared_ptr<std_srvs::srv::Trigger::Response> response){

  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_WARN(
        get_logger(),
        "Received RunFinalOptimization request but not in ACTIVE state, ignoring!");
      return false;
    }

  // This blocks
  RunFinalOptimization();

  response->success=true;
  return true;
}

bool Node::handleLoadState(const cartographer_ros_msgs::srv::WriteState::Request::SharedPtr request,
                           cartographer_ros_msgs::srv::WriteState::Response::SharedPtr response){

  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_WARN(
        get_logger(),
        "Received LoadState request but not in ACTIVE state, ignoring!");
      return false;
    }

  // filename should be actually: FLAGS_load_state_filename
  // include_unfinished_submaps should be actually: FLAGS_load_frozen_state
  LoadState(request.get()->filename, request.get()->include_unfinished_submaps);

  response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
  return true;
}

bool Node::handleLoadOptions(const cartographer_ros_msgs::srv::LoadOptions::Request::SharedPtr request, cartographer_ros_msgs::srv::LoadOptions::Response::SharedPtr response){

  // Only srv that can be call when the node isn't active

  configuration_directory=request.get()->configuration_directory;
  configuration_basename=request.get()->configuration_basename;
  collect_metrics = request.get()->collect_metrics;
  start_trajectory_with_default_topics = request.get()->start_trajectory_with_default_topics;

  RCLCPP_WARN(get_logger(),"A transition to INACTIVE and then back to ACTIVE state is needed apply the new options loaded");

  response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
  return true;
}

bool Node::handleStartTrajectoryWithDefaultTopics(const std::shared_ptr<std_srvs::srv::Trigger::Request> , std::shared_ptr<std_srvs::srv::Trigger::Response> response){

  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_WARN(
        get_logger(),
        "Received StartTrajectoryWithDefaultTopics request but not in ACTIVE state, ignoring!");
      return false;
    }

  StartTrajectoryWithDefaultTopics(trajectory_options_);

  response->success=true;
  return true;
}

bool Node::handleFinishAllTrajectories(const std::shared_ptr<std_srvs::srv::Trigger::Request> , std::shared_ptr<std_srvs::srv::Trigger::Response> response){

  if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
      RCLCPP_WARN(
        get_logger(),
        "Received FinishAllTrajectories request but not in ACTIVE state, ignoring!");
      return false;
    }

  FinishAllTrajectories();

  response->success=true;
  return true;
}

bool Node::handleSubmapQuery(
    const cartographer_ros_msgs::srv::SubmapQuery::Request::SharedPtr request,
    cartographer_ros_msgs::srv::SubmapQuery::Response::SharedPtr response) {
  absl::MutexLock lock(&mutex_);
  map_builder_bridge_->HandleSubmapQuery(request, response);
  return true;
}

bool Node::handleTrajectoryQuery(
    const cartographer_ros_msgs::srv::TrajectoryQuery::Request::SharedPtr request,
    cartographer_ros_msgs::srv::TrajectoryQuery::Response::SharedPtr response) {
  absl::MutexLock lock(&mutex_);
  response->status = TrajectoryStateToStatus(
      request->trajectory_id,
      {TrajectoryState::ACTIVE, TrajectoryState::FINISHED,
       TrajectoryState::FROZEN} /* valid states */);
  if (response->status.code != cartographer_ros_msgs::msg::StatusCode::OK) {
    LOG(ERROR) << "Can't query trajectory from pose graph: "
               << response->status.message;
    return true;
  }
  map_builder_bridge_->HandleTrajectoryQuery(request, response);
  return true;
}

void Node::PublishSubmapList() {
  absl::MutexLock lock(&mutex_);
  submap_list_publisher_->publish(map_builder_bridge_->GetSubmapList(this->now()));
}

void Node::AddExtrapolator(const int trajectory_id,
                           const cartographer_ros::TrajectoryOptions& options) {
  constexpr double kExtrapolationEstimationTimeSec = 0.001;  // 1 ms
  CHECK(extrapolators_.count(trajectory_id) == 0);
  const double gravity_time_constant =
      node_options_.map_builder_options.use_trajectory_builder_3d()
          ? options.trajectory_builder_options.trajectory_builder_3d_options()
                .imu_gravity_time_constant()
          : options.trajectory_builder_options.trajectory_builder_2d_options()
                .imu_gravity_time_constant();
  extrapolators_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          ::cartographer::common::FromSeconds(kExtrapolationEstimationTimeSec),
          gravity_time_constant));
}

void Node::AddSensorSamplers(const int trajectory_id,
                             const cartographer_ros::TrajectoryOptions& options) {
  CHECK(sensor_samplers_.count(trajectory_id) == 0);
  sensor_samplers_.emplace(
      std::piecewise_construct, std::forward_as_tuple(trajectory_id),
      std::forward_as_tuple(
          options.rangefinder_sampling_ratio, options.odometry_sampling_ratio,
          options.fixed_frame_pose_sampling_ratio, options.imu_sampling_ratio,
          options.landmarks_sampling_ratio));
}

void Node::PublishLocalTrajectoryData() {
  absl::MutexLock lock(&mutex_);
  for (const auto& entry : map_builder_bridge_->GetLocalTrajectoryData()) {
    const auto& trajectory_data = entry.second;

    auto& extrapolator = extrapolators_.at(entry.first);
    // We only publish a point cloud if it has changed. It is not needed at high
    // frequency, and republishing it would be computationally wasteful.
    if (trajectory_data.local_slam_data->time !=
        extrapolator.GetLastPoseTime()) {
      if (scan_matched_point_cloud_publisher_->get_subscription_count() > 0) {
        // TODO(gaschler): Consider using other message without time
        // information.
        carto::sensor::TimedPointCloud point_cloud;
        point_cloud.reserve(trajectory_data.local_slam_data->range_data_in_local
                                .returns.size());
        for (const cartographer::sensor::RangefinderPoint point :
             trajectory_data.local_slam_data->range_data_in_local.returns) {
          point_cloud.push_back(cartographer::sensor::ToTimedRangefinderPoint(
              point, 0.f /* time */));
        }
        scan_matched_point_cloud_publisher_->publish(cartographer_ros::ToPointCloud2Message(
            carto::common::ToUniversal(trajectory_data.local_slam_data->time),
            node_options_.map_frame,
            carto::sensor::TransformTimedPointCloud(
                point_cloud, trajectory_data.local_to_map.cast<float>())));
      }
      extrapolator.AddPose(trajectory_data.local_slam_data->time,
                           trajectory_data.local_slam_data->local_pose);
    }

    geometry_msgs::msg::TransformStamped stamped_transform;
    // If we do not publish a new point cloud, we still allow time of the
    // published poses to advance. If we already know a newer pose, we use its
    // time instead. Since tf knows how to interpolate, providing newer
    // information is better.
    const ::cartographer::common::Time now = std::max(
        cartographer_ros::FromRos(this->now()), extrapolator.GetLastExtrapolatedTime());
    stamped_transform.header.stamp =
        node_options_.use_pose_extrapolator
            ? cartographer_ros::ToRos(now)
            : cartographer_ros::ToRos(trajectory_data.local_slam_data->time);

    // Suppress publishing if we already published a transform at this time.
    // Due to 2020-07 changes to geometry2, tf buffer will issue warnings for
    // repeated transforms with the same timestamp.
    if (last_published_tf_stamps_.count(entry.first) &&
        last_published_tf_stamps_[entry.first] == stamped_transform.header.stamp)
      continue;
    last_published_tf_stamps_[entry.first] = stamped_transform.header.stamp;

    const Rigid3d tracking_to_local_3d =
        node_options_.use_pose_extrapolator
            ? extrapolator.ExtrapolatePose(now)
            : trajectory_data.local_slam_data->local_pose;
    const Rigid3d tracking_to_local = [&] {
      if (trajectory_data.trajectory_options.publish_frame_projected_to_2d) {
        return carto::transform::Embed3D(
            carto::transform::Project2D(tracking_to_local_3d));
      }
      return tracking_to_local_3d;
    }();

    const Rigid3d tracking_to_map =
        trajectory_data.local_to_map * tracking_to_local;

    if (trajectory_data.published_to_tracking != nullptr) {
      if (node_options_.publish_to_tf) {
        if (trajectory_data.trajectory_options.provide_odom_frame) {
          std::vector<geometry_msgs::msg::TransformStamped> stamped_transforms;

          stamped_transform.header.frame_id = node_options_.map_frame;
          stamped_transform.child_frame_id =
              trajectory_data.trajectory_options.odom_frame;
          stamped_transform.transform =
              cartographer_ros::ToGeometryMsgTransform(trajectory_data.local_to_map);
          stamped_transforms.push_back(stamped_transform);

          stamped_transform.header.frame_id =
              trajectory_data.trajectory_options.odom_frame;
          stamped_transform.child_frame_id =
              trajectory_data.trajectory_options.published_frame;
          stamped_transform.transform = cartographer_ros::ToGeometryMsgTransform(
              tracking_to_local * (*trajectory_data.published_to_tracking));
          stamped_transforms.push_back(stamped_transform);

          tf_broadcaster_->sendTransform(stamped_transforms);
        } else {
          stamped_transform.header.frame_id = node_options_.map_frame;
          stamped_transform.child_frame_id =
              trajectory_data.trajectory_options.published_frame;
          stamped_transform.transform = cartographer_ros::ToGeometryMsgTransform(
              tracking_to_map * (*trajectory_data.published_to_tracking));
          tf_broadcaster_->sendTransform(stamped_transform);
        }
      }
      if (node_options_.publish_tracked_pose) {
        ::geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.frame_id = node_options_.map_frame;
        pose_msg.header.stamp = stamped_transform.header.stamp;
        pose_msg.pose = cartographer_ros::ToGeometryMsgPose(tracking_to_map);
        tracked_pose_publisher_->publish(pose_msg);
      }
    }
  }
}

void Node::PublishTrajectoryNodeList() {
  if (trajectory_node_list_publisher_->get_subscription_count() > 0) {
    absl::MutexLock lock(&mutex_);
    trajectory_node_list_publisher_->publish(
        map_builder_bridge_->GetTrajectoryNodeList(this->now()));
  }
}

void Node::PublishLandmarkPosesList() {
  if (landmark_poses_list_publisher_->get_subscription_count() > 0) {
    absl::MutexLock lock(&mutex_);
    landmark_poses_list_publisher_->publish(
        map_builder_bridge_->GetLandmarkPosesList(this->now()));
  }
}

void Node::PublishConstraintList() {
  if (constraint_list_publisher_->get_subscription_count() > 0) {
    absl::MutexLock lock(&mutex_);
    constraint_list_publisher_->publish(map_builder_bridge_->GetConstraintList(this->now()));
  }
}

std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
Node::ComputeExpectedSensorIds(const cartographer_ros::TrajectoryOptions& options) const {
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  using SensorType = SensorId::SensorType;
  std::set<SensorId> expected_topics;
  // Subscribe to all laser scan, multi echo laser scan, and point cloud topics.
  for (const std::string& topic :
       cartographer_ros::ComputeRepeatedTopicNames(cartographer_ros::kLaserScanTopic, options.num_laser_scans)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  for (const std::string& topic : cartographer_ros::ComputeRepeatedTopicNames(
           cartographer_ros::kMultiEchoLaserScanTopic, options.num_multi_echo_laser_scans)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  for (const std::string& topic :
       cartographer_ros::ComputeRepeatedTopicNames(cartographer_ros::kPointCloud2Topic, options.num_point_clouds)) {
    expected_topics.insert(SensorId{SensorType::RANGE, topic});
  }
  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    expected_topics.insert(SensorId{SensorType::IMU, cartographer_ros::kImuTopic});
  }
  // Odometry is optional.
  if (options.use_odometry) {
    expected_topics.insert(SensorId{SensorType::ODOMETRY, cartographer_ros::kOdometryTopic});
  }
  // NavSatFix is optional.
  if (options.use_nav_sat) {
    expected_topics.insert(
        SensorId{SensorType::FIXED_FRAME_POSE, cartographer_ros::kNavSatFixTopic});
  }
  // Landmark is optional.
  if (options.use_landmarks) {
    expected_topics.insert(SensorId{SensorType::LANDMARK, cartographer_ros::kLandmarkTopic});
  }
  return expected_topics;
}

int Node::AddTrajectory(const cartographer_ros::TrajectoryOptions& options) {
  const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>
      expected_sensor_ids = ComputeExpectedSensorIds(options);
  const int trajectory_id =
      map_builder_bridge_->AddTrajectory(expected_sensor_ids, options);
  DEBUG<<"100"<<END;
  AddExtrapolator(trajectory_id, options);
  AddSensorSamplers(trajectory_id, options);
  LaunchSubscribers(options, trajectory_id);
  maybe_warn_about_topic_mismatch_timer_ = create_wall_timer(
    std::chrono::milliseconds(int(cartographer_ros::kTopicMismatchCheckDelaySec * 1000)),
    [this]() {
      MaybeWarnAboutTopicMismatch();
    });
  for (const auto& sensor_id : expected_sensor_ids) {
    subscribed_topics_.insert(sensor_id.id);
  }
  return trajectory_id;
}

void Node::LaunchSubscribers(const cartographer_ros::TrajectoryOptions& options,
                             const int trajectory_id) {
  for (const std::string& topic :
       cartographer_ros::ComputeRepeatedTopicNames(cartographer_ros::kLaserScanTopic, options.num_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::LaserScan>(
             &Node::HandleLaserScanMessage, trajectory_id, topic, this),
         topic});
  }
  for (const std::string& topic : cartographer_ros::ComputeRepeatedTopicNames(
           cartographer_ros::kMultiEchoLaserScanTopic, options.num_multi_echo_laser_scans)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::MultiEchoLaserScan>(
             &Node::HandleMultiEchoLaserScanMessage, trajectory_id, topic, this),
         topic});
  }
  for (const std::string& topic :
       cartographer_ros::ComputeRepeatedTopicNames(cartographer_ros::kPointCloud2Topic, options.num_point_clouds)) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::PointCloud2>(
             &Node::HandlePointCloud2Message, trajectory_id, topic, this),
         topic});
  }

  // For 2D SLAM, subscribe to the IMU if we expect it. For 3D SLAM, the IMU is
  // required.
  if (node_options_.map_builder_options.use_trajectory_builder_3d() ||
      (node_options_.map_builder_options.use_trajectory_builder_2d() &&
       options.trajectory_builder_options.trajectory_builder_2d_options()
           .use_imu_data())) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::Imu>(&Node::HandleImuMessage,
                                                trajectory_id, cartographer_ros::kImuTopic,
                                                this),
         cartographer_ros::kImuTopic});
  }

  if (options.use_odometry) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<nav_msgs::msg::Odometry>(&Node::HandleOdometryMessage,
                                                  trajectory_id, cartographer_ros::kOdometryTopic,
                                                  this),
         cartographer_ros::kOdometryTopic});
  }
  if (options.use_nav_sat) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<sensor_msgs::msg::NavSatFix>(
             &Node::HandleNavSatFixMessage, trajectory_id, cartographer_ros::kNavSatFixTopic,
             this),
         cartographer_ros::kNavSatFixTopic});
  }
  if (options.use_landmarks) {
    subscribers_[trajectory_id].push_back(
        {SubscribeWithHandler<cartographer_ros_msgs::msg::LandmarkList>(
             &Node::HandleLandmarkMessage, trajectory_id, cartographer_ros::kLandmarkTopic,
             this),
         cartographer_ros::kLandmarkTopic});
  }
}

bool Node::ValidateTrajectoryOptions(const cartographer_ros::TrajectoryOptions& options) {
  if (node_options_.map_builder_options.use_trajectory_builder_2d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_2d_options();
  }
  if (node_options_.map_builder_options.use_trajectory_builder_3d()) {
    return options.trajectory_builder_options
        .has_trajectory_builder_3d_options();
  }
  return false;
}

bool Node::ValidateTopicNames(const cartographer_ros::TrajectoryOptions& options) {
  for (const auto& sensor_id : ComputeExpectedSensorIds(options)) {
    const std::string& topic = sensor_id.id;
    if (subscribed_topics_.count(topic) > 0) {
      LOG(ERROR) << "Topic name [" << topic << "] is already used.";
      return false;
    }
  }
  return true;
}

cartographer_ros_msgs::msg::StatusResponse Node::TrajectoryStateToStatus(
    const int trajectory_id, const std::set<TrajectoryState>& valid_states) {
  const auto trajectory_states = map_builder_bridge_->GetTrajectoryStates();
  cartographer_ros_msgs::msg::StatusResponse status_response;

  const auto it = trajectory_states.find(trajectory_id);
  if (it == trajectory_states.end()) {
    status_response.message = "Trajectory " + std::to_string(trajectory_id) + " doesn't exist.";
    status_response.code = cartographer_ros_msgs::msg::StatusCode::NOT_FOUND;
    return status_response;
  }

  status_response.message = "Trajectory " + std::to_string(trajectory_id) + " is in '" +
    TrajectoryStateToString(it->second) + "' state.";
  status_response.code =
      valid_states.count(it->second)
          ? cartographer_ros_msgs::msg::StatusCode::OK
          : cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
  return status_response;
}

cartographer_ros_msgs::msg::StatusResponse Node::FinishTrajectoryUnderLock(
    const int trajectory_id) {
  cartographer_ros_msgs::msg::StatusResponse status_response;
  if (trajectories_scheduled_for_finish_.count(trajectory_id)) {
    status_response.message =
        "Trajectory " + std::to_string(trajectory_id) + " already pending to finish.";
    status_response.code = cartographer_ros_msgs::msg::StatusCode::OK;
    LOG(INFO) << status_response.message;
    return status_response;
  }

  // First, check if we can actually finish the trajectory.
  status_response = TrajectoryStateToStatus(
      trajectory_id, {TrajectoryState::ACTIVE} /* valid states */);
  if (status_response.code != cartographer_ros_msgs::msg::StatusCode::OK) {
    LOG(ERROR) << "Can't finish trajectory: " << status_response.message;
    return status_response;
  }

  // Shutdown the subscribers of this trajectory.
  // A valid case with no subscribers is e.g. if we just visualize states.
  if (subscribers_.count(trajectory_id)) {
    for (auto& entry : subscribers_[trajectory_id]) {
      entry.subscriber.reset();
      subscribed_topics_.erase(entry.topic);
      LOG(INFO) << "Shutdown the subscriber of [" << entry.topic << "]";
    }
    CHECK_EQ(subscribers_.erase(trajectory_id), 1);
  }
  map_builder_bridge_->FinishTrajectory(trajectory_id);
  trajectories_scheduled_for_finish_.emplace(trajectory_id);
  status_response.message =
      "Finished trajectory " + std::to_string(trajectory_id) + ".";
  status_response.code = cartographer_ros_msgs::msg::StatusCode::OK;
  return status_response;
}

bool Node::handleStartTrajectory(
    const cartographer_ros_msgs::srv::StartTrajectory::Request::SharedPtr request,
    cartographer_ros_msgs::srv::StartTrajectory::Response::SharedPtr response) {

  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE){
    response->status.code=cartographer_ros_msgs::msg::StatusCode::UNAVAILABLE;
    response->status.message="Node isn't in Active state and start trajectory requires it to be";
    return true;
  }

  cartographer_ros::TrajectoryOptions trajectory_options;
  std::tie(std::ignore, trajectory_options) = cartographer_ros::LoadOptions(
      request->configuration_directory, request->configuration_basename);

  if (request->use_initial_pose) {
    const auto pose = cartographer_ros::ToRigid3d(request->initial_pose);
    if (!pose.IsValid()) {
      response->status.message =
          "Invalid pose argument. Orientation quaternion must be normalized.";
      LOG(ERROR) << response->status.message;
      response->status.code =
          cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
      return true;
    }

    // Check if the requested trajectory for the relative initial pose exists.
    response->status = TrajectoryStateToStatus(
        request->relative_to_trajectory_id,
        {TrajectoryState::ACTIVE, TrajectoryState::FROZEN,
         TrajectoryState::FINISHED} /* valid states */);
    if (response->status.code != cartographer_ros_msgs::msg::StatusCode::OK) {
      LOG(ERROR) << "Can't start a trajectory with initial pose: "
                 << response->status.message;
      return true;
    }

    ::cartographer::mapping::proto::InitialTrajectoryPose
        initial_trajectory_pose;
    initial_trajectory_pose.set_to_trajectory_id(
        request->relative_to_trajectory_id);
    *initial_trajectory_pose.mutable_relative_pose() =
        cartographer::transform::ToProto(pose);
    initial_trajectory_pose.set_timestamp(cartographer::common::ToUniversal(
        ::cartographer_ros::FromRos(rclcpp::Time(0))));
    *trajectory_options.trajectory_builder_options
         .mutable_initial_trajectory_pose() = initial_trajectory_pose;
  }

  if (!ValidateTrajectoryOptions(trajectory_options)) {
    response->status.message = "Invalid trajectory options.";
    LOG(ERROR) << response->status.message;
    response->status.code = cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
  } else if (!ValidateTopicNames(trajectory_options)) {
    response->status.message = "Topics are already used by another trajectory.";
    LOG(ERROR) << response->status.message;
    response->status.code = cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
  } else {
    response->status.message = "Success.";
    response->trajectory_id = AddTrajectory(trajectory_options);
    response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
  }
  return true;
}

void Node::StartTrajectoryWithDefaultTopics(const cartographer_ros::TrajectoryOptions& options) {
  DEBUG<<"7.2"<<END;
  absl::MutexLock lock(&mutex_);
  DEBUG<<"7.3"<<END;
  CHECK(ValidateTrajectoryOptions(options));
  DEBUG<<"7.4"<<END;
  AddTrajectory(options);
}

std::vector<
    std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>>
Node::ComputeDefaultSensorIdsForMultipleBags(
    const std::vector<cartographer_ros::TrajectoryOptions>& bags_options) const {
  using SensorId = cartographer::mapping::TrajectoryBuilderInterface::SensorId;
  std::vector<std::set<SensorId>> bags_sensor_ids;
  for (size_t i = 0; i < bags_options.size(); ++i) {
    std::string prefix;
    if (bags_options.size() > 1) {
      prefix = "bag_" + std::to_string(i + 1) + "_";
    }
    std::set<SensorId> unique_sensor_ids;
    for (const auto& sensor_id : ComputeExpectedSensorIds(bags_options.at(i))) {
      unique_sensor_ids.insert(SensorId{sensor_id.type, prefix + sensor_id.id});
    }
    bags_sensor_ids.push_back(unique_sensor_ids);
  }
  return bags_sensor_ids;
}

int Node::AddOfflineTrajectory(
    const std::set<cartographer::mapping::TrajectoryBuilderInterface::SensorId>&
        expected_sensor_ids,
    const cartographer_ros::TrajectoryOptions& options) {
  absl::MutexLock lock(&mutex_);
  const int trajectory_id =
      map_builder_bridge_->AddTrajectory(expected_sensor_ids, options);
  AddExtrapolator(trajectory_id, options);
  AddSensorSamplers(trajectory_id, options);
  return trajectory_id;
}

bool Node::handleGetTrajectoryStates(
    const cartographer_ros_msgs::srv::GetTrajectoryStates::Request::SharedPtr request,
    cartographer_ros_msgs::srv::GetTrajectoryStates::Response::SharedPtr response) {
  using TrajectoryState =
      ::cartographer::mapping::PoseGraphInterface::TrajectoryState;
  absl::MutexLock lock(&mutex_);
  response->status.code = ::cartographer_ros_msgs::msg::StatusCode::OK;
  response->trajectory_states.header.stamp = this->now();
  for (const auto& entry : map_builder_bridge_->GetTrajectoryStates()) {
    response->trajectory_states.trajectory_id.push_back(entry.first);
    switch (entry.second) {
      case TrajectoryState::ACTIVE:
        response->trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::msg::TrajectoryStates::ACTIVE);
        break;
      case TrajectoryState::FINISHED:
        response->trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::msg::TrajectoryStates::FINISHED);
        break;
      case TrajectoryState::FROZEN:
        response->trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::msg::TrajectoryStates::FROZEN);
        break;
      case TrajectoryState::DELETED:
        response->trajectory_states.trajectory_state.push_back(
            ::cartographer_ros_msgs::msg::TrajectoryStates::DELETED);
        break;
    }
  }
  return true;
}

bool Node::handleFinishTrajectory(
    const cartographer_ros_msgs::srv::FinishTrajectory::Request::SharedPtr request,
    cartographer_ros_msgs::srv::FinishTrajectory::Response::SharedPtr response) {
  absl::MutexLock lock(&mutex_);
  response->status = FinishTrajectoryUnderLock(request->trajectory_id);
  return true;
}

bool Node::handleWriteState(
    const cartographer_ros_msgs::srv::WriteState::Request::SharedPtr request,
    cartographer_ros_msgs::srv::WriteState::Response::SharedPtr response) {
  absl::MutexLock lock(&mutex_);
  if (map_builder_bridge_->SerializeState(request->filename,
                                         request->include_unfinished_submaps)) {
    response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
    response->status.message =
        "State written to '" + request->filename + "'.";
  } else {
    response->status.code = cartographer_ros_msgs::msg::StatusCode::INVALID_ARGUMENT;
    response->status.message =
        "Failed to write '" + request->filename + "'.";
  }
  return true;
}

bool Node::handleReadMetrics(
    const cartographer_ros_msgs::srv::ReadMetrics::Request::SharedPtr request,
    cartographer_ros_msgs::srv::ReadMetrics::Response::SharedPtr response) {
  absl::MutexLock lock(&mutex_);
  response->timestamp = this->now();
  if (!metrics_registry_) {
    response->status.code = cartographer_ros_msgs::msg::StatusCode::UNAVAILABLE;
    response->status.message = "Collection of runtime metrics is not activated.";
    return true;
  }
  metrics_registry_->ReadMetrics(response);
  response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
  response->status.message = "Successfully read metrics.";
  return true;
}

void Node::FinishAllTrajectories() {
  absl::MutexLock lock(&mutex_);
  for (const auto& entry : map_builder_bridge_->GetTrajectoryStates()) {
    if (entry.second == TrajectoryState::ACTIVE) {
      const int trajectory_id = entry.first;
      CHECK_EQ(FinishTrajectoryUnderLock(trajectory_id).code,
               cartographer_ros_msgs::msg::StatusCode::OK);
    }
  }
}

bool Node::FinishTrajectory(const int trajectory_id) {
  absl::MutexLock lock(&mutex_);
  return FinishTrajectoryUnderLock(trajectory_id).code ==
         cartographer_ros_msgs::msg::StatusCode::OK;
}

void Node::RunFinalOptimization() {
  {
    for (const auto& entry : map_builder_bridge_->GetTrajectoryStates()) {
      const int trajectory_id = entry.first;
      if (entry.second == TrajectoryState::ACTIVE) {
        LOG(WARNING)
            << "Can't run final optimization if there are one or more active "
               "trajectories. Trying to finish trajectory with ID "
            << std::to_string(trajectory_id) << " now.";
        CHECK(FinishTrajectory(trajectory_id))
            << "Failed to finish trajectory with ID "
            << std::to_string(trajectory_id) << ".";
      }
    }
  }
  // Assuming we are not adding new data anymore, the final optimization
  // can be performed without holding the mutex.
  map_builder_bridge_->RunFinalOptimization();
}

void Node::HandleOdometryMessage(const int trajectory_id,
                                 const std::string& sensor_id,
                                 const nav_msgs::msg::Odometry::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).odometry_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_->sensor_bridge(trajectory_id);
  auto odometry_data_ptr = sensor_bridge_ptr->ToOdometryData(msg);
  if (odometry_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddOdometryData(*odometry_data_ptr);
  }
  sensor_bridge_ptr->HandleOdometryMessage(sensor_id, msg);
}

void Node::HandleNavSatFixMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::msg::NavSatFix::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).fixed_frame_pose_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandleNavSatFixMessage(sensor_id, msg);
}

void Node::HandleLandmarkMessage(
    const int trajectory_id, const std::string& sensor_id,
    const cartographer_ros_msgs::msg::LandmarkList::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).landmark_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandleLandmarkMessage(sensor_id, msg);
}

void Node::HandleImuMessage(const int trajectory_id,
                            const std::string& sensor_id,
                            const sensor_msgs::msg::Imu::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).imu_sampler.Pulse()) {
    return;
  }
  auto sensor_bridge_ptr = map_builder_bridge_->sensor_bridge(trajectory_id);
  auto imu_data_ptr = sensor_bridge_ptr->ToImuData(msg);
  if (imu_data_ptr != nullptr) {
    extrapolators_.at(trajectory_id).AddImuData(*imu_data_ptr);
  }
  sensor_bridge_ptr->HandleImuMessage(sensor_id, msg);
}

void Node::HandleLaserScanMessage(const int trajectory_id,
                                  const std::string& sensor_id,
                                  const sensor_msgs::msg::LaserScan::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandleLaserScanMessage(sensor_id, msg);
}

void Node::HandleMultiEchoLaserScanMessage(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::msg::MultiEchoLaserScan::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandleMultiEchoLaserScanMessage(sensor_id, msg);
}

void Node::HandlePointCloud2Message(
    const int trajectory_id, const std::string& sensor_id,
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr& msg) {
  absl::MutexLock lock(&mutex_);
  if (!sensor_samplers_.at(trajectory_id).rangefinder_sampler.Pulse()) {
    return;
  }
  map_builder_bridge_->sensor_bridge(trajectory_id)
      ->HandlePointCloud2Message(sensor_id, msg);
}

void Node::SerializeState(const std::string& filename,
                          const bool include_unfinished_submaps) {
  absl::MutexLock lock(&mutex_);
  CHECK(
      map_builder_bridge_->SerializeState(filename, include_unfinished_submaps))
      << "Could not write state.";
}

void Node::LoadState(const std::string& state_filename,
                     const bool load_frozen_state) {
  absl::MutexLock lock(&mutex_);
  map_builder_bridge_->LoadState(state_filename, load_frozen_state);
}

// TODO: find ROS equivalent to ros::master::getTopics
// TODO: this->get_topic_names_and_types() and this->get_node_base_interface()->resolve_topic_or_service_name()
void Node::MaybeWarnAboutTopicMismatch() {
//  ::ros::master::V_TopicInfo ros_topics;
//  ::ros::master::getTopics(ros_topics);
//  std::set<std::string> published_topics;
//  std::stringstream published_topics_string;
//  for (const auto& it : ros_topics) {
//    std::string resolved_topic = node_handle_.resolveName(it.name, false);
//    published_topics.insert(resolved_topic);
//    published_topics_string << resolved_topic << ",";
//  }
//  bool print_topics = false;
//  for (const auto& entry : subscribers_) {
//    int trajectory_id = entry.first;
//    for (const auto& subscriber : entry.second) {
//      std::string resolved_topic = node_handle_.resolveName(subscriber.topic);
//      if (published_topics.count(resolved_topic) == 0) {
//        LOG(WARNING) << "Expected topic \"" << subscriber.topic
//                     << "\" (trajectory " << trajectory_id << ")"
//                     << " (resolved topic \"" << resolved_topic << "\")"
//                     << " but no publisher is currently active.";
//        print_topics = true;
//      }
//    }
//  }
//  if (print_topics) {
//    LOG(WARNING) << "Currently available topics are: "
//                 << published_topics_string.str();
//  }
}
} // cartographer_ros_lifecycle namespace

int main(int argc, char** argv) {

  // Init rclcpp first because gflags reorders command line flags in argv
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]);
//  google::ParseCommandLineFlags(&argc, &argv, false);

  cartographer_ros::ScopedRosLogSink ros_log_sink;

  auto node = std::make_shared<cartographer_ros_lifecycle::Node>();

  ::rclcpp::spin(node->get_node_base_interface());
  ::rclcpp::shutdown();

  return 0;
}
