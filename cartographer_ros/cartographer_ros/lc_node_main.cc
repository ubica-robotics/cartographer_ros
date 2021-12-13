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
#include "cartographer_ros/node.h"
#include "cartographer_ros/node_options.h"
#include "cartographer_ros/ros_log_sink.h"
#include "gflags/gflags.h"
#include "tf2_ros/transform_listener.h"

#include "cartographer_ros_msgs/srv/load_options.hpp"

#include "nav2_util/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "lifecycle_msgs/msg/state.hpp"

#include "cartographer_ros_msgs/msg/status_code.hpp"
#include "cartographer_ros_msgs/msg/status_response.hpp"

// Log messages
#define DEBUG (std::cout<<"\033[36;1m")
#define END "\033[0m" << std::endl


class lc_cartographer_ros: public nav2_util::LifecycleNode{

public:
  lc_cartographer_ros() : nav2_util::LifecycleNode("cartographer_lc_node_main","",false){

    DEBUG<<"Constructor"<<END;

    declare_parameter<std::string>("configuration_directory",configuration_directory);
    declare_parameter<std::string>("configuration_basename",configuration_basename);
    declare_parameter<std::string>("load_state_filename",load_state_filename);
    declare_parameter<std::string>("save_state_filename",save_state_filename);
    declare_parameter<bool>("collect_metrics",collect_metrics);
    declare_parameter<bool>("load_frozen_state",load_frozen_state);
    declare_parameter<bool>("start_trajectory_with_default_topics",start_trajectory_with_default_topics);

    get_parameter_or("use_sim_time",use_sim_time,false);
    cartographer_node = rclcpp::Node::make_shared("cartographer_node");
    rclcpp::Parameter simTime( "use_sim_time", rclcpp::ParameterValue( use_sim_time ) );
    cartographer_node->set_parameter( simTime );
  }

protected:
  nav2_util::CallbackReturn on_configure (const rclcpp_lifecycle::State & /*state*/){

    DEBUG<<"On config"<<END;

    get_parameter("configuration_directory",configuration_directory);
    get_parameter("configuration_basename",configuration_basename);
    get_parameter("load_state_filename",load_state_filename);
    get_parameter("save_state_filename",save_state_filename);
    get_parameter_or("collect_metrics",collect_metrics,false);
    get_parameter_or("load_frozen_state",load_frozen_state,false);
    get_parameter_or("start_trajectory_with_default_topics",start_trajectory_with_default_topics,true);

    CHECK(!configuration_directory.empty())
        << "-configuration_directory is missing.";
    CHECK(!configuration_basename.empty())
        << "-configuration_basename is missing.";

    load_options_server = create_service<cartographer_ros_msgs::srv::LoadOptions>(
          cartographer_ros::kLoadOptionsServiceName,
          std::bind(
            &lc_cartographer_ros::handleLoadOptions, this, std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default);

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State & /*state*/){

    DEBUG<<"On activate"<<END;

    constexpr double kTfBufferCacheTimeInSeconds = 10.;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer =
        std::make_shared<tf2_ros::Buffer>(
          cartographer_node->get_clock(),
          tf2::durationFromSec(kTfBufferCacheTimeInSeconds),
          cartographer_node);

    std::shared_ptr<tf2_ros::TransformListener> tf_listener =
        std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    cartographer_ros::NodeOptions node_options;
    cartographer_ros::TrajectoryOptions trajectory_options;
    std::tie(node_options, trajectory_options) =
        cartographer_ros::LoadOptions(configuration_directory, configuration_basename);

    auto map_builder =
      cartographer::mapping::CreateMapBuilder(node_options.map_builder_options);

    node = std::make_shared<cartographer_ros::Node>(
      node_options, std::move(map_builder), tf_buffer, cartographer_node,
      collect_metrics);

    if (!load_state_filename.empty()) {
      node->LoadState(load_state_filename, load_frozen_state);
    }

    if (start_trajectory_with_default_topics) {
      node->StartTrajectoryWithDefaultTopics(trajectory_options);
    }

    createBond();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*state*/){

    DEBUG<<"On deactivate"<<END;

    node->FinishAllTrajectories();
    node->RunFinalOptimization();

    if (!save_state_filename.empty()) {
      node->SerializeState(save_state_filename,
                          true /* include_unfinished_submaps */);}

    node.reset();

    destroyBond();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*state*/){

    DEBUG<<"On cleanup"<<END;

    configuration_directory.clear();
    configuration_basename.clear();
    load_state_filename.clear();
    save_state_filename.clear();

    load_options_server.reset();

    return nav2_util::CallbackReturn::SUCCESS;
  }

  nav2_util::CallbackReturn on_shutdown (const rclcpp_lifecycle::State & /*state*/){

    return nav2_util::CallbackReturn::SUCCESS;
  }

private:
  bool handleLoadOptions(const cartographer_ros_msgs::srv::LoadOptions::Request::SharedPtr request, cartographer_ros_msgs::srv::LoadOptions::Response::SharedPtr response){

    DEBUG<<"LoadOptions"<<END;
    if (get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
        RCLCPP_WARN(
          get_logger(),
          "Received LoadOptions request but in ACTIVE state, ignoring!");
        return false;
      }

    configuration_directory = request.get()->configuration_directory;
    configuration_basename = request.get()->configuration_basename;
    load_state_filename = request.get()->load_state_filename;
    save_state_filename = request.get()->save_state_filename;
    collect_metrics = request.get()->collect_metrics;
    load_frozen_state = request.get()->load_frozen_state;
    start_trajectory_with_default_topics = request.get()->start_trajectory_with_default_topics;

    // We must go through on_deactivate to clean the system and then on_activate to create a new instance with the options loaded
    RCLCPP_WARN(get_logger(),"A transition to ACTIVE state is needed to apply the new options loaded");

    response->status.code = cartographer_ros_msgs::msg::StatusCode::OK;
    return true;
  }

public:

  rclcpp::Node::SharedPtr cartographer_node;


private:


  std::shared_ptr<cartographer_ros::Node> node;
  ::rclcpp::Service<cartographer_ros_msgs::srv::LoadOptions>::SharedPtr load_options_server;

  std::string configuration_directory;
  std::string configuration_basename;
  std::string load_state_filename;
  std::string save_state_filename;
  bool use_sim_time;
  bool collect_metrics;
  bool load_frozen_state;
  bool start_trajectory_with_default_topics;
};

int main(int argc, char** argv) {

  // Init rclcpp first because gflags reorders command line flags in argv
  rclcpp::init(argc, argv);

  google::AllowCommandLineReparsing();
  google::InitGoogleLogging(argv[0]);
  google::ParseCommandLineFlags(&argc, &argv, false);

  cartographer_ros::ScopedRosLogSink ros_log_sink;
  auto lc_node = std::make_shared<lc_cartographer_ros>();

  rclcpp::executors::SingleThreadedExecutor exec;
  exec.add_node(lc_node->get_node_base_interface());
  exec.add_node(lc_node->cartographer_node);
  exec.spin();

  ::rclcpp::shutdown();

  return 0;
}
