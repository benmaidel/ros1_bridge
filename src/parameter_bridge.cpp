// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <xmlrpcpp/XmlRpcException.h>

#include <list>
#include <string>

// include ROS 1
#ifdef __clang__
# pragma clang diagnostic push
# pragma clang diagnostic ignored "-Wunused-parameter"
#endif
#include "ros/callback_queue.h"
#include "ros/ros.h"
#ifdef __clang__
# pragma clang diagnostic pop
#endif

// include ROS 2
#include "rclcpp/rclcpp.hpp"

#include "ros1_bridge/bridge.hpp"
#include "ros1_bridge/helper.hpp"


bool parse_command_options(
  int argc, char ** argv, bool &multi_threads)
{
  std::vector<std::string> args(argv, argv + argc);

  if (ros1_bridge::find_command_option(args, "-h") || ros1_bridge::find_command_option(args, "--help")) {
    std::stringstream ss;
    ss << "Usage:" << std::endl;
    ss << " -h, --help: This message." << std::endl;
    ss << " --multi-threads: Bridge with multiple threads for spinner of ROS 1 and ROS 2.";
    ss << std::endl;
    std::cout << ss.str();
    return false;
  }

  multi_threads = ros1_bridge::get_flag_option(args, "--multi-threads");

  return true;
}

int main(int argc, char * argv[])
{
  bool multi_threads;

  if (!parse_command_options(argc, argv, multi_threads))
    return 0;

  // ROS 1 node
  ros::init(argc, argv, "ros_bridge");
  ros::NodeHandle ros1_node;
  std::unique_ptr<ros::CallbackQueue> ros1_callback_queue = nullptr;
  if (multi_threads) {
    ros1_callback_queue = std::make_unique<ros::CallbackQueue>();
    ros1_node.setCallbackQueue(ros1_callback_queue.get());
  }

  // ROS 2 node
  rclcpp::init(argc, argv);
  auto ros2_node = rclcpp::Node::make_shared("ros_bridge");

  std::list<ros1_bridge::BridgeHandles> all_handles;
  std::list<ros1_bridge::ServiceBridge1to2> service_bridges_1_to_2;
  std::list<ros1_bridge::ServiceBridge2to1> service_bridges_2_to_1;

  // bridge all topics listed in a ROS 1 parameter
  // the topics parameter needs to be an array
  // and each item needs to be a dictionary with the following keys;
  // topic: the name of the topic to bridge (e.g. '/topic_name')
  // type: the type of the topic to bridge (e.g. 'pkgname/msg/MsgName')
  // queue_size: the queue size to use (default: 100)
  const char * topics_parameter_name = "topics";
  // the services parameters need to be arrays
  // and each item needs to be a dictionary with the following keys;
  // service: the name of the service to bridge (e.g. '/service_name')
  // type: the type of the service to bridge (e.g. 'pkgname/srv/SrvName')
  const char * services_1_to_2_parameter_name = "services_1_to_2";
  const char * services_2_to_1_parameter_name = "services_2_to_1";
  const char * service_execution_timeout_parameter_name =
    "ros1_bridge/parameter_bridge/service_execution_timeout";
  if (argc > 1) {
    topics_parameter_name = argv[1];
  }
  if (argc > 2) {
    services_1_to_2_parameter_name = argv[2];
  }
  if (argc > 3) {
    services_2_to_1_parameter_name = argv[3];
  }

  // Topics
  XmlRpc::XmlRpcValue topics;
  if (
    ros1_node.getParam(topics_parameter_name, topics) &&
    topics.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (size_t i = 0; i < static_cast<size_t>(topics.size()); ++i) {
      std::string topic_name = static_cast<std::string>(topics[i]["topic"]);
      std::string type_name = static_cast<std::string>(topics[i]["type"]);
      size_t queue_size = static_cast<int>(topics[i]["queue_size"]);
      if (!queue_size) {
        queue_size = 100;
      }
      printf(
        "Trying to create bidirectional bridge for topic '%s' "
        "with ROS 2 type '%s'\n",
        topic_name.c_str(), type_name.c_str());

      try {
        if (topics[i].hasMember("qos")) {
          printf("Setting up QoS for '%s': ", topic_name.c_str());
          auto qos_settings = ros1_bridge::qos_from_params(topics[i]["qos"]);
          printf("\n");
          ros1_bridge::BridgeHandles handles = ros1_bridge::create_bidirectional_bridge(
            ros1_node, ros2_node, "", type_name, topic_name, queue_size, qos_settings);
          all_handles.push_back(handles);
        } else {
          ros1_bridge::BridgeHandles handles = ros1_bridge::create_bidirectional_bridge(
            ros1_node, ros2_node, "", type_name, topic_name, queue_size);
          all_handles.push_back(handles);
        }
      } catch (std::runtime_error & e) {
        fprintf(
          stderr,
          "failed to create bidirectional bridge for topic '%s' "
          "with ROS 2 type '%s': %s\n",
          topic_name.c_str(), type_name.c_str(), e.what());
      }
    }
  } else {
    fprintf(
      stderr,
      "The parameter '%s' either doesn't exist or isn't an array\n", topics_parameter_name);
  }

  // ROS 1 Services in ROS 2
  XmlRpc::XmlRpcValue services_1_to_2;
  if (
    ros1_node.getParam(services_1_to_2_parameter_name, services_1_to_2) &&
    services_1_to_2.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    int service_execution_timeout{5};
    ros1_node.getParamCached(
      service_execution_timeout_parameter_name, service_execution_timeout);
    for (size_t i = 0; i < static_cast<size_t>(services_1_to_2.size()); ++i) {
      std::string service_name = static_cast<std::string>(services_1_to_2[i]["service"]);
      std::string type_name = static_cast<std::string>(services_1_to_2[i]["type"]);
      {
        // for backward compatibility
        std::string package_name = static_cast<std::string>(services_1_to_2[i]["package"]);
        if (!package_name.empty()) {
          fprintf(
            stderr,
            "The service '%s' uses the key 'package' which is deprecated for "
            "services. Instead prepend the 'type' value with '<package>/'.\n",
            service_name.c_str());
          type_name = package_name + "/" + type_name;
        }
      }
      printf(
        "Trying to create bridge for ROS 2 service '%s' with type '%s'\n",
        service_name.c_str(), type_name.c_str());

      const size_t index = type_name.find("/");
      if (index == std::string::npos) {
        fprintf(
          stderr,
          "the service '%s' has a type '%s' without a slash.\n",
          service_name.c_str(), type_name.c_str());
        continue;
      }
      auto factory = ros1_bridge::get_service_factory(
        "ros2", type_name.substr(0, index), type_name.substr(index + 1));
      if (factory) {
        try {
          service_bridges_1_to_2.push_back(
            factory->service_bridge_1_to_2(
              ros1_node, ros2_node, service_name, service_execution_timeout, multi_threads));
          printf("Created 1 to 2 bridge for service %s\n", service_name.c_str());
        } catch (std::runtime_error & e) {
          fprintf(
            stderr,
            "failed to create bridge ROS 1 service '%s' with type '%s': %s\n",
            service_name.c_str(), type_name.c_str(), e.what());
        }
      } else {
        fprintf(
          stderr,
          "failed to create bridge ROS 1 service '%s' no conversion for type '%s'\n",
          service_name.c_str(), type_name.c_str());
      }
    }

  } else {
    fprintf(
      stderr,
      "The parameter '%s' either doesn't exist or isn't an array\n",
      services_1_to_2_parameter_name);
  }

  // ROS 2 Services in ROS 1
  XmlRpc::XmlRpcValue services_2_to_1;
  if (
    ros1_node.getParam(services_2_to_1_parameter_name, services_2_to_1) &&
    services_2_to_1.getType() == XmlRpc::XmlRpcValue::TypeArray)
  {
    for (size_t i = 0; i < static_cast<size_t>(services_2_to_1.size()); ++i) {
      std::string service_name = static_cast<std::string>(services_2_to_1[i]["service"]);
      std::string type_name = static_cast<std::string>(services_2_to_1[i]["type"]);
      {
        // for backward compatibility
        std::string package_name = static_cast<std::string>(services_2_to_1[i]["package"]);
        if (!package_name.empty()) {
          fprintf(
            stderr,
            "The service '%s' uses the key 'package' which is deprecated for "
            "services. Instead prepend the 'type' value with '<package>/'.\n",
            service_name.c_str());
          type_name = package_name + "/" + type_name;
        }
      }
      printf(
        "Trying to create bridge for ROS 1 service '%s' with type '%s'\n",
        service_name.c_str(), type_name.c_str());

      const size_t index = type_name.find("/");
      if (index == std::string::npos) {
        fprintf(
          stderr,
          "the service '%s' has a type '%s' without a slash.\n",
          service_name.c_str(), type_name.c_str());
        continue;
      }

      auto factory = ros1_bridge::get_service_factory(
        "ros1", type_name.substr(0, index), type_name.substr(index + 1));
      if (factory) {
        try {
          service_bridges_2_to_1.push_back(
            factory->service_bridge_2_to_1(ros1_node, ros2_node, service_name, multi_threads));
          printf("Created 2 to 1 bridge for service %s\n", service_name.c_str());
        } catch (std::runtime_error & e) {
          fprintf(
            stderr,
            "failed to create bridge ROS 2 service '%s' with type '%s': %s\n",
            service_name.c_str(), type_name.c_str(), e.what());
        }
      } else {
        fprintf(
          stderr,
          "failed to create bridge ROS 2 service '%s' no conversion for type '%s'\n",
          service_name.c_str(), type_name.c_str());
      }
    }

  } else {
    fprintf(
      stderr,
      "The parameter '%s' either doesn't exist or isn't an array\n",
      services_2_to_1_parameter_name);
  }

  auto check_ros1_flag = [&ros1_node] {
    if (!ros1_node.ok()) {
      rclcpp::shutdown();
    }
  };

  auto ros2_poll_timer = ros2_node->create_wall_timer(
    std::chrono::seconds(1), [&check_ros1_flag] {
      check_ros1_flag();
    }
  );

  // ROS 1 asynchronous spinner
  std::unique_ptr<ros::AsyncSpinner> async_spinner = nullptr;
  if (!multi_threads) {
    async_spinner = std::make_unique<ros::AsyncSpinner>(1);
  } else {
    async_spinner = std::make_unique<ros::AsyncSpinner>(0, ros1_callback_queue.get());
  }
  async_spinner->start();

  // ROS 2 spinning loop
  std::unique_ptr<rclcpp::Executor> executor = nullptr;
  if (!multi_threads) {
    executor = std::make_unique<rclcpp::executors::SingleThreadedExecutor>();
  } else {
    executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
  }
  executor->add_node(ros2_node);
  executor->spin();

  return 0;
}
