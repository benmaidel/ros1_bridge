#pragma once


#include <xmlrpcpp/XmlRpcException.h>

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
#include "ros/this_node.h"
#include "ros/header.h"
#include "ros/service_manager.h"
#include "ros/transport/transport_tcp.h"

// include ROS 2
#include "rclcpp/rclcpp.hpp"
#include "rcpputils/scope_exit.hpp"

#include <list>
#include <string>

#include "ros1_bridge/bridge.hpp"

namespace ros1_bridge {

struct Bridge1to2HandlesAndMessageTypes
{
  ros1_bridge::Bridge1to2Handles bridge_handles;
  std::string ros1_type_name;
  std::string ros2_type_name;
};

struct Bridge2to1HandlesAndMessageTypes
{
  ros1_bridge::Bridge2to1Handles bridge_handles;
  std::string ros1_type_name;
  std::string ros2_type_name;
};

rclcpp::QoS qos_from_params(XmlRpc::XmlRpcValue qos_params)
{
  auto ros2_publisher_qos = rclcpp::QoS(rclcpp::KeepLast(10));

  printf("Qos(");

  if (qos_params.getType() == XmlRpc::XmlRpcValue::TypeStruct) {
    if (qos_params.hasMember("history")) {
      auto history = static_cast<std::string>(qos_params["history"]);
      printf("history: ");
      if (history == "keep_all") {
        ros2_publisher_qos.keep_all();
        printf("keep_all, ");
      } else if (history == "keep_last") {
        if (qos_params.hasMember("depth")) {
          auto depth = static_cast<int>(qos_params["depth"]);
          ros2_publisher_qos.keep_last(depth);
          printf("keep_last(%i), ", depth);
        } else {
          fprintf(
            stderr,
            "history: keep_last requires that also a depth is set\n");
        }
      } else {
        fprintf(
          stderr,
          "invalid value for 'history': '%s', allowed values are 'keep_all',"
          "'keep_last' (also requires 'depth' to be set)\n",
          history.c_str());
      }
    }

    if (qos_params.hasMember("reliability")) {
      auto reliability = static_cast<std::string>(qos_params["reliability"]);
      printf("reliability: ");
      if (reliability == "best_effort") {
        ros2_publisher_qos.best_effort();
        printf("best_effort, ");
      } else if (reliability == "reliable") {
        ros2_publisher_qos.reliable();
        printf("reliable, ");
      } else {
        fprintf(
          stderr,
          "invalid value for 'reliability': '%s', allowed values are 'best_effort', 'reliable'\n",
          reliability.c_str());
      }
    }

    if (qos_params.hasMember("durability")) {
      auto durability = static_cast<std::string>(qos_params["durability"]);
      printf("durability: ");
      if (durability == "transient_local") {
        ros2_publisher_qos.transient_local();
        printf("transient_local, ");
      } else if (durability == "volatile") {
        ros2_publisher_qos.durability_volatile();
        printf("volatile, ");
      } else {
        fprintf(
          stderr,
          "invalid value for 'durability': '%s', allowed values are 'best_effort', 'volatile'\n",
          durability.c_str());
      }
    }

    if (qos_params.hasMember("deadline")) {
      try {
        rclcpp::Duration dur = rclcpp::Duration(
          static_cast<int>(qos_params["deadline"]["secs"]),
          static_cast<int>(qos_params["deadline"]["nsecs"]));
        ros2_publisher_qos.deadline(dur);
        printf("deadline: Duration(nsecs: %ld), ", dur.nanoseconds());
      } catch (std::runtime_error & e) {
        fprintf(
          stderr,
          "failed to parse deadline: '%s'\n",
          e.what());
      } catch (XmlRpc::XmlRpcException & e) {
        fprintf(
          stderr,
          "failed to parse deadline: '%s'\n",
          e.getMessage().c_str());
      }
    }

    if (qos_params.hasMember("lifespan")) {
      try {
        rclcpp::Duration dur = rclcpp::Duration(
          static_cast<int>(qos_params["lifespan"]["secs"]),
          static_cast<int>(qos_params["lifespan"]["nsecs"]));
        ros2_publisher_qos.lifespan(dur);
        printf("lifespan: Duration(nsecs: %ld), ", dur.nanoseconds());
      } catch (std::runtime_error & e) {
        fprintf(
          stderr,
          "failed to parse lifespan: '%s'\n",
          e.what());
      } catch (XmlRpc::XmlRpcException & e) {
        fprintf(
          stderr,
          "failed to parse lifespan: '%s'\n",
          e.getMessage().c_str());
      }
    }

    if (qos_params.hasMember("liveliness")) {
      if (qos_params["liveliness"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
        try {
          auto liveliness = static_cast<int>(qos_params["liveliness"]);
          ros2_publisher_qos.liveliness(static_cast<rmw_qos_liveliness_policy_t>(liveliness));
          printf("liveliness: %i, ", static_cast<int>(liveliness));
        } catch (std::runtime_error & e) {
          fprintf(
            stderr,
            "failed to parse liveliness: '%s'\n",
            e.what());
        } catch (XmlRpc::XmlRpcException & e) {
          fprintf(
            stderr,
            "failed to parse liveliness: '%s'\n",
            e.getMessage().c_str());
        }
      } else if (qos_params["liveliness"].getType() == XmlRpc::XmlRpcValue::TypeString) {
        try {
          rmw_qos_liveliness_policy_t liveliness =
            rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
          auto liveliness_str = static_cast<std::string>(qos_params["liveliness"]);
          if (liveliness_str == "LIVELINESS_SYSTEM_DEFAULT" ||
            liveliness_str == "liveliness_system_default")
          {
            liveliness = rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT;
          } else if (liveliness_str == "LIVELINESS_AUTOMATIC" ||  // NOLINT
            liveliness_str == "liveliness_automatic")
          {
            liveliness = rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_AUTOMATIC;
          } else if (liveliness_str == "LIVELINESS_MANUAL_BY_TOPIC" ||  // NOLINT
            liveliness_str == "liveliness_manual_by_topic")
          {
            liveliness = rmw_qos_liveliness_policy_t::RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC;
          } else {
            fprintf(
              stderr,
              "invalid value for 'liveliness': '%s', allowed values are "
              "LIVELINESS_{SYSTEM_DEFAULT, AUTOMATIC, MANUAL_BY_TOPIC}, upper or lower case\n",
              liveliness_str.c_str());
          }

          ros2_publisher_qos.liveliness(liveliness);
          printf("liveliness: %s, ", liveliness_str.c_str());
        } catch (std::runtime_error & e) {
          fprintf(
            stderr,
            "failed to parse liveliness: '%s'\n",
            e.what());
        } catch (XmlRpc::XmlRpcException & e) {
          fprintf(
            stderr,
            "failed to parse liveliness: '%s'\n",
            e.getMessage().c_str());
        }
      } else {
        fprintf(
          stderr,
          "failed to parse liveliness, parameter was not a string or int \n");
      }
    }

    if (qos_params.hasMember("liveliness_lease_duration")) {
      try {
        rclcpp::Duration dur = rclcpp::Duration(
          static_cast<int>(qos_params["liveliness_lease_duration"]["secs"]),
          static_cast<int>(qos_params["liveliness_lease_duration"]["nsecs"]));
        ros2_publisher_qos.liveliness_lease_duration(dur);
        printf("liveliness_lease_duration: Duration(nsecs: %ld), ", dur.nanoseconds());
      } catch (std::runtime_error & e) {
        fprintf(
          stderr,
          "failed to parse liveliness_lease_duration: '%s'\n",
          e.what());
      } catch (XmlRpc::XmlRpcException & e) {
        fprintf(
          stderr,
          "failed to parse liveliness_lease_duration: '%s'\n",
          e.getMessage().c_str());
      }
    }
  } else {
    fprintf(
      stderr,
      "QoS parameters could not be read\n");
  }

  printf(")");
  return ros2_publisher_qos;
}

bool find_command_option(const std::vector<std::string> & args, const std::string & option)
{
  return std::find(args.begin(), args.end(), option) != args.end();
}

bool get_flag_option(const std::vector<std::string> & args, const std::string & option)
{
  auto it = std::find(args.begin(), args.end(), option);
  return it != args.end();
}

bool get_ros1_master_system_state(XmlRpc::XmlRpcValue &payload)
{
  XmlRpc::XmlRpcValue args, result;
  args[0] = ros::this_node::getName();
  if (!ros::master::execute("getSystemState", args, result, payload, true)) {
    fprintf(stderr, "failed to get system state from ROS 1 master\n");
    return false;
  }
  return true;
}

bool get_ros1_active_publishers(const XmlRpc::XmlRpcValue &payload, std::set<std::string> &active_publishers)
{
  // check publishers
  if (payload.size() >= 1) {
    for (int j = 0; j < payload[0].size(); ++j) {
      std::string topic_name = payload[0][j][0];
      for (int k = 0; k < payload[0][j][1].size(); ++k) {
        std::string node_name = payload[0][j][1][k];
        // ignore publishers from the bridge itself
        if (node_name == ros::this_node::getName()) {
          continue;
        }
        active_publishers.insert(topic_name);
        break;
      }
    }
  }
  return true;
}

bool get_ros1_active_subscribers(const XmlRpc::XmlRpcValue &payload, std::set<std::string> &active_subscribers)
{
  // check subscribers
  if (payload.size() >= 2) {
    for (int j = 0; j < payload[1].size(); ++j) {
      std::string topic_name = payload[1][j][0];
      for (int k = 0; k < payload[1][j][1].size(); ++k) {
        std::string node_name = payload[1][j][1][k];
        // ignore subscribers from the bridge itself
        if (node_name == ros::this_node::getName()) {
          continue;
        }
        active_subscribers.insert(topic_name);
        break;
      }
    }
  }
  return true;
}

void get_ros1_service_info(
  const std::string name, std::map<std::string, std::map<std::string, std::string>> & ros1_services)
{
  // NOTE(rkozik):
  // I tried to use Connection class but could not make it work
  // auto callback = [](const ros::ConnectionPtr&, const ros::Header&)
  //                 { printf("Callback\n"); return true; };
  // ros::HeaderReceivedFunc f(callback);
  // ros::ConnectionPtr connection(new ros::Connection);
  // connection->initialize(transport, false, ros::HeaderReceivedFunc());
  ros::ServiceManager manager;
  std::string host;
  std::uint32_t port;
  if (!manager.lookupService(name, host, port)) {
    fprintf(stderr, "Failed to look up %s\n", name.data());
    return;
  }
  ros::TransportTCPPtr transport(new ros::TransportTCP(nullptr, ros::TransportTCP::SYNCHRONOUS));
  auto transport_exit = rcpputils::make_scope_exit(
    [transport]() {
      transport->close();
    });
  if (!transport->connect(host, port)) {
    fprintf(stderr, "Failed to connect to %s (%s:%d)\n", name.data(), host.data(), port);
    return;
  }
  ros::M_string header_out;
  header_out["probe"] = "1";
  header_out["md5sum"] = "*";
  header_out["service"] = name;
  header_out["callerid"] = ros::this_node::getName();
  boost::shared_array<uint8_t> buffer;
  uint32_t len;
  ros::Header::write(header_out, buffer, len);
  std::vector<uint8_t> message(len + 4);
  std::memcpy(&message[0], &len, 4);
  std::memcpy(&message[4], buffer.get(), len);
  transport->write(message.data(), message.size());
  uint32_t length;
  auto read = transport->read(reinterpret_cast<uint8_t *>(&length), 4);
  if (read != 4) {
    fprintf(stderr, "Failed to read a response from a service server\n");
    return;
  }
  std::vector<uint8_t> response(length);
  read = transport->read(response.data(), length);
  if (read < 0 || static_cast<uint32_t>(read) != length) {
    fprintf(stderr, "Failed to read a response from a service server\n");
    return;
  }
  std::string key = name;
  ros1_services[key] = std::map<std::string, std::string>();
  ros::Header header_in;
  std::string error;
  auto success = header_in.parse(response.data(), length, error);
  if (!success) {
    fprintf(stderr, "%s\n", error.data());
    return;
  }
  for (std::string field : {"type"}) {
    std::string value;
    auto success = header_in.getValue(field, value);
    if (!success) {
      fprintf(stderr, "Failed to read '%s' from a header for '%s'\n", field.data(), key.c_str());
      ros1_services.erase(key);
      return;
    }
    ros1_services[key][field] = value;
  }
  std::string t = ros1_services[key]["type"];
  ros1_services[key]["package"] = std::string(t.begin(), t.begin() + t.find("/"));
  ros1_services[key]["name"] = std::string(t.begin() + t.find("/") + 1, t.end());
}

bool get_ros1_current_topics(const std::set<std::string> &active_publishers,
                        const std::set<std::string> &active_subscribers,
                        std::map<std::string, std::string> &current_ros1_publishers,
                        std::map<std::string, std::string> &current_ros1_subscribers,
                        bool output_topic_introspection = false)
{
  // get message types for all topics
  ros::master::V_TopicInfo topics;
  bool success = ros::master::getTopics(topics);
  if (!success) {
    fprintf(stderr, "failed to poll ROS 1 master\n");
    return false;
  }

  for (auto topic : topics) {
    auto topic_name = topic.name;
    bool has_publisher = active_publishers.find(topic_name) != active_publishers.end();
    bool has_subscriber = active_subscribers.find(topic_name) != active_subscribers.end();
    if (!has_publisher && !has_subscriber) {
      // skip inactive topics
      continue;
    }
    if (has_publisher) {
      current_ros1_publishers[topic_name] = topic.datatype;
    }
    if (has_subscriber) {
      current_ros1_subscribers[topic_name] = topic.datatype;
    }
    if (output_topic_introspection) {
      printf(
        "  ROS 1: %s (%s) [%s pubs, %s subs]\n",
        topic_name.c_str(), topic.datatype.c_str(),
        has_publisher ? ">0" : "0", has_subscriber ? ">0" : "0");
    }
  }

  // since ROS 1 subscribers don't report their type they must be added anyway
  for (auto active_subscriber : active_subscribers) {
    if (current_ros1_subscribers.find(active_subscriber) == current_ros1_subscribers.end()) {
      current_ros1_subscribers[active_subscriber] = "";
      if (output_topic_introspection) {
        printf("  ROS 1: %s (<unknown>) sub++\n", active_subscriber.c_str());
      }
    }
  }

  if (output_topic_introspection) {
    printf("\n");
  }
  return true;
}

// check services
bool get_ros1_services(const XmlRpc::XmlRpcValue &payload, std::map<std::string, std::map<std::string, std::string>> &ros1_services)
{
  if (payload.size() >= 3) {
    for (int j = 0; j < payload[2].size(); ++j) {
      if (payload[2][j][0].getType() == XmlRpc::XmlRpcValue::TypeString) {
        std::string name = payload[2][j][0];
        ros1_bridge::get_ros1_service_info(name, ros1_services);
      }
    }
  }
  return true;
}

void get_ros2_current_topics(rclcpp::Node::SharedPtr ros2_node,
                            std::map<std::string, std::string> &current_ros2_publishers,
                            std::map<std::string, std::string> &current_ros2_subscribers,
                            std::map<std::string, Bridge1to2HandlesAndMessageTypes>& bridges_1to2,
                            std::map<std::string, Bridge2to1HandlesAndMessageTypes>& bridges_2to1,
                            std::set<std::string> &already_ignored_topics,
                            bool output_topic_introspection=false)
{
  auto ros2_topics = ros2_node->get_topic_names_and_types();

  std::set<std::string> ignored_topics;
  ignored_topics.insert("parameter_events");

  for (auto topic_and_types : ros2_topics) {
    // ignore some common ROS 2 specific topics
    if (ignored_topics.find(topic_and_types.first) != ignored_topics.end()) {
      continue;
    }

    auto & topic_name = topic_and_types.first;
    auto & topic_type = topic_and_types.second[0];  // explicitly take the first

    // explicitly avoid topics with more than one type
    if (topic_and_types.second.size() > 1) {
      if (already_ignored_topics.count(topic_name) == 0) {
        std::string types = "";
        for (auto type : topic_and_types.second) {
          types += type + ", ";
        }
        fprintf(
          stderr,
          "warning: ignoring topic '%s', which has more than one type: [%s]\n",
          topic_name.c_str(),
          types.substr(0, types.length() - 2).c_str()
        );
        already_ignored_topics.insert(topic_name);
      }
      continue;
    }

    auto publisher_count = ros2_node->count_publishers(topic_name);
    auto subscriber_count = ros2_node->count_subscribers(topic_name);

    // ignore publishers from the bridge itself
    if (bridges_1to2.find(topic_name) != bridges_1to2.end()) {
      if (publisher_count > 0) {
        --publisher_count;
      }
    }
    // ignore subscribers from the bridge itself
    if (bridges_2to1.find(topic_name) != bridges_2to1.end()) {
      if (subscriber_count > 0) {
        --subscriber_count;
      }
    }

    if (publisher_count) {
      current_ros2_publishers[topic_name] = topic_type;
    }

    if (subscriber_count) {
      current_ros2_subscribers[topic_name] = topic_type;
    }

    if (output_topic_introspection) {
      printf(
        "  ROS 2: %s (%s) [%zu pubs, %zu subs]\n",
        topic_name.c_str(), topic_type.c_str(), publisher_count, subscriber_count);
    }
  }
  if (output_topic_introspection) {
    printf("\n");
  }
}

void get_ros2_services(rclcpp::Node::SharedPtr ros2_node,
                        std::map<std::string, std::map<std::string, std::string>> &active_ros2_services,
                        std::set<std::string>& already_ignored_services)
{
  // collect available services (not clients)
  std::set<std::string> service_names;
  std::vector<std::pair<std::string, std::string>> node_names_and_namespaces =
    ros2_node->get_node_graph_interface()->get_node_names_and_namespaces();
  for (auto & pair : node_names_and_namespaces) {
    if (pair.first == ros2_node->get_name() && pair.second == ros2_node->get_namespace()) {
      continue;
    }
    std::map<std::string, std::vector<std::string>> services_and_types =
      ros2_node->get_service_names_and_types_by_node(pair.first, pair.second);
    for (auto & it : services_and_types) {
      service_names.insert(it.first);
    }
  }

  auto ros2_services_and_types = ros2_node->get_service_names_and_types();

  for (const auto & service_and_types : ros2_services_and_types) {
    auto & service_name = service_and_types.first;
    auto & service_type = service_and_types.second[0];  // explicitly take the first

    // explicitly avoid services with more than one type
    if (service_and_types.second.size() > 1) {
      if (already_ignored_services.count(service_name) == 0) {
        std::string types = "";
        for (auto type : service_and_types.second) {
          types += type + ", ";
        }
        fprintf(
          stderr,
          "warning: ignoring service '%s', which has more than one type: [%s]\n",
          service_name.c_str(),
          types.substr(0, types.length() - 2).c_str()
        );
        already_ignored_services.insert(service_name);
      }
      continue;
    }

    // TODO(wjwwood): this should be common functionality in the C++ rosidl package
    size_t separator_position = service_type.find('/');
    if (separator_position == std::string::npos) {
      fprintf(stderr, "invalid service type '%s', skipping...\n", service_type.c_str());
      continue;
    }

    // only bridge if there is a service, not for a client
    if (service_names.find(service_name) != service_names.end()) {
      auto service_type_package_name = service_type.substr(0, separator_position);
      auto service_type_srv_name = service_type.substr(separator_position + 1);
      active_ros2_services[service_name]["package"] = service_type_package_name;
      active_ros2_services[service_name]["name"] = service_type_srv_name;
    }
  }
}
} //namespace ros1_bridge