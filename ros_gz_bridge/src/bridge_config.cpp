// Copyright 2022 Open Source Robotics Foundation, Inc.
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

#include <yaml-cpp/exceptions.h>
#include <yaml-cpp/yaml.h>

#include <fstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <ros_gz_bridge/bridge_config.hpp>

namespace ros_gz_bridge
{

// YAML tag string constants
constexpr const char kServiceName[] = "service_name";
constexpr const char kTopicName[] = "topic_name";
constexpr const char kRosTopicName[] = "ros_topic_name";
constexpr const char kGzTopicName[] = "gz_topic_name";
constexpr const char kRosTypeName[] = "ros_type_name";
constexpr const char kGzTypeName[] = "gz_type_name";
constexpr const char kDirection[] = "direction";
constexpr const char kPublisherQueue[] = "publisher_queue";
constexpr const char kSubscriberQueue[] = "subscriber_queue";
constexpr const char kQosProfile[] = "qos_profile";
constexpr const char kLazy[] = "lazy";
constexpr const char kGzReqTypeName[] = "gz_req_type_name";
constexpr const char kGzRepTypeName[] = "gz_rep_type_name";

// Comparison strings for bridge directions
constexpr const char kBidirectional[] = "BIDIRECTIONAL";
constexpr const char kGzToRos[] = "GZ_TO_ROS";
constexpr const char kRosToGz[] = "ROS_TO_GZ";

rclcpp::QoS parseQoS(const std::string & qos_profile)
{
  if (qos_profile == "CLOCK") {
    return rclcpp::ClockQoS();
  } else if (qos_profile == "SENSOR_DATA") {
    return rclcpp::SensorDataQoS();
  } else if (qos_profile == "PARAMETERS") {
    return rclcpp::ParametersQoS();
  } else if (qos_profile == "SERVICES") {
    return rclcpp::ServicesQoS();
  } else if (qos_profile == "PARAMETER_EVENTS") {
    return rclcpp::ParameterEventsQoS();
  } else if (qos_profile == "ROSOUT") {
    return rclcpp::RosoutQoS();
  } else if (qos_profile == "SYSTEM_DEFAULT") {
    return rclcpp::SystemDefaultsQoS();
  } else if (qos_profile == "BEST_AVAILABLE") {
    return rclcpp::BestAvailableQoS();
  }
  throw std::invalid_argument(std::string("Invalid QoS profile '") + qos_profile + "'");
}

/// \brief Parse a single sequence entry into a BridgeConfig
/// \param[in] yaml_node A node containing a map of bridge config params
/// \return BridgeConfig on success, nullopt on failure
std::optional<BridgeConfig> parseEntry(const YAML::Node & yaml_node)
{
  auto logger = rclcpp::get_logger("BridgeConfig");

  if (!yaml_node.IsMap()) {
    RCLCPP_ERROR(
      logger,
      "Could not parse entry: entry must be a YAML map");
    return {};
  }

  auto getValue = [yaml_node](const char * key) -> std::string
    {
      if (yaml_node[key]) {
        return yaml_node[key].as<std::string>();
      } else {
        return "";
      }
    };

  const auto service_name = getValue(kServiceName);
  const auto gz_req_type_name = getValue(kGzReqTypeName);
  const auto gz_rep_type_name = getValue(kGzRepTypeName);
  const auto topic_name = getValue(kTopicName);
  const auto ros_topic_name = getValue(kRosTopicName);
  const auto ros_type_name = getValue(kRosTypeName);
  const auto gz_topic_name = getValue(kGzTopicName);
  const auto gz_type_name = getValue(kGzTypeName);
  const auto direction = getValue(kDirection);

  if (!topic_name.empty() && !ros_topic_name.empty()) {
    RCLCPP_ERROR(
      logger,
      "Could not parse entry: %s and %s are mutually exclusive", kTopicName, kRosTopicName);
    return {};
  }

  if (!topic_name.empty() && !gz_topic_name.empty()) {
    RCLCPP_ERROR(
      logger,
      "Could not parse entry: %s and %s are mutually exclusive", kTopicName, kGzTopicName);
    return {};
  }

  if (service_name.empty()) {
    if (ros_type_name.empty() || gz_type_name.empty()) {
      RCLCPP_ERROR(
        logger,
        "Could not parse entry: both %s and %s must be set", kRosTypeName, kGzTypeName);
      return {};
    }
  } else {
    if (gz_req_type_name.empty() || gz_rep_type_name.empty()) {
      RCLCPP_ERROR(
        logger,
        "Could not parse entry: both %s and %s must be set", kGzReqTypeName, kGzRepTypeName);
      return {};
    }
  }

  BridgeConfig ret;

  ret.direction = BridgeDirection::BIDIRECTIONAL;

  if (yaml_node[kDirection]) {
    if (direction == kBidirectional) {
      ret.direction = BridgeDirection::BIDIRECTIONAL;
    } else if (direction == kGzToRos) {
      ret.direction = BridgeDirection::GZ_TO_ROS;
    } else if (direction == kRosToGz) {
      ret.direction = BridgeDirection::ROS_TO_GZ;
    } else {
      RCLCPP_ERROR(
        logger,
        "Could not parse entry: invalid direction [%s]", direction.c_str());
      return {};
    }
  }

  if (service_name.empty()) {
    if (!topic_name.empty()) {
      // Only "topic_name" is set
      ret.gz_topic_name = topic_name;
      ret.ros_topic_name = topic_name;
    } else if (!ros_topic_name.empty() && gz_topic_name.empty()) {
      // Only "ros_topic_name" is set
      ret.gz_topic_name = ros_topic_name;
      ret.ros_topic_name = ros_topic_name;
    } else if (!gz_topic_name.empty() && ros_topic_name.empty()) {
      // Only kGzTopicName is set
      ret.gz_topic_name = gz_topic_name;
      ret.ros_topic_name = gz_topic_name;
    } else {
      // Both are set
      ret.gz_topic_name = gz_topic_name;
      ret.ros_topic_name = ros_topic_name;
    }

    ret.gz_type_name = gz_type_name;
    ret.ros_type_name = ros_type_name;


    if (yaml_node[kQosProfile]) {
      const auto qos_profile_str = getValue(kQosProfile);
      if (!qos_profile_str.empty()) {
        try {
          ret.qos_profile = parseQoS(qos_profile_str);
        } catch (const std::invalid_argument & e) {
          RCLCPP_ERROR(logger, "Could not parse entry: %s", e.what());
          return {};
        }
      }
    }
    if (yaml_node[kPublisherQueue]) {
      const auto queue_size_int = yaml_node[kPublisherQueue].as<int64_t>();
      if (queue_size_int >= 0) {
        ret.publisher_queue_size = static_cast<size_t>(queue_size_int);
      } else if (!ret.qos_profile.has_value()) {
        ret.publisher_queue_size = kDefaultPublisherQueue;
      }
    }
    if (yaml_node[kSubscriberQueue]) {
      const auto queue_size_int = yaml_node[kSubscriberQueue].as<int64_t>();
      if (queue_size_int >= 0) {
        ret.subscriber_queue_size = static_cast<size_t>(queue_size_int);
      } else if (!ret.qos_profile.has_value()) {
        ret.subscriber_queue_size = kDefaultSubscriberQueue;
      }
    }
    if (yaml_node[kLazy]) {
      ret.is_lazy = yaml_node[kLazy].as<bool>();
    }
  } else {
    ret.service_name = service_name;
    ret.gz_rep_type_name = gz_rep_type_name;
    ret.gz_req_type_name = gz_req_type_name;
    ret.ros_type_name = ros_type_name;
  }

  return ret;
}

std::vector<BridgeConfig> readFromYaml(std::istream & in)
{
  auto logger = rclcpp::get_logger("readFromYaml");
  auto ret = std::vector<BridgeConfig>();

  YAML::Node yaml_node;
  yaml_node = YAML::Load(in);

  if (!yaml_node.IsSequence()) {
    RCLCPP_ERROR(
      logger,
      "Could not parse config: top level must be a YAML sequence");
    return ret;
  }

  for (auto it : yaml_node) {
    auto entry = parseEntry(it);
    if (entry) {
      ret.push_back(entry.value());
    }
  }

  return ret;
}

std::vector<BridgeConfig> readFromYamlFile(const std::string & filename)
{
  std::vector<BridgeConfig> ret;
  std::ifstream in(filename);

  auto logger = rclcpp::get_logger("readFromYamlFile");
  if (!in.is_open()) {
    RCLCPP_ERROR(
      logger,
      "Could not parse config: failed to open file [%s]", filename.c_str());
    return ret;
  }

  // Compute file size to warn on empty configuration
  const auto fbegin = in.tellg();
  in.seekg(0, std::ios::end);
  const auto fend = in.tellg();
  const auto fsize = fend - fbegin;

  if (fsize == 0) {
    RCLCPP_ERROR(
      logger,
      "Could not parse config: file empty [%s]", filename.c_str());
    return ret;
  }

  in.seekg(0, std::ios::beg);
  return readFromYaml(in);
}

rclcpp::QoS BridgeConfig::PublisherQoS() const
{
  const auto & queue_size = this->publisher_queue_size;
  if (!this->qos_profile.has_value()) {
    return rclcpp::QoS(rclcpp::KeepLast(queue_size.value_or(kDefaultPublisherQueue)));
  }

  auto qos = *this->qos_profile;
  if (queue_size.has_value() && *queue_size > 0) {
    qos.keep_last(*queue_size);
  }
  return qos;
}

rclcpp::QoS BridgeConfig::SubscriberQoS() const
{
  const auto & queue_size = this->subscriber_queue_size;
  if (!this->qos_profile.has_value()) {
    return rclcpp::QoS(rclcpp::KeepLast(queue_size.value_or(kDefaultSubscriberQueue)));
  }

  auto qos = *this->qos_profile;
  if (queue_size.has_value() && *queue_size > 0) {
    qos.keep_last(*queue_size);
  }
  return qos;
}

std::vector<BridgeConfig> readFromYamlString(const std::string & data)
{
  std::stringstream ss(data);
  return readFromYaml(ss);
}
}  // namespace ros_gz_bridge
