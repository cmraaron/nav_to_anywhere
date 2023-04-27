// Copyright (c) 2023 Aaron Lipinski

#pragma once
#include <regex>

#include <string>
#include <vector>

std::vector<std::string> known_actions{"pick", "drop", "reset", "navigate", "beepboop"};
struct ActionDetail
{
  const std::string regex;
  const std::vector<std::string>::iterator type;
  const double duration;
};

std::vector<ActionDetail> get_action_details(const rclcpp::Node::SharedPtr & node)
{
  std::vector<ActionDetail> bt_actions;

  for (const auto & bt_action :
    node->declare_parameter<std::vector<std::string>>("bt_actions", std::vector<std::string>{}))
  {
    const auto build_path = [&](const std::string & key) {
        return "bt_action_details." + bt_action + "." + key;
      };
    const auto a_type = node->declare_parameter<std::string>(build_path("type"), "beepboop");
    const auto parsed_action = std::find(known_actions.begin(), known_actions.end(), a_type);

    if (parsed_action == known_actions.end()) {
      RCLCPP_WARN(node->get_logger(), "action not supported [%s]", a_type.c_str());
      continue;
    }

    const ActionDetail ad{
      node->declare_parameter<std::string>(build_path("regex"), ""),
      parsed_action,
      node->declare_parameter<double>(build_path("duration"), 0.0)
    };
    bt_actions.push_back(ad);
  }
  return bt_actions;
}

std::string get_action(const std::vector<ActionDetail> & bt_actions, const std::string & bt_file)
{
  for (const auto & ad : bt_actions) {
    try {
      std::regex regex(ad.regex);
      if (std::regex_search(bt_file, regex)) {
        return *ad.type;
      }
    } catch (std::regex_error & e) {
      return "_error_";
    }
  }

  return "_unknown_";
}
