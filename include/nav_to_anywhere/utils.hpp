// Copyright (c) 2023 Aaron Lipinski

#pragma once
#include <regex>

#include <string>
#include <vector>

std::vector<std::string> known_actions{"pick", "drop", "reset", "navigate", "beepboop"};
struct ActionDetail
{
  const std::string name;
  const std::string regex;
  const std::string type;
  const double duration;
  std::regex _regex;
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
      RCLCPP_WARN(
        node->get_logger(), "action type not supported [%s] for action [%s]",
        a_type.c_str(), bt_action.c_str());
      continue;
    }

    const auto regex_string = node->declare_parameter<std::string>(build_path("regex"), "");
    try {
      std::regex regex(regex_string);
    } catch (std::regex_error & e) {
      RCLCPP_ERROR(
        node->get_logger(), "invalid regex [%s] for action [%s]",
        regex_string.c_str(), bt_action.c_str());
      continue;
    }

    const ActionDetail ad{
      bt_action,
      regex_string,
      *parsed_action,
      node->declare_parameter<double>(build_path("duration"), 0.0),
      std::regex(regex_string),
    };
    bt_actions.push_back(ad);
  }
  return bt_actions;
}

ActionDetail get_action(const std::vector<ActionDetail> & bt_actions, const std::string & bt_file)
{
  for (const auto & ad : bt_actions) {
    if (std::regex_search(bt_file, ad._regex)) {
      return ad;
    }
  }

  return {};
}
