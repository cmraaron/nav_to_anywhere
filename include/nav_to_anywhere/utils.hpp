// Copyright (c) 2023 Aaron Lipinski

#pragma once
#include <regex>

#include <string>
#include <vector>

#define ACTION_PICK    ("pick")
#define ACTION_DROP    ("drop")
#define ACTION_NAV     ("navigate")
#define ACTION_DEFAULT ("beepboop")

std::vector<std::string> known_actions{ACTION_PICK, ACTION_DROP, ACTION_NAV};

struct ActionDetail
{
  ActionDetail(
    const std::string & name, const std::string & regex, const std::string & type,
    double duration = 0.0)
  : name(name), regex(regex), type(type), duration(duration), _regex(regex) {}
  const std::string name;
  const std::string regex;
  const std::string type;
  const double duration;
  const std::regex _regex;
};

std::vector<ActionDetail> get_default_action_details()
{
  return {
    {"reset", "-reset\\.", ACTION_DROP, 0.1},
    {"undock", "undock\\.", ACTION_DROP, 4.0},
    {"dock", "dock\\.", ACTION_PICK, 4.0},
    {"nav", "-nav\\.", ACTION_NAV},
  };
}

std::vector<ActionDetail> get_action_details(const rclcpp::Node::SharedPtr & node)
{
  const auto action_names = node->declare_parameter<std::vector<std::string>>(
    "bt_actions",
    std::vector<std::string>{});
  if (action_names.empty()) {
    return get_default_action_details();
  }

  std::vector<ActionDetail> bt_actions;

  for (const auto & name : action_names) {
    const auto build_path = [&name](const std::string & key) {
        return "bt_action_details." + name + "." + key;
      };
    const auto type_wanted =
      node->declare_parameter<std::string>(build_path("type"), ACTION_DEFAULT);
    const auto parsed_type = std::find(known_actions.begin(), known_actions.end(), type_wanted);

    if (parsed_type == known_actions.end()) {
      RCLCPP_WARN(
        node->get_logger(), "action type not supported [%s] for action [%s]",
        type_wanted.c_str(), name.c_str());
      continue;
    }

    const auto regex_string = node->declare_parameter<std::string>(build_path("regex"), "");
    try {
      bt_actions.emplace_back(
        name,
        regex_string,
        *parsed_type,
        node->declare_parameter<double>(build_path("duration"), 0.0)
      );
    } catch (std::regex_error & e) {
      RCLCPP_ERROR(
        node->get_logger(), "invalid regex [%s] for action [%s]",
        regex_string.c_str(), name.c_str());
      continue;
    }
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

  return {"default", "", ACTION_DEFAULT};
}
