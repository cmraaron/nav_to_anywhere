// Copyright (c) 2023 Aaron Lipinski

#include <fstream>
#include <filesystem>
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_to_anywhere/utils.hpp"

namespace fs = std::filesystem;

struct TempFile
{
  FILE * tmpf = std::tmpfile();
  const std::string path{
    fs::read_symlink(fs::path("/proc/self/fd") / std::to_string(fileno(tmpf)))
  };

  TempFile()
  {
    std::ofstream tmpstream(path);
    tmpstream <<
      "/**:\n"
      "  ros__parameters:\n"
      "    bt_actions: ['undock', 'dock', 'reset', 'nav', 'default', 'except']\n"
      "    bt_action_details:\n"
      "      dock:\n"
      "        regex: 'dock.xml$'\n"
      "        type: pick\n"
      "        duration: 4.0\n"
      "      undock:\n"
      "        regex: 'undock.xml'\n"
      "        type: drop\n"
      "        duration: 4.0\n"
      "      reset:\n"
      "        regex: 'reset.xml'\n"
      "        type: reset\n"
      "        duration: 0.1\n"
      "      nav:\n"
      "        regex: 'nav.xml'\n"
      "        type: navigate\n"
      "      except:\n"
      "        regex: '*nav.xml'\n"
      "        type: navigate\n"
      "      default:\n"
      "        regex: ''\n"
      "        type: beepboop\n"
      "        duration: 4.0\n";
  }

  ~TempFile()
  {
    std::remove(path.c_str());
  }
};

TEST(Utils, get_action_details)
{
  rclcpp::init(0, nullptr);
  TempFile tempfile;
  rclcpp::NodeOptions no;
  no.arguments(
  {
    "--ros-args",
    "--params-file", tempfile.path.c_str(),
  });

  const auto node = std::make_shared<rclcpp::Node>("test", no);
  const auto bt_actions = get_action_details(node);

  EXPECT_EQ(get_action(bt_actions, "blahdock.xml").type, "pick");
  EXPECT_EQ(get_action(bt_actions, "blahundock.xml").type, "drop");
  EXPECT_EQ(get_action(bt_actions, "blahreset.xml").type, "reset");
  EXPECT_EQ(get_action(bt_actions, "blahnav.xml").type, "navigate");
  EXPECT_EQ(get_action(bt_actions, "nomatch.xml").type, "beepboop");

  // "except" is not found because it threw an exception when creating
  for (const auto & detail : bt_actions) {
    EXPECT_NE(detail.name, "except");
  }
  rclcpp::shutdown();
}
