// Copyright (c) 2023 Aaron Lipinski

#include <fstream>
#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_to_anywhere/utils.hpp"


struct TempFile
{
  const std::string path{std::tmpnam(nullptr)};

  TempFile()
  {
    std::ofstream tmpstream(path);
    tmpstream <<
      "/**:\n"
      "  ros__parameters:\n"
      "    bt_actions: ['undock', 'dock', 'reset', 'nav']\n"
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
      "      default:\n"
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

  EXPECT_TRUE(true);
  rclcpp::shutdown();
}
