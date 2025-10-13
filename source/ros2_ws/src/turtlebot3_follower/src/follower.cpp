// Copyright 2025 ROBOTIS CO., LTD.
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
//
// Author: Hyungyu Kim

#include "turtlebot3_follower/follower.hpp"


Follower::Follower(const std::string & follower_name, const std::string & leader_name)
: Node(follower_name + "_follower_node"),
  tf_buffer_(this->get_clock()),
  tf_listener_(tf_buffer_),
  leader_name_(leader_name),
  follower_name_(follower_name)
{
  get_parameter("use_sim_time", use_sim_time_);

  tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
  nav2_action_client_ = rclcpp_action::create_client<nav2_msgs::action::FollowPath>(
    this,
    follower_name_ + "/follow_path");

  tf_publisher();
  send_path_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(100),
    std::bind(&Follower::send_path, this));

  RCLCPP_INFO(
    this->get_logger(),
    "[%s] initialized successfully",
    this->get_name());
}

void Follower::tf_publisher()
{
  geometry_msgs::msg::TransformStamped tf_msg;
  tf_msg.header.stamp = this->get_clock()->now();
  tf_msg.header.frame_id = this->leader_name_ + "/odom";
  tf_msg.child_frame_id = this->follower_name_ + "/odom";

  if (this->use_sim_time_ == true) {
    tf_msg.transform.translation.x = 0;
    tf_msg.transform.translation.y = 0;
    tf_msg.transform.translation.z = 0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = 0.0;
    tf_msg.transform.rotation.w = 1.0;
  } else {
    tf_msg.transform.translation.x = -0.14;
    tf_msg.transform.translation.y = 0;
    tf_msg.transform.translation.z = 0;
    tf_msg.transform.rotation.x = 0.0;
    tf_msg.transform.rotation.y = 0.0;
    tf_msg.transform.rotation.z = 0.0;
    tf_msg.transform.rotation.w = 1.0;
  }
  RCLCPP_INFO(this->get_logger(), "tf publish");
  this->tf_broadcaster_->sendTransform(tf_msg);
}

void Follower::send_path()
{
  geometry_msgs::msg::TransformStamped leader_transform;
  try {
    // Get leader's pose in the follower's odom frame, which is the fixed frame for navigation
    leader_transform = this->tf_buffer_.lookupTransform(
      this->follower_name_ + "/odom",
      this->leader_name_ + "/base_link",
      tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Could not transform %s to %s: %s",
      (this->leader_name_ + "/base_link").c_str(),
      (this->follower_name_ + "/odom").c_str(),
      ex.what());
    return;
  }

  // Parameters for controlling the formation
  const double LATERAL_OFFSET = 0.5;  // meters, to the right
  const double LOOK_AHEAD_DIST = 0.3; // meters, in front of the leader

  // Extract leader's position and orientation
  double leader_x = leader_transform.transform.translation.x;
  double leader_y = leader_transform.transform.translation.y;
  tf2::Quaternion q(
    leader_transform.transform.rotation.x,
    leader_transform.transform.rotation.y,
    leader_transform.transform.rotation.z,
    leader_transform.transform.rotation.w);
  tf2::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);

  // Define the target point in the leader's reference frame (forward=+x, left=+y)
  double local_target_x = LOOK_AHEAD_DIST;
  double local_target_y = -LATERAL_OFFSET; // Right is negative Y

  // Transform the local target point to the odom frame using the leader's pose
  double target_x = leader_x + (local_target_x * cos(yaw) - local_target_y * sin(yaw));
  double target_y = leader_y + (local_target_x * sin(yaw) + local_target_y * cos(yaw));

  // Create a single goal pose
  geometry_msgs::msg::PoseStamped goal_pose;
  goal_pose.header.stamp = this->get_clock()->now();
  goal_pose.header.frame_id = this->follower_name_ + "/odom";
  goal_pose.pose.position.x = target_x;
  goal_pose.pose.position.y = target_y;
  goal_pose.pose.orientation = leader_transform.transform.rotation; // Match leader's orientation

  // Create a path with a single pose
  nav_msgs::msg::Path path;
  path.header = goal_pose.header;
  path.poses.push_back(goal_pose);

  auto goal_msg = nav2_msgs::action::FollowPath::Goal();
  goal_msg.path = path;

  if (!this->nav2_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_WARN(this->get_logger(), "Action server not available");
    return;
  }
  this->nav2_action_client_->async_send_goal(goal_msg);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  int number = 0;
  if (argc > 1) {
    try {
      number = std::stoi(argv[1]);
    } catch (const std::exception & e) {
      std::cerr << "Invalid number of followers: " << argv[1] << std::endl;
      return 1;
    }
  }

  std::vector<std::shared_ptr<Follower>> followers;
  for (int i = 1; i <= number; ++i) {
    auto node = std::make_shared<Follower>(
      "TB3_" + std::to_string(i + 1),
      "TB3_" + std::to_string(i));
    followers.push_back(node);
  }

  rclcpp::executors::MultiThreadedExecutor executor;
  for (auto & node : followers) {
    executor.add_node(node);
  }

  executor.spin();
  rclcpp::shutdown();
  return 0;
}
