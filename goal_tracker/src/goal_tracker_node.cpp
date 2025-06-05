#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <cmath>

#include "rover_msgs/msg/proprioception.hpp"

class GoalTracker : public rclcpp::Node {
    public:
    GoalTracker() : Node("goal_tracker_node") {

        // 파라미터 선언 및 기본값 설정
        this->declare_parameter("goal_x", 5.0);
        this->declare_parameter("goal_y", 3.0);
        this->get_parameter("goal_x", goal_x_);
        this->get_parameter("goal_y", goal_y_);

        // /zed/zed_node/pose 구독
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/zed/zed_node/pose", 10,
        std::bind(&GoalTracker::poseCallback, this, std::placeholders::_1));

        goal_pub_ = this->create_publisher<rover_msgs::msg::Proprioception>("/proprioception", 10);

        RCLCPP_INFO(this->get_logger(), "GoalTracker node started.");
        RCLCPP_INFO(this->get_logger(), "Goal: (%.2f, %.2f)", goal_x_, goal_y_);
  }
  private:
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Publisher<rover_msgs::msg::Proprioception>::SharedPtr goal_pub_;
  double goal_x_;
  double goal_y_;
  rclcpp::Time curr_time_;

  void poseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    // 현재 위치
    double x_r = msg->pose.position.x;
    double y_r = msg->pose.position.y;

    // 현재 orientation → yaw 추출
    tf2::Quaternion q(
      msg->pose.orientation.x,
      msg->pose.orientation.y,
      msg->pose.orientation.z,
      msg->pose.orientation.w
    );
    tf2::Matrix3x3 rot(q);
    double roll, pitch, yaw;
    rot.getRPY(roll, pitch, yaw);  // 우리는 yaw만 사용

    // 목표 방향 계산
    double dx = goal_x_ - x_r;
    double dy = goal_y_ - y_r;
    double distance = std::sqrt(dx*dx + dy*dy);
    double theta_target = std::atan2(dy, dx);
    double yaw_error = theta_target - yaw;

    // -π ~ π 범위로 정규화
    while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
    while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

    rover_msgs::msg::Proprioception pr_msg;
    pr_msg.distance = distance/11;
    pr_msg.heading = yaw_error/M_PI;

    auto curr_time_ = this->now();
    pr_msg.curr_time = curr_time_.seconds();
    goal_pub_->publish(pr_msg);

    // 출력
    RCLCPP_INFO(this->get_logger(),
      "[Distance: %.2f m] [Yaw error: %.2f deg]", distance, yaw_error * 180.0 / M_PI);
}
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GoalTracker>());
  rclcpp::shutdown();
  return 0;
}
