#include <chrono>
#include <cmath>
#include <memory>
#include <array>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono_literals;

class DummyDepthNode : public rclcpp::Node {
public:
  DummyDepthNode() : Node("dummy_depth_node") {
    // Preferred input for ArduPilot ExternalNav
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/mavros/odometry/in", 10);

    // Optional (pose + covariance)
    vision_pose_cov_pub_ =
        this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/mavros/vision_pose/pose_cov", 10);

    // Optional (velocity)
    vision_speed_pub_ =
        this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/mavros/vision_speed/speed_twist", 10);

    timer_ = this->create_wall_timer(50ms, std::bind(&DummyDepthNode::timer_callback, this));
    start_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "ExternalNav depth injector running (publishing odom + optional vision pose/speed).");
  }

private:
  void timer_callback() {
    const rclcpp::Time now = this->now();
    const double t = (now - start_time_).seconds();

    // ---- Dummy depth profile ----
    // depth(t) = 1 + 1*sin(w t)  -> 0..2 m
    const double period_s = 20.0;
    const double w = 2.0 * M_PI / period_s;

    const double depth_m = 1.0 + 1.0 * std::sin(w * t);     // positive DOWN (conceptually)
    const double depth_rate_mps = 1.0 * w * std::cos(w * t); // d(depth)/dt (positive means going down)

    // ROS is ENU: +Z up. Underwater depth means Z is negative.
    const double z_enu = -depth_m;
    const double vz_enu = -depth_rate_mps;

    // Common header fields
    const std::string map_frame = "map";
    const std::string base_frame = "base_link";

    // -----------------------------
    // 1) /mavros/odometry/in  (Preferred)
    // -----------------------------
    nav_msgs::msg::Odometry odom{};
    odom.header.stamp = now;
    odom.header.frame_id = map_frame;
    odom.child_frame_id = base_frame;

    // Position (keep x,y at 0 for this dummy)
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = z_enu;

    // Orientation (identity quaternion)
    odom.pose.pose.orientation.w = 1.0;
    odom.pose.pose.orientation.x = 0.0;
    odom.pose.pose.orientation.y = 0.0;
    odom.pose.pose.orientation.z = 0.0;

    // Velocity (optional but helpful if you have it)
    odom.twist.twist.linear.x = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = vz_enu;

    odom.twist.twist.angular.x = 0.0;
    odom.twist.twist.angular.y = 0.0;
    odom.twist.twist.angular.z = 0.0;

    // Reasonable covariances (tune as needed)
    // Pose covariance (6x6): x,y,z,roll,pitch,yaw
    // Smaller = more trusted. Don’t set all zeros.
    set_diag6(odom.pose.covariance,
              0.05 * 0.05,  // x
              0.05 * 0.05,  // y
              0.10 * 0.10,  // z (depth)
              0.5  * 0.5,   // roll
              0.5  * 0.5,   // pitch
              0.5  * 0.5);  // yaw

    // Twist covariance (6x6): vx,vy,vz,vroll,vpitch,vyaw
    set_diag6(odom.twist.covariance,
              0.10 * 0.10,  // vx
              0.10 * 0.10,  // vy
              0.15 * 0.15,  // vz
              1.0  * 1.0,   // vroll
              1.0  * 1.0,   // vpitch
              1.0  * 1.0);  // vyaw

    odom_pub_->publish(odom);

    // -----------------------------
    // 2) /mavros/vision_pose/pose_cov (Optional)
    // -----------------------------
    geometry_msgs::msg::PoseWithCovarianceStamped pose_cov{};
    pose_cov.header.stamp = now;
    pose_cov.header.frame_id = map_frame;

    pose_cov.pose.pose.position.x = 0.0;
    pose_cov.pose.pose.position.y = 0.0;
    pose_cov.pose.pose.position.z = z_enu;

    pose_cov.pose.pose.orientation = odom.pose.pose.orientation;

    // Match pose covariance (6x6) to odom’s pose covariance
    pose_cov.pose.covariance = odom.pose.covariance;

    vision_pose_cov_pub_->publish(pose_cov);

    // -----------------------------
    // 3) /mavros/vision_speed/speed_twist (Optional)
    // -----------------------------
    geometry_msgs::msg::TwistStamped twist{};
    twist.header.stamp = now;
    twist.header.frame_id = base_frame;

    twist.twist.linear.x = 0.0;
    twist.twist.linear.y = 0.0;
    twist.twist.linear.z = vz_enu;

    twist.twist.angular.x = 0.0;
    twist.twist.angular.y = 0.0;
    twist.twist.angular.z = 0.0;

    vision_speed_pub_->publish(twist);

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                         "Publishing depth=%.2f m (ENU z=%.2f), vz=%.2f m/s",
                         depth_m, z_enu, vz_enu);
  }

  // Helper: set diagonal entries of a 6x6 covariance stored in a 36-array
  static void set_diag6(std::array<double, 36> &cov,
                        double d0, double d1, double d2,
                        double d3, double d4, double d5) {
    cov.fill(0.0);
    cov[0]  = d0;
    cov[7]  = d1;
    cov[14] = d2;
    cov[21] = d3;
    cov[28] = d4;
    cov[35] = d5;
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time start_time_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr vision_pose_cov_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vision_speed_pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DummyDepthNode>());
  rclcpp::shutdown();
  return 0;
}
