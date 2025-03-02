#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

class EthernetControllerNode : public rclcpp::Node
{
public:
  EthernetControllerNode() : Node("ethernet_controller_node")
  {
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "ethernet_sim/cmd_vel", 10);
      
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "ethernet_sim/scan", 10, 
      std::bind(&EthernetControllerNode::scanCallback, this, std::placeholders::_1));
      
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "ethernet_sim/odom", 10, 
      std::bind(&EthernetControllerNode::odomCallback, this, std::placeholders::_1));
      
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),
      std::bind(&EthernetControllerNode::controlLoop, this));
      
    RCLCPP_INFO(this->get_logger(), "Ethernet controller node initialized");
  }

private:
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    latest_scan_ = msg;
    RCLCPP_DEBUG(this->get_logger(), "Received scan data via ethernet");
  }
  
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    latest_odom_ = msg;
    RCLCPP_DEBUG(this->get_logger(), "Received odometry data via ethernet");
  }
  
  void controlLoop()
  {
    if (!latest_scan_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                          "Waiting for scan data...");
      return;
    }
    
    auto twist = geometry_msgs::msg::Twist();
    
    bool obstacle_detected = false;
    for (size_t i = 0; i < latest_scan_->ranges.size(); i++) {
      float angle = latest_scan_->angle_min + i * latest_scan_->angle_increment;
      if (angle > -0.3 && angle < 0.3) {  // ~30 degree arc in front
        if (latest_scan_->ranges[i] < 0.5 && latest_scan_->ranges[i] > 0.01) {  // 0.5m threshold
          obstacle_detected = true;
          break;
        }
      }
    }
    
    if (obstacle_detected) {
      twist.linear.x = 0.0;
      twist.angular.z = 0.5;
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                          "Obstacle detected! Turning...");
    } else {
      // Move forward
      twist.linear.x = 0.2;
      twist.angular.z = 0.0;
    }
    
    cmd_vel_pub_->publish(twist);
  }

  // Publishers and subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  sensor_msgs::msg::LaserScan::SharedPtr latest_scan_;
  nav_msgs::msg::Odometry::SharedPtr latest_odom_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EthernetControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}