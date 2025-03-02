#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"

class EthernetBridgeNode : public rclcpp::Node
{
public:
  EthernetBridgeNode() : Node("ethernet_bridge_node")
  {
    this->declare_parameter("ip_address", "192.168.1.100");
    this->declare_parameter("port", 5000);
    this->declare_parameter("bandwidth_limit", 1000000);  // 1 Mbps
    this->declare_parameter("latency_ms", 2);  // 2ms latency
    
    ip_address_ = this->get_parameter("ip_address").as_string();
    port_ = this->get_parameter("port").as_int();
    bandwidth_limit_ = this->get_parameter("bandwidth_limit").as_int();
    latency_ms_ = this->get_parameter("latency_ms").as_int();
    
    RCLCPP_INFO(this->get_logger(), "Ethernet Bridge initialized with IP: %s, Port: %d", 
                ip_address_.c_str(), port_);
    RCLCPP_INFO(this->get_logger(), "Simulated bandwidth: %d bps, Latency: %d ms", 
                bandwidth_limit_, latency_ms_);
    
    // Publishers and subscribers
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
      "ethernet_sim/cmd_vel", 10);
      
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, 
      std::bind(&EthernetBridgeNode::cmdVelCallback, this, std::placeholders::_1));
      
    scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(
      "ethernet_sim/scan", 10);
      
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "scan", 10, 
      std::bind(&EthernetBridgeNode::scanCallback, this, std::placeholders::_1));
      
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "ethernet_sim/odom", 10);
      
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10, 
      std::bind(&EthernetBridgeNode::odomCallback, this, std::placeholders::_1));
  }

private:
  template<typename T>
  void addSimulatedLatency(const T& msg, std::function<void(const T&)> publish_func)
  {
    auto timer = this->create_wall_timer(
      std::chrono::milliseconds(latency_ms_),
      [msg, publish_func]() {
        publish_func(msg);
      });
    latency_timers_.push_back(timer);
    
    // Cleanup
    while (latency_timers_.size() > 100) {
      latency_timers_.erase(latency_timers_.begin());
    }
  }
  
  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    // Latency
    addSimulatedLatency<geometry_msgs::msg::Twist>(
      *msg, 
      [this](const geometry_msgs::msg::Twist& m) {
        this->cmd_vel_pub_->publish(m);
      }
    );
  }
  
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // Latency
    addSimulatedLatency<sensor_msgs::msg::LaserScan>(
      *msg,
      [this](const sensor_msgs::msg::LaserScan& m) {
        this->scan_pub_->publish(m);
      }
    );
  }
  
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Latency
    addSimulatedLatency<nav_msgs::msg::Odometry>(
      *msg,
      [this](const nav_msgs::msg::Odometry& m) {
        this->odom_pub_->publish(m);
      }
    );
  }

  std::string ip_address_;
  int port_;
  int bandwidth_limit_;
  int latency_ms_;
  
  // Publishers and subscribers
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  
  // Store timers used for latency simulation
  std::vector<rclcpp::TimerBase::SharedPtr> latency_timers_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EthernetBridgeNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}