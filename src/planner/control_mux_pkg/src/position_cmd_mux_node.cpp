#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8.hpp>
#include <quadrotor_msgs/msg/position_command.hpp>

class PositionCmdMux : public rclcpp::Node
{
public:
  PositionCmdMux() : Node("position_cmd_mux"), current_mode_(Mode::TRACK_TRAJ)
  {
    planner_cmd_sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
        "planner_cmd", 10, std::bind(&PositionCmdMux::plannerCallback, this, std::placeholders::_1));

    pid_cmd_sub_ = create_subscription<quadrotor_msgs::msg::PositionCommand>(
        "pid_cmd", 10, std::bind(&PositionCmdMux::pidCallback, this, std::placeholders::_1));

    mode_sub_ = create_subscription<std_msgs::msg::UInt8>(
        "control_mode", 10, std::bind(&PositionCmdMux::modeCallback, this, std::placeholders::_1));

    cmd_pub_ = create_publisher<quadrotor_msgs::msg::PositionCommand>("position_cmd_out", 50);

    timer_ = create_wall_timer(std::chrono::milliseconds(10), std::bind(&PositionCmdMux::publishCallback, this));

    RCLCPP_INFO(get_logger(), "PositionCmdMux initialized. Modes: 0=TRACK_TRAJ, 1=HOVER_PID, 2=EMERGENCY");
  }

private:
  enum Mode
  {
    TRACK_TRAJ = 0,
    HOVER_PID = 1,
    EMERGENCY = 2
  };

  void plannerCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    planner_cmd_ = *msg;
    planner_cmd_stamp_ = now();
  }

  void pidCallback(const quadrotor_msgs::msg::PositionCommand::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pid_cmd_ = *msg;
    pid_cmd_stamp_ = now();
  }

  void modeCallback(const std_msgs::msg::UInt8::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (msg->data <= Mode::EMERGENCY)
    {
      current_mode_ = static_cast<Mode>(msg->data);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 2000, "Control mode changed to: %d", current_mode_);
    }
  }

  void publishCallback()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    quadrotor_msgs::msg::PositionCommand cmd_to_publish;
    bool valid = false;

    double timeout_sec = 0.1;

    switch (current_mode_)
    {
    case Mode::TRACK_TRAJ:
      if ((now() - planner_cmd_stamp_).seconds() < timeout_sec)
      {
        cmd_to_publish = planner_cmd_;
        valid = true;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "TRACK_TRAJ: planner cmd timeout, fallback to EMERGENCY");
        current_mode_ = Mode::EMERGENCY;
      }
      break;

    case Mode::HOVER_PID:
      if ((now() - pid_cmd_stamp_).seconds() < timeout_sec)
      {
        cmd_to_publish = pid_cmd_;
        valid = true;
      }
      else
      {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "HOVER_PID: pid cmd timeout, fallback to EMERGENCY");
        current_mode_ = Mode::EMERGENCY;
      }
      break;

    case Mode::EMERGENCY:
    default:
      cmd_to_publish.header.stamp = now();
      cmd_to_publish.header.frame_id = "world";
      cmd_to_publish.velocity.x = 0.0;
      cmd_to_publish.velocity.y = 0.0;
      cmd_to_publish.velocity.z = 0.0;
      cmd_to_publish.acceleration.x = 0.0;
      cmd_to_publish.acceleration.y = 0.0;
      cmd_to_publish.acceleration.z = 0.0;
      cmd_to_publish.trajectory_flag = quadrotor_msgs::msg::PositionCommand::TRAJECTORY_STATUS_READY;
      valid = true;
      break;
    }

    if (valid)
    {
      cmd_pub_->publish(cmd_to_publish);
    }
  }

  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr planner_cmd_sub_;
  rclcpp::Subscription<quadrotor_msgs::msg::PositionCommand>::SharedPtr pid_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr mode_sub_;
  rclcpp::Publisher<quadrotor_msgs::msg::PositionCommand>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex mutex_;
  Mode current_mode_;

  quadrotor_msgs::msg::PositionCommand planner_cmd_;
  quadrotor_msgs::msg::PositionCommand pid_cmd_;
  rclcpp::Time planner_cmd_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time pid_cmd_stamp_{0, 0, RCL_ROS_TIME};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PositionCmdMux>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
