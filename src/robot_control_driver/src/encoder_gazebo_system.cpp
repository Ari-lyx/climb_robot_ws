// Use position-derived encoder velocity for the arm's effort controller.
// Gazebo Classic ODE can report a persistent hinge rate while the measured
// hinge angle is stationary under constrained contact. Preserve raw telemetry
// separately; do not change commands, joint positions, or stopping tolerances.
#include <gazebo_ros2_control/gazebo_system_interface.hpp>
#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <map>
#include <cmath>
#include "encoder_rate.hpp"

namespace climb_robot {
class EncoderGazeboSystem : public gazebo_ros2_control::GazeboSystemInterface {
 public:
  EncoderGazeboSystem()
      : loader_("gazebo_ros2_control", "gazebo_ros2_control::GazeboSystemInterface"),
        system_(loader_.createSharedInstance("gazebo_ros2_control/GazeboSystem")) {}

  bool initSim(rclcpp::Node::SharedPtr &node, gazebo::physics::ModelPtr model,
               const hardware_interface::HardwareInfo &info, sdf::ElementPtr sdf) override {
    raw_pub_ = node->create_publisher<sensor_msgs::msg::JointState>(
        "/arm/raw_joint_states", rclcpp::SensorDataQoS());
    return system_->initSim(node, model, info, sdf);
  }
  hardware_interface::CallbackReturn on_init(
      const hardware_interface::HardwareInfo &info) override {
    auto result = GazeboSystemInterface::on_init(info);
    return result == hardware_interface::CallbackReturn::SUCCESS ? system_->on_init(info) : result;
  }
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &state) override {
    for (auto &rate : rates_) rate.reset();
    return system_->on_activate(state);
  }
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &state) override {
    return system_->on_deactivate(state);
  }
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
    auto interfaces = system_->export_state_interfaces();
    for (const auto &state : interfaces) {
      if (state.get_interface_name() == "position") {
        positions_.push_back(state);
        velocities_.emplace(state.get_prefix_name(), 0.0);
      }
    }
    std::vector<hardware_interface::StateInterface> exported;
    for (const auto &state : interfaces) {
      if (state.get_interface_name() == "velocity") {
        raw_velocities_.push_back(state);
        const auto name = state.get_prefix_name();
        exported.emplace_back(name, "velocity", &velocities_.at(name));
      } else {
        exported.push_back(state);
      }
    }
    rates_.resize(positions_.size());
    return exported;
  }
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
    return system_->export_command_interfaces();
  }
  hardware_interface::return_type perform_command_mode_switch(
      const std::vector<std::string> &start, const std::vector<std::string> &stop) override {
    return system_->perform_command_mode_switch(start, stop);
  }
  hardware_interface::return_type read(const rclcpp::Time &time,
                                       const rclcpp::Duration &period) override {
    const auto result = system_->read(time, period);
    if (result != hardware_interface::return_type::OK) return result;
    const int64_t stamp = time.nanoseconds();
    for (size_t i = 0; i < positions_.size(); ++i) {
      const double q = positions_[i].get_value();
      if (!std::isfinite(q)) return hardware_interface::return_type::ERROR;
      velocities_.at(positions_[i].get_prefix_name()) = rates_[i].update(q, stamp);
    }
    if (raw_pub_ && (stamp < last_raw_stamp_ || stamp - last_raw_stamp_ >= 20000000)) {
      sensor_msgs::msg::JointState msg;
      msg.header.stamp = time;
      for (const auto &state : raw_velocities_) {
        msg.name.push_back(state.get_prefix_name());
        msg.velocity.push_back(state.get_value());
        for (const auto &position : positions_) {
          if (position.get_prefix_name() == state.get_prefix_name()) {
            msg.position.push_back(position.get_value());
            break;
          }
        }
      }
      raw_pub_->publish(msg);
      last_raw_stamp_ = stamp;
    }
    return result;
  }
  hardware_interface::return_type write(const rclcpp::Time &time,
                                        const rclcpp::Duration &period) override {
    return system_->write(time, period);
  }
 private:
  // Declare loader first: it must outlive the system and copied state handles.
  pluginlib::ClassLoader<gazebo_ros2_control::GazeboSystemInterface> loader_;
  std::shared_ptr<gazebo_ros2_control::GazeboSystemInterface> system_;
  std::vector<hardware_interface::StateInterface> positions_, raw_velocities_;
  std::map<std::string, double> velocities_;
  std::vector<EncoderRate> rates_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr raw_pub_;
  int64_t last_raw_stamp_ = 0;
};
}
PLUGINLIB_EXPORT_CLASS(climb_robot::EncoderGazeboSystem, gazebo_ros2_control::GazeboSystemInterface)
