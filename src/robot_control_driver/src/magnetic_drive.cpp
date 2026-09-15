#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <array>
#include <atomic>
#include <mutex>
#include <limits>
#include "magnet_math.hpp"

namespace climb_robot {
class MagneticDrive : public gazebo::ModelPlugin {
 public:
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_=model;world_=model->GetWorld();node_=gazebo_ros::Node::Get(sdf);
    auto param=[&](const std::string &key, double fallback) {
      const double value=sdf->Get<double>(key,fallback).first;
      if (!std::isfinite(value)||value<=0) throw std::runtime_error(key+" must be finite and positive");
      return value;
    };
    radius_=param("wheel_radius",0.075);width_=param("wheel_width",0.045);track_=param("track_width",0.36);
    torque_=param("motor_torque",8);gain_=param("velocity_gain",2);integral_gain_=param("velocity_integral_gain",8);vmax_=param("max_linear_speed",0.35);
    wmax_=param("max_angular_speed",0.8);accel_=param("wheel_acceleration",8);timeout_=param("command_timeout",0.5);
    force_=param("magnet_force",90);decay_=param("magnet_decay",0.012);range_=param("magnet_range",0.04);
    tank_name_=sdf->Get<std::string>("tank_model","tank").first;
    for (size_t i=0;i<4;++i) {
      wheels_[i]=model_->GetLink(names_[i]+"_wheel");joints_[i]=model_->GetJoint(names_[i]+"_joint");
      if (!wheels_[i]||!joints_[i]) {RCLCPP_FATAL(node_->get_logger(),"Missing wheel %s",names_[i].c_str());return;}
    }
    base_=model_->GetLink("base_link");
    cmd_sub_=node_->create_subscription<geometry_msgs::msg::Twist>("cmd_vel",1,[this](geometry_msgs::msg::Twist::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!std::isfinite(msg->linear.x)||!std::isfinite(msg->angular.z)) {linear_=angular_=0;return;}
      linear_=std::clamp(msg->linear.x,-vmax_,vmax_);angular_=std::clamp(msg->angular.z,-wmax_,wmax_);
      received_at_=sim_time_.load();
    });
    magnet_service_=node_->create_service<std_srvs::srv::SetBool>("magnet/enable",[this](std::shared_ptr<std_srvs::srv::SetBool::Request> req,std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
      enabled_=req->data;res->success=true;res->message=req->data?"Magnetic attraction enabled":"Magnetic attraction disabled";
    });
    odom_pub_=node_->create_publisher<nav_msgs::msg::Odometry>("odom",10);
    joints_pub_=node_->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
    diag_pub_=node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("magnet/diagnostics",10);
    tf_=std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    last_time_=world_->SimTime().Double();last_publish_=last_time_;
    update_=gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&MagneticDrive::Update,this));
    RCLCPP_INFO(node_->get_logger(),"Four-wheel torque drive and finite-range magnetic attraction ready");
  }
  void Reset() override {
    std::lock_guard<std::mutex> lock(mutex_);
    linear_=angular_=0;received_at_=-1e9;target_.fill(0);integral_.fill(0);
    last_time_=world_->SimTime().Double();last_publish_=last_time_;sim_time_=last_time_;
  }
 private:
  void Update() {
    const double now=world_->SimTime().Double();sim_time_=now;
    if (now<last_time_) {Reset();return;}
    const double dt=now-last_time_;last_time_=now;if(dt<=0)return;
    double linear,angular;
    {std::lock_guard<std::mutex> lock(mutex_);const bool fresh=now-received_at_<=timeout_&&now>=received_at_;
      linear=fresh?linear_:0;angular=fresh?angular_:0;}
    // Look up every step: deleting/replacing the tank immediately removes the old field.
    auto tank=world_->ModelByName(tank_name_);
    gazebo::physics::CollisionPtr shell;
    bool hemisphere=false;
    if(tank&&tank->IsStatic()) {
      auto link=tank->GetLink("shell");
      if(link) {shell=link->GetCollision("hemisphere_shell");hemisphere=bool(shell);
        if(!shell)shell=link->GetCollision("sphere_shell");}
    }
    double tank_radius=0;
    ignition::math::Pose3d shell_pose;
    if(shell) {
      auto geometry=shell->GetSDF()->GetElement("geometry");
      if(geometry->HasElement("mesh")) {
        const auto scale=geometry->GetElement("mesh")->Get<ignition::math::Vector3d>("scale");
        if(std::abs(scale.X()-scale.Y())<1e-9&&std::abs(scale.X()-scale.Z())<1e-9) tank_radius=scale.X();
      }
      shell_pose=shell->WorldPose();
    }
    for(size_t i=0;i<4;++i) {
      const double desired=(linear+(i%2==0?-1:1)*angular*track_/2)/radius_;
      target_[i]+=std::clamp(desired-target_[i],-accel_*dt,accel_*dt);
      const double error=target_[i]-joints_[i]->GetVelocity(0);
      const double candidate=integral_[i]+integral_gain_*error*dt;
      const double raw=gain_*error+candidate;
      // Conditional integration prevents windup under torque saturation.
      if(std::abs(raw)<=torque_ || raw*error<0)
        integral_[i]=std::clamp(candidate,-torque_,torque_);
      const double effort=std::clamp(gain_*error+integral_[i],-torque_,torque_);
      joints_[i]->SetForce(0,effort);
      forces_[i]=0;gaps_[i]=std::numeric_limits<double>::quiet_NaN();
      if(!shell||tank_radius<=0)continue;
      const auto position=wheels_[i]->WorldCoGPose().Pos();
      auto delta=position-shell_pose.Pos();const double distance=delta.Length();
      if(distance<1e-6)continue;
      const auto n=delta/distance;
      const auto local_n=shell_pose.Rot().RotateVectorReverse(n);
      // No field across the open rim or on the outer side of the tank.
      if((hemisphere&&local_n.Z()>0)||distance>tank_radius)continue;
      const auto axle=wheels_[i]->WorldPose().Rot().RotateVector(ignition::math::Vector3d::UnitY);
      gaps_[i]=tank_radius-distance-cylinderSupport(radius_,width_,axle.Dot(n));
      if(enabled_)forces_[i]=magneticForce(gaps_[i],force_,decay_,range_);
      // Symmetric permanent-magnet wheel resultant through wheel COM; no artificial spin torque.
      wheels_[i]->AddForce(n*forces_[i]);
    }
    if(now-last_publish_>=0.02) {Publish(now);last_publish_=now;}
  }
  void Publish(double now) {
    const auto stamp=rclcpp::Time(static_cast<int64_t>(now*1e9));
    const auto pose=base_->WorldPose();const auto q=pose.Rot();
    nav_msgs::msg::Odometry odom;odom.header.stamp=stamp;odom.header.frame_id="world";odom.child_frame_id="base_link";
    odom.pose.pose.position.x=pose.Pos().X();odom.pose.pose.position.y=pose.Pos().Y();odom.pose.pose.position.z=pose.Pos().Z();
    odom.pose.pose.orientation.x=q.X();odom.pose.pose.orientation.y=q.Y();odom.pose.pose.orientation.z=q.Z();odom.pose.pose.orientation.w=q.W();
    const auto v=q.RotateVectorReverse(base_->WorldLinearVel());const auto w=q.RotateVectorReverse(base_->WorldAngularVel());
    odom.twist.twist.linear.x=v.X();odom.twist.twist.linear.y=v.Y();odom.twist.twist.linear.z=v.Z();
    odom.twist.twist.angular.x=w.X();odom.twist.twist.angular.y=w.Y();odom.twist.twist.angular.z=w.Z();odom_pub_->publish(odom);
    geometry_msgs::msg::TransformStamped transform;transform.header=odom.header;transform.child_frame_id="base_link";
    transform.transform.translation.x=pose.Pos().X();transform.transform.translation.y=pose.Pos().Y();transform.transform.translation.z=pose.Pos().Z();transform.transform.rotation=odom.pose.pose.orientation;tf_->sendTransform(transform);
    sensor_msgs::msg::JointState state;state.header=odom.header;
    diagnostic_msgs::msg::DiagnosticArray diagnostics;diagnostics.header=odom.header;
    for(size_t i=0;i<4;++i) {
      state.name.push_back(names_[i]+"_joint");state.position.push_back(joints_[i]->Position(0));state.velocity.push_back(joints_[i]->GetVelocity(0));
      diagnostic_msgs::msg::DiagnosticStatus status;status.name="magnet/"+names_[i];status.hardware_id="simulated_permanent_magnet";
      status.level=diagnostic_msgs::msg::DiagnosticStatus::OK;status.message=forces_[i]>0?"attracting":(enabled_?"no surface in capture range":"disabled");
      auto add=[&](const std::string &key,const std::string &value){diagnostic_msgs::msg::KeyValue kv;kv.key=key;kv.value=value;status.values.push_back(kv);};
      add("gap_m",std::to_string(gaps_[i]));add("force_N",std::to_string(forces_[i]));add("enabled",enabled_?"true":"false");diagnostics.status.push_back(status);
    }
    joints_pub_->publish(state);diag_pub_->publish(diagnostics);
  }
  gazebo::physics::ModelPtr model_;gazebo::physics::WorldPtr world_;gazebo::physics::LinkPtr base_;
  std::array<gazebo::physics::LinkPtr,4>wheels_;std::array<gazebo::physics::JointPtr,4>joints_;
  const std::array<std::string,4>names_={"front_left","front_right","rear_left","rear_right"};
  std::array<double,4>target_{},integral_{},forces_{},gaps_{};
  gazebo::event::ConnectionPtr update_;gazebo_ros::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr magnet_service_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  std::mutex mutex_;std::atomic<bool>enabled_{true};std::atomic<double>sim_time_{0};
  double linear_=0,angular_=0,received_at_=-1e9,last_time_=0,last_publish_=0;
  double radius_,width_,track_,torque_,gain_,integral_gain_,vmax_,wmax_,accel_,timeout_,force_,decay_,range_;
  std::string tank_name_;
};
GZ_REGISTER_MODEL_PLUGIN(MagneticDrive)
} // namespace climb_robot
