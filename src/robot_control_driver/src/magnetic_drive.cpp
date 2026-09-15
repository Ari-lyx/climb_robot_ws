// 磁吸 + 四轮差速驱动的 Gazebo Classic ModelPlugin。
//
// 【整体思路】本插件不做“把机器人钉在墙上”的作弊，而是每个物理步做两件物理上正当的事：
//   1. 差速驱动：订阅 /cmd_vel，换算成四个轮子的目标角速度，用带抗积分饱和的
//      速度环算出关节扭矩，交给 ODE 求解器（滑移转向，靠接触摩擦产生牵引）。
//   2. 磁吸：对每个轮子算它到球壳内壁的“轮面间隙” g，用近场衰减公式
//      F = F_contact·taper(g)/(1+g/decay)² 得到吸力，沿壁面内法线方向施加到轮心。
//      法向吸力只提供压紧力，**不产生额外力矩**；沿壁面的重力由轮胎摩擦力与
//      电机扭矩平衡（这就是能爬上去的物理原因）。
// 吸力是“有界、有捕获距离”的：离开球壳、驶出半球开口、间隙超过 magnet_range、
// 或通过 /magnet/enable 关闭时，吸力恒为 0。
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
#include "magnet_math.hpp"   // 纯函数：cylinderSupport() / magneticForce()

namespace climb_robot {
// GZ_REGISTER_MODEL_PLUGIN 要求类可默认构造，并实现 Load/Reset 两个虚函数。
class MagneticDrive : public gazebo::ModelPlugin {
 public:
  // Load：Gazebo 加载模型时调用一次。读取 URDF <plugin> 里的参数、抓取四个轮子
  // 的 link/joint、创建 ROS 订阅/服务/发布器，并挂上每个物理步的回调。
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_=model;world_=model->GetWorld();node_=gazebo_ros::Node::Get(sdf);
    // 统一参数读取：从 SDF 取同名子标签，缺省值兜底，并强制“有限且为正”。
    // 这里的 "wheel_radius"、"magnet_force" 等字符串就是 xacro 里写的标签名。
    auto param=[&](const std::string &key, double fallback) {
      const double value=sdf->Get<double>(key,fallback).first;
      if (!std::isfinite(value)||value<=0) throw std::runtime_error(key+" must be finite and positive");
      return value;
    };
    // 几何与驱动参数
    radius_=param("wheel_radius",0.075);width_=param("wheel_width",0.045);track_=param("track_width",0.36);
    torque_=param("motor_torque",8);gain_=param("velocity_gain",2);integral_gain_=param("velocity_integral_gain",8);vmax_=param("max_linear_speed",0.35);
    wmax_=param("max_angular_speed",0.8);accel_=param("wheel_acceleration",8);timeout_=param("command_timeout",0.5);
    // 磁吸三参数：吸力上限[N]、衰减长度[m]、捕获距离[m]
    force_=param("magnet_force",90);decay_=param("magnet_decay",0.012);range_=param("magnet_range",0.04);
    // 要吸附的模型名（默认 "tank"，对应 world.py 生成的球罐模型）
    tank_name_=sdf->Get<std::string>("tank_model","tank").first;
    // 按固定顺序抓取四轮：names_ 的顺序同时决定“左右”的符号约定（偶数索引=左轮）
    for (size_t i=0;i<4;++i) {
      wheels_[i]=model_->GetLink(names_[i]+"_wheel");joints_[i]=model_->GetJoint(names_[i]+"_joint");
      if (!wheels_[i]||!joints_[i]) {RCLCPP_FATAL(node_->get_logger(),"Missing wheel %s",names_[i].c_str());return;}
    }
    base_=model_->GetLink("base_link");
    // /cmd_vel 回调只做“限幅 + 记录时间戳”，真正算扭矩在 Update() 里（与物理步同步），
    // 避免任意频率的消息直接改物理状态，保证可复现。
    cmd_sub_=node_->create_subscription<geometry_msgs::msg::Twist>("cmd_vel",1,[this](geometry_msgs::msg::Twist::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!std::isfinite(msg->linear.x)||!std::isfinite(msg->angular.z)) {linear_=angular_=0;return;}
      linear_=std::clamp(msg->linear.x,-vmax_,vmax_);angular_=std::clamp(msg->angular.z,-wmax_,wmax_);
      received_at_=sim_time_.load();
    });
    // /magnet/enable：仿真实验接口，用于验证“关掉磁吸就会掉下来”（实物磁铁做不到）
    magnet_service_=node_->create_service<std_srvs::srv::SetBool>("magnet/enable",[this](std::shared_ptr<std_srvs::srv::SetBool::Request> req,std::shared_ptr<std_srvs::srv::SetBool::Response> res) {
      enabled_=req->data;res->success=true;res->message=req->data?"Magnetic attraction enabled":"Magnetic attraction disabled";
    });
    odom_pub_=node_->create_publisher<nav_msgs::msg::Odometry>("odom",10);
    joints_pub_=node_->create_publisher<sensor_msgs::msg::JointState>("joint_states",10);
    diag_pub_=node_->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("magnet/diagnostics",10);
    tf_=std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    last_time_=world_->SimTime().Double();last_publish_=last_time_;
    // 每个物理步回调一次 Update()：这是施力与算扭矩的唯一时机
    update_=gazebo::event::Events::ConnectWorldUpdateBegin(std::bind(&MagneticDrive::Update,this));
    RCLCPP_INFO(node_->get_logger(),"Four-wheel torque drive and finite-range magnetic attraction ready");
  }
  // Reset：Gazebo 复位仿真时调用（如 ros2 service call /reset_simulation），
  // 清空速度命令与积分项，避免复位后残量导致机器人突然窜出。
  void Reset() override {
    std::lock_guard<std::mutex> lock(mutex_);
    linear_=angular_=0;received_at_=-1e9;target_.fill(0);integral_.fill(0);
    last_time_=world_->SimTime().Double();last_publish_=last_time_;sim_time_=last_time_;
  }
 private:
  // ============ 每个物理步的核心逻辑 ============
  void Update() {
    const double now=world_->SimTime().Double();sim_time_=now;
    // 时间回退（复位/回放）时重新初始化，避免 dt<0
    if (now<last_time_) {Reset();return;}
    const double dt=now-last_time_;last_time_=now;if(dt<=0)return;
    // ---- (a) 取当前速度命令：超过 command_timeout 未收到新命令 => 视为 0（停车制动）----
    double linear,angular;
    {std::lock_guard<std::mutex> lock(mutex_);const bool fresh=now-received_at_<=timeout_&&now>=received_at_;
      linear=fresh?linear_:0;angular=fresh?angular_:0;}
    // ---- (b) 每步重新查找球壳：删掉/替换球罐时磁场立即消失，无需重启插件 ----
    // 球罐由 world.py 生成：模型名 "tank"、link 名 "shell"，碰撞名按半球/整球
    // 分别叫 hemisphere_shell / sphere_shell（用后者之名判断是否为开口半球）。
    // Look up every step: deleting/replacing the tank immediately removes the old field.
    auto tank=world_->ModelByName(tank_name_);
    gazebo::physics::CollisionPtr shell;
    bool hemisphere=false;
    if(tank&&tank->IsStatic()) {
      auto link=tank->GetLink("shell");
      if(link) {shell=link->GetCollision("hemisphere_shell");hemisphere=bool(shell);
        if(!shell)shell=link->GetCollision("sphere_shell");}
    }
    // ---- (c) 从碰撞体元数据反推球罐半径与位姿 ----
    // 网格按单位半径生成再用 <scale> 缩放，所以 scale 就等于球罐内半径；
    // 三轴缩放不一致说明不是球面，此时视为无效（tank_radius=0 => 无磁力）。
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
      // ---- (d) 差速运动学：把车体 (v, ω) 换算成每个轮子的目标角速度 ----
      // 左轮 = (v - ω·B/2)/R，右轮 = (v + ω·B/2)/R（B = track_）。
      // i%2==0 是左轮（见 names_ 顺序），所以左侧减、右侧加。
      // 目标角速度再按 wheel_acceleration 做斜率限幅，避免阶跃命令造成打滑/翻车。
      const double desired=(linear+(i%2==0?-1:1)*angular*track_/2)/radius_;
      target_[i]+=std::clamp(desired-target_[i],-accel_*dt,accel_*dt);
      // ---- (e) 速度环 PI + 条件积分抗饱和 ----
      // error 为目标与实测角速度之差；输出 effort 被限幅在 ±motor_torque 内。
      const double error=target_[i]-joints_[i]->GetVelocity(0);
      const double candidate=integral_[i]+integral_gain_*error*dt;
      const double raw=gain_*error+candidate;
      // Conditional integration prevents windup under torque saturation.
      // 中文说明：只有当“未饱和”或“误差方向与积分方向相反（正在回退）”时才接受积分，
      // 否则扭矩饱和期间积分会越积越大，松手后产生超调。
      if(std::abs(raw)<=torque_ || raw*error<0)
        integral_[i]=std::clamp(candidate,-torque_,torque_);
      const double effort=std::clamp(gain_*error+integral_[i],-torque_,torque_);
      joints_[i]->SetForce(0,effort);
      // ---- (f) 磁吸：默认无吸力，只有满足全部条件才施力 ----
      forces_[i]=0;gaps_[i]=std::numeric_limits<double>::quiet_NaN();  // NaN 表示“没有有效壁面”
      if(!shell||tank_radius<=0)continue;
      // n = 由球心指向轮心的单位向量（对球内壁来说，这就是把轮子压向壁面的方向）
      const auto position=wheels_[i]->WorldCoGPose().Pos();
      auto delta=position-shell_pose.Pos();const double distance=delta.Length();
      if(distance<1e-6)continue;
      const auto n=delta/distance;
      const auto local_n=shell_pose.Rot().RotateVectorReverse(n);
      // No field across the open rim or on the outer side of the tank.
      // 中文说明：半球开口(局部 z>0)之外没有可吸附的壳；distance>半径说明在壳外。
      // 两种情况磁场都为 0 —— 所以驶出下半球边缘就会脱落。
      if((hemisphere&&local_n.Z()>0)||distance>tank_radius)continue;
      // 轮轴方向（世界系），用于把“轮心到壁面距离”修正成真实轮面间隙：
      //   g = R_tank - |p_wheel - center| - cylinderSupport(...)
      const auto axle=wheels_[i]->WorldPose().Rot().RotateVector(ignition::math::Vector3d::UnitY);
      gaps_[i]=tank_radius-distance-cylinderSupport(radius_,width_,axle.Dot(n));
      if(enabled_)forces_[i]=magneticForce(gaps_[i],force_,decay_,range_);
      // Symmetric permanent-magnet wheel resultant through wheel COM; no artificial spin torque.
      // 中文说明：沿 n 施加在轮心（质心），等效于“磁铁对钢壁的合力”，
      // 不加人为力矩——车轮转动仍完全由接触摩擦与关节扭矩决定。
      wheels_[i]->AddForce(n*forces_[i]);
    }
    // 以 50 Hz 发布话题（物理步 1000 Hz 太高，没必要全发）
    if(now-last_publish_>=0.02) {Publish(now);last_publish_=now;}
  }
  // ============ 话题发布：odom / tf / joint_states / 磁吸诊断 ============
  void Publish(double now) {
    const auto stamp=rclcpp::Time(static_cast<int64_t>(now*1e9));
    const auto pose=base_->WorldPose();const auto q=pose.Rot();
    // /odom 是仿真真值（含完整三维姿态），不是轮式里程计积分结果
    nav_msgs::msg::Odometry odom;odom.header.stamp=stamp;odom.header.frame_id="world";odom.child_frame_id="base_link";
    odom.pose.pose.position.x=pose.Pos().X();odom.pose.pose.position.y=pose.Pos().Y();odom.pose.pose.position.z=pose.Pos().Z();
    odom.pose.pose.orientation.x=q.X();odom.pose.pose.orientation.y=q.Y();odom.pose.pose.orientation.z=q.Z();odom.pose.pose.orientation.w=q.W();
    // 速度先旋转到机体坐标系再发布（ROS 约定：twist 在 child_frame 下）
    const auto v=q.RotateVectorReverse(base_->WorldLinearVel());const auto w=q.RotateVectorReverse(base_->WorldAngularVel());
    odom.twist.twist.linear.x=v.X();odom.twist.twist.linear.y=v.Y();odom.twist.twist.linear.z=v.Z();
    odom.twist.twist.angular.x=w.X();odom.twist.twist.angular.y=w.Y();odom.twist.twist.angular.z=w.Z();odom_pub_->publish(odom);
    // world -> base_link 动态 TF（轮子与雷达的静态 TF 由 robot_state_publisher 发布）
    geometry_msgs::msg::TransformStamped transform;transform.header=odom.header;transform.child_frame_id="base_link";
    transform.transform.translation.x=pose.Pos().X();transform.transform.translation.y=pose.Pos().Y();transform.transform.translation.z=pose.Pos().Z();transform.transform.rotation=odom.pose.pose.orientation;tf_->sendTransform(transform);
    sensor_msgs::msg::JointState state;state.header=odom.header;
    diagnostic_msgs::msg::DiagnosticArray diagnostics;diagnostics.header=odom.header;
    // 每轮一条诊断：可直接看到“这个轮子有没有吸住、间隙多少、力多大”
    for(size_t i=0;i<4;++i) {
      state.name.push_back(names_[i]+"_joint");state.position.push_back(joints_[i]->Position(0));state.velocity.push_back(joints_[i]->GetVelocity(0));
      diagnostic_msgs::msg::DiagnosticStatus status;status.name="magnet/"+names_[i];status.hardware_id="simulated_permanent_magnet";
      status.level=diagnostic_msgs::msg::DiagnosticStatus::OK;status.message=forces_[i]>0?"attracting":(enabled_?"no surface in capture range":"disabled");
      auto add=[&](const std::string &key,const std::string &value){diagnostic_msgs::msg::KeyValue kv;kv.key=key;kv.value=value;status.values.push_back(kv);};
      add("gap_m",std::to_string(gaps_[i]));add("force_N",std::to_string(forces_[i]));add("enabled",enabled_?"true":"false");diagnostics.status.push_back(status);
    }
    joints_pub_->publish(state);diag_pub_->publish(diagnostics);
  }
  // ============ 成员变量 ============
  gazebo::physics::ModelPtr model_;gazebo::physics::WorldPtr world_;gazebo::physics::LinkPtr base_;
  std::array<gazebo::physics::LinkPtr,4>wheels_;std::array<gazebo::physics::JointPtr,4>joints_;
  // 顺序即索引约定：0/2 为左侧，1/3 为右侧（差速公式依赖这一点）
  const std::array<std::string,4>names_={"front_left","front_right","rear_left","rear_right"};
  std::array<double,4>target_{},integral_{},forces_{},gaps_{};   // 目标角速度 / 积分项 / 吸力 / 间隙
  gazebo::event::ConnectionPtr update_;gazebo_ros::Node::SharedPtr node_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr magnet_service_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_;
  // cmd_vel 回调在 ROS 线程、Update 在物理线程，共享变量用 mutex 保护；
  // enabled_ / sim_time_ 是简单标量，用 atomic 即可。
  std::mutex mutex_;std::atomic<bool>enabled_{true};std::atomic<double>sim_time_{0};
  double linear_=0,angular_=0,received_at_=-1e9,last_time_=0,last_publish_=0;
  // 参数缓存：几何 radius_/width_/track_，驱动 torque_/gain_/integral_gain_/vmax_/wmax_/accel_/timeout_，
  // 磁吸 force_/decay_/range_
  double radius_,width_,track_,torque_,gain_,integral_gain_,vmax_,wmax_,accel_,timeout_,force_,decay_,range_;
  std::string tank_name_;
};
// 注册为 Gazebo 模型插件；名字与 URDF 里 filename="libmagnetic_drive.so" 对应
GZ_REGISTER_MODEL_PLUGIN(MagneticDrive)
} // namespace climb_robot
