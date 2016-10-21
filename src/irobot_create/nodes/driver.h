#ifndef DRIVER_H
#define DRIVER_H

#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <diagnostic_updater/diagnostic_updater.h>

#include <create/Contact.h>
#include <create/IrRange.h>
#include <create/MotorSpeed.h>

#include <create/Beep.h>
#include <create/Brake.h>
#include <create/Leds.h>

#include "create/create.h"

class CreateDriver
{
private:
  create::Create* robot_;
  create::RobotModel model_;
  diagnostic_updater::Updater diagnostics_;
  ros::Time last_cmd_vel_time_;

  sensor_msgs::BatteryState battery_msg_;
  create::Contact contact_msg_;
  create::MotorSpeed motors_msg_;
  create::IrRange ir_range_msg_;
  sensor_msgs::JointState joint_state_msg_;

  bool is_running_slowly_;
  int safety_restriction_;
  bool safety_problem_;

  // ROS params
  double rate_;
  std::string dev_;
  int baud_;
  double latch_duration_;
  bool safety_;

  void cmdVelCallback(const create::MotorSpeed& msg);

  bool beepSrvCallback(create::Beep::Request& req, create::Beep::Response& res);
  bool brakeSrvCallback(create::Brake::Request& req, create::Brake::Response& res);
  bool ledsSrvCallback(create::Leds::Request& req, create::Leds::Response& res);

  bool update();
  void updateBatteryDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void updateSafetyDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void updateSerialDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void updateModeDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);
  void updateDriverDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

  void publishBatteryInfo();
  void publishContactInfo();
  void publishMotorsInfo();
  void publishIrRangeInfo();
  void publishJointInfo();

  bool applySafety();
  void beep();
  bool driveWheels(const float& leftWheel, const float& rightWheel);
  int convertChargingStatus(const create::ChargingState& status);

protected:
  ros::NodeHandle nh_;
  ros::NodeHandle priv_nh_;

  ros::Subscriber cmd_vel_sub_;

  ros::ServiceServer beep_srv_;
  ros::ServiceServer brake_srv_;
  ros::ServiceServer leds_srv_;

  ros::Publisher battery_pub_;
  ros::Publisher contact_pub_;
  ros::Publisher motors_pub_;
  ros::Publisher ir_range_pub_;
  ros::Publisher wheel_joint_pub_;

public:
  CreateDriver(ros::NodeHandle& nh);
  ~CreateDriver();
  virtual void spin();
  virtual void spinOnce();

};  // class CreateDriver

#endif  // DRIVER_H
