#include <tf/transform_datatypes.h>
#include "driver.h"

#define SAFETY_RESTRICTION_NONE		0
#define SAFETY_RESTRICTION_DONTMOVE 1
#define SAFETY_RESTRICTION_TURNONLY 2
#define SAFETY_RESTRICTION_BACKONLY 3

CreateDriver::CreateDriver(ros::NodeHandle& nh)
  : nh_(nh),
    priv_nh_("~"),
    diagnostics_(),
    model_(create::RobotModel::CREATE_1),
    is_running_slowly_(false)
{
  std::string robot_model_name = "CREATE_1";
  priv_nh_.param<double>("rate", rate_, 20.0);
  priv_nh_.param<std::string>("dev", dev_, "/dev/ttyUSB0");
  priv_nh_.param<double>("latch_cmd_duration", latch_duration_, 0.2);
  priv_nh_.param<bool>("safety", safety_, true);

  create::SerialMode serialMode;
  std::string serial_mode_str;
  priv_nh_.param<std::string>("serial_mode", serial_mode_str, "auto");
  if (serial_mode_str == "auto"){
	  serialMode = create::AUTO;
  }else if (serial_mode_str == "streaming"){
	  serialMode = create::STREAMING;
  }else if (serial_mode_str == "query"){
	  serialMode = create::QUERY;
  }else{
	  ROS_FATAL("[CREATE] Unknown serial mode.");
	  ros::shutdown();
  }

  model_ = create::RobotModel::CREATE_1;
  safety_restriction_ = SAFETY_RESTRICTION_NONE;
  safety_problem_ = false;

  priv_nh_.param<int>("baud", baud_, model_.getBaud());

  robot_ = new create::Create(model_, serialMode);

  if (!robot_->connect(dev_, baud_))
  {
    ROS_FATAL("[CREATE] Failed to establish serial connection with Create.");
    ros::shutdown();
  }

  ROS_INFO("[CREATE] Connection established.");

  // Start in full control mode
  robot_->setMode(create::MODE_FULL);

  // Show robot's battery level
  ROS_INFO("[CREATE] Battery level %.2f %%", (robot_->getBatteryCharge() / robot_->getBatteryCapacity()) * 100.0);

  // Set frame_id's
  const std::string str_base_baselink("base_link");
  battery_msg_.header.frame_id = str_base_baselink;
  contact_msg_.header.frame_id = str_base_baselink;
  motors_msg_.header.frame_id = str_base_baselink;
  ir_range_msg_.header.frame_id = str_base_baselink;

  joint_state_msg_.name.resize(2);
  joint_state_msg_.position.resize(2);
  joint_state_msg_.velocity.resize(2);
  joint_state_msg_.effort.resize(2);
  joint_state_msg_.name[0] = "left_wheel_joint";
  joint_state_msg_.name[1] = "right_wheel_joint";

  // Setup services
  beep_srv_ = nh.advertiseService("/irobot_create/beep", &CreateDriver::beepSrvCallback, this);
  brake_srv_ = nh.advertiseService("/irobot_create/brake", &CreateDriver::brakeSrvCallback, this);
  leds_srv_ = nh.advertiseService("/irobot_create/leds", &CreateDriver::ledsSrvCallback, this);

  // Setup subscribers
  cmd_vel_sub_ = nh.subscribe("/irobot_create/cmd_raw", 1, &CreateDriver::cmdVelCallback, this);

  // Setup publishers
  battery_pub_ = nh.advertise<sensor_msgs::BatteryState>("/irobot_create/battery", 30);
  contact_pub_ = nh.advertise<create::Contact>("/irobot_create/contact", 30);
  motors_pub_ = nh.advertise<create::MotorSpeed>("/irobot_create/motors", 30);
  ir_range_pub_ = nh.advertise<create::IrRange>("/irobot_create/irRange", 30);
  wheel_joint_pub_ = nh.advertise<sensor_msgs::JointState>("/irobot_create/joints", 30);

  // Setup diagnostics
  diagnostics_.add("Battery Status", this, &CreateDriver::updateBatteryDiagnostics);
  diagnostics_.add("Safety Status", this, &CreateDriver::updateSafetyDiagnostics);
  diagnostics_.add("Serial Status", this, &CreateDriver::updateSerialDiagnostics);
  diagnostics_.add("Base Mode", this, &CreateDriver::updateModeDiagnostics);
  diagnostics_.add("Driver Status", this, &CreateDriver::updateDriverDiagnostics);

  diagnostics_.setHardwareID(robot_model_name);

  beep();
  ROS_INFO("[CREATE] Ready.");
}

CreateDriver::~CreateDriver()
{
  ROS_INFO("[CREATE] Destruct sequence initiated.");
  robot_->disconnect();
  delete robot_;
}

bool CreateDriver::applySafety(){
	bool is_problem = false;

	// Cliff detection
	if (robot_->isCliff()){
		safety_restriction_ = SAFETY_RESTRICTION_BACKONLY;
		ROS_INFO_THROTTLE(10, "[CREATE] Safety activated: Cliff detection");
		ROS_INFO_THROTTLE(10, "[CREATE] Allowing back action only");
		is_problem = true;
	}

	// Bumper detection
	if (robot_->isLeftBumper() || robot_->isRightBumper() || robot_->isVirtualWall()){
		safety_restriction_ = SAFETY_RESTRICTION_BACKONLY;
		ROS_INFO_THROTTLE(10, "[CREATE] Safety activated: Collision detection with bumper");
		ROS_INFO_THROTTLE(10, "[CREATE] Allowing back action only");
		is_problem = true;
	}

	// Wheel drop detection
	if (robot_->isWheeldrop()){
		safety_restriction_ = SAFETY_RESTRICTION_DONTMOVE;
		ROS_INFO_THROTTLE(10, "[CREATE] Safety activated: Wheel drop detection");
		ROS_INFO_THROTTLE(10, "[CREATE] Allowing no action");
		is_problem = true;
	}

	if (!is_problem){
		safety_restriction_ = SAFETY_RESTRICTION_NONE;
	}

	if (!safety_problem_ && is_problem){
		// Apply brake on problem detection
		robot_->driveWheels(0.0, 0.0);
		ROS_INFO("[CREATE] Safety activated: brake enabled");
		safety_problem_ = true;
	}else if (safety_problem_ && !is_problem){
		ROS_INFO("[CREATE] Safety deactivated");
		safety_problem_ = false;
	}

	return is_problem;
}

void CreateDriver::beep(){
	const uint8_t notes[1] = {96};
	const float durations[1] = {0.25};
	robot_->defineSong(1, 1, notes, durations);
	robot_->playSong(1);
}

int CreateDriver::convertChargingStatus(const create::ChargingState& state){

	int ros_status;
	switch(state) {
		case create::CHARGE_NONE : ros_status=3; // Not charging
			break;
		case create::CHARGE_RECONDITION : ros_status=0; // Reconditioning Charging -> Unknown
			break;
		case create::CHARGE_FULL : ros_status=1; // Full charging -> Full
			break;
		case create::CHARGE_TRICKLE : ros_status=4; // Trickle Charging -> Charging
			break;
		case create::CHARGE_WAITING : ros_status=2; // Waiting -> Discharging
			break;
		case create::CHARGE_FAULT : ros_status=0; // Charging fault condition -> Unknown
			break;
	}
	return ros_status;
}

bool CreateDriver::driveWheels(const float& leftWheel, const float& rightWheel){

	bool ignored = false;
	if (safety_restriction_ == SAFETY_RESTRICTION_NONE){
		robot_->driveWheels(leftWheel, rightWheel);
	} else if (safety_restriction_ == SAFETY_RESTRICTION_TURNONLY){
		if (leftWheel == 0 || rightWheel == 0){
			robot_->driveWheels(leftWheel, rightWheel);
		}else{
			ignored = true;
		}
	} else if (safety_restriction_ == SAFETY_RESTRICTION_BACKONLY){
		if (leftWheel < 0 || rightWheel < 0){
			robot_->driveWheels(leftWheel, rightWheel);
		}else{
			ignored = true;
		}
	}else{
		ignored = true;
	}

	return ignored;
}

void CreateDriver::cmdVelCallback(const create::MotorSpeed& msg)
{
  float leftWheel = ((float) msg.left) / 1000;
  float rightWheel = ((float) msg.right) / 1000;
  driveWheels(leftWheel, rightWheel);
  last_cmd_vel_time_ = ros::Time::now();
}

bool CreateDriver::beepSrvCallback(create::Beep::Request& req, create::Beep::Response& res){
	beep();
	res.success = true;
	return true;
}

bool CreateDriver::brakeSrvCallback(create::Brake::Request& req, create::Brake::Response& res){
	robot_->driveWheels(0.0, 0.0);
	res.success = true;
	return true;
}

bool CreateDriver::ledsSrvCallback(create::Leds::Request& req, create::Leds::Response& res){
	robot_->setPowerLED(req.color, req.intensity);
	res.success = true;
	return true;
}

bool CreateDriver::update()
{
  if (safety_){
	applySafety();
  }

  publishJointInfo();
  publishContactInfo();
  publishBatteryInfo();
  publishMotorsInfo();
  publishIrRangeInfo();

  // If last velocity command was sent longer than latch duration, stop robot
  if (ros::Time::now() - last_cmd_vel_time_ >= ros::Duration(latch_duration_))
  {
    robot_->drive(0, 0);
  }

  return true;
}

void CreateDriver::updateBatteryDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  const float charge = robot_->getBatteryCharge();
  const float capacity = robot_->getBatteryCapacity();
  const create::ChargingState charging_state = robot_->getChargingState();
  const float charge_ratio = charge / capacity;

  if (charging_state == create::CHARGE_FAULT)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Charging fault reported by base");
  }
  else if (charge_ratio == 0)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Battery reports no charge");
  }
  else if (charge_ratio < 0.1)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Battery reports less than 10% charge");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Battery is OK");
  }

  stat.add("Charge (Ah)", charge);
  stat.add("Capacity (Ah)", capacity);
  stat.add("Temperature (Celsius)", robot_->getTemperature());
  stat.add("Current (A)", robot_->getCurrent());
  stat.add("Voltage (V)", robot_->getVoltage());

  switch (charging_state)
  {
    case create::CHARGE_NONE:
      stat.add("Charging state", "Not charging");
      break;
    case create::CHARGE_RECONDITION:
      stat.add("Charging state", "Reconditioning");
      break;
    case create::CHARGE_FULL:
      stat.add("Charging state", "Full charge");
      break;
    case create::CHARGE_TRICKLE:
      stat.add("Charging state", "Trickle charging");
      break;
    case create::CHARGE_WAITING:
      stat.add("Charging state", "Waiting");
      break;
    case create::CHARGE_FAULT:
      stat.add("Charging state", "Fault");
      break;
  }
}

void CreateDriver::updateSafetyDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  const bool is_wheeldrop = robot_->isWheeldrop();
  const bool is_cliff = robot_->isCliff();
  if (is_wheeldrop)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Wheeldrop detected");
  }
  else if (is_cliff)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Cliff detected");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "No safety issues detected");
  }

  stat.add("Wheeldrop", is_wheeldrop);
  stat.add("Cliff", is_cliff);
}

void CreateDriver::updateSerialDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  const bool is_connected = robot_->connected();
  const uint64_t corrupt_packets = robot_->getNumCorruptPackets();
  const uint64_t total_packets = robot_->getTotalPackets();

  if (!is_connected)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Serial port to base not open");
  }
  else if (corrupt_packets)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN,
                 "Corrupt packets detected. If the number of corrupt packets is increasing, data may be unreliable");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Serial connection is good");
  }

  stat.add("Corrupt packets", corrupt_packets);
  stat.add("Total packets", total_packets);
}

void CreateDriver::updateModeDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  const create::CreateMode mode = robot_->getMode();
  switch (mode)
  {
    case create::MODE_UNAVAILABLE:
      stat.summary(diagnostic_msgs::DiagnosticStatus::ERROR, "Unknown mode of operation");
      break;
    case create::MODE_OFF:
      stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Mode is set to OFF");
      break;
    case create::MODE_PASSIVE:
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Mode is set to PASSIVE");
      break;
    case create::MODE_SAFE:
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Mode is set to SAFE");
      break;
    case create::MODE_FULL:
      stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Mode is set to FULL");
      break;
  }
}

void CreateDriver::updateDriverDiagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  if (is_running_slowly_)
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::WARN, "Internal loop running slowly");
  }
  else
  {
    stat.summary(diagnostic_msgs::DiagnosticStatus::OK, "Maintaining loop frequency");
  }
}

void CreateDriver::publishJointInfo() {
    // Publish joint states
    float wheelRadius = model_.getWheelDiameter() / 2.0;

    // Positions of the joints are expressed in rad, and velocities in rad/sec
    joint_state_msg_.header.stamp = ros::Time::now();
    joint_state_msg_.position[0] = robot_->getLeftWheelDistance() / wheelRadius;
    joint_state_msg_.position[1] = robot_->getRightWheelDistance() / wheelRadius;
    joint_state_msg_.velocity[0] = ((float)robot_->getRequestedLeftWheelVel() / 1000.0) / wheelRadius;
    joint_state_msg_.velocity[1] = ((float)robot_->getRequestedRightWheelVel() / 1000.0) / wheelRadius;
    wheel_joint_pub_.publish(joint_state_msg_);
}

void CreateDriver::publishBatteryInfo() {

	battery_msg_.header.stamp =         ros::Time::now();
	battery_msg_.voltage =              robot_->getVoltage();
	battery_msg_.current =              robot_->getCurrent();
	battery_msg_.charge  =              robot_->getBatteryCharge();
	battery_msg_.capacity =             robot_->getBatteryCapacity();
	battery_msg_.percentage =           float(robot_->getBatteryCharge())/float(robot_->getBatteryCapacity());
	battery_msg_.design_capacity = 		float(3.0);
	battery_msg_.power_supply_status =  convertChargingStatus(robot_->getChargingState());
	battery_msg_.present =              true;

	battery_pub_.publish(battery_msg_);
}

void CreateDriver::publishContactInfo() {

	contact_msg_.header.stamp = ros::Time::now();
	contact_msg_.wheeldropCaster = robot_->isCasterWheeldrop();
	contact_msg_.wheeldropLeft =   robot_->isLeftWheeldrop();
	contact_msg_.wheeldropRight =  robot_->isRightWheeldrop();
	contact_msg_.bumpLeft =        robot_->isLeftBumper();
	contact_msg_.bumpRight =       robot_->isRightBumper();
	contact_msg_.wall =            robot_->isWall();
	contact_msg_.cliffLeft =       robot_->isLeftCliff();
	contact_msg_.cliffFrontLeft =  robot_->isFrontLeftCliff();
	contact_msg_.cliffRight =      robot_->isRightCliff();
	contact_msg_.cliffFrontRight = robot_->isFrontRightCliff();
	contact_msg_.virtualWall =     robot_->isVirtualWall();

	contact_pub_.publish(contact_msg_);
}


void CreateDriver::publishMotorsInfo(){

	motors_msg_.header.stamp =      ros::Time::now();
	motors_msg_.left =    			robot_->getRequestedLeftWheelVel();
	motors_msg_.right =   			robot_->getRequestedRightWheelVel();

	motors_pub_.publish(motors_msg_);
}

void CreateDriver::publishIrRangeInfo(){

	ir_range_msg_.header.stamp =      	   ros::Time::now();
	ir_range_msg_.wallSignal =             robot_->getWallSignal();
	ir_range_msg_.cliffLeftSignal =        robot_->getCliffLeftSignal();
	ir_range_msg_.cliffFrontLeftSignal =   robot_->getCliffFrontLeftSignal();
	ir_range_msg_.cliffFrontRightSignal =  robot_->getCliffFrontRightSignal();
	ir_range_msg_.cliffRightSignal =       robot_->getCliffRightSignal();

	ir_range_pub_.publish(ir_range_msg_);
}

void CreateDriver::spinOnce()
{
  update();
  diagnostics_.update();
  ros::spinOnce();
}

void CreateDriver::spin()
{
  ros::Rate rate(rate_);
  while (ros::ok())
  {
    spinOnce();

    is_running_slowly_ = !rate.sleep();
    if (is_running_slowly_)
    {
    	ROS_WARN_THROTTLE(1, "[CREATE] Loop running slowly.");
    }
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "irobot_create");
  ros::NodeHandle nh;

  CreateDriver create_driver(nh);

  try
  {
    create_driver.spin();
  }
  catch (std::runtime_error& ex)
  {
    ROS_FATAL_STREAM("[CREATE] Runtime error: " << ex.what());
    return 1;
  }
  return 0;
}
