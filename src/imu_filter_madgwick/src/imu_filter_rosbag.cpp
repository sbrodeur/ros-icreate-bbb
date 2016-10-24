#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include "imu_filter_madgwick/imu_filter.h"
#include "imu_filter_madgwick/stateless_orientation.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using namespace std;

// NOTE: adapted from http://wiki.ros.org/rosbag/Cookbook

/**
 * Inherits from message_filters::SimpleFilter<M>
 * to use protected signalMessage function
 */
template <class M>
class BagSubscriber : public message_filters::SimpleFilter<M>
{
public:
  void newMessage(const boost::shared_ptr<M const> &msg)
  {
    this->signalMessage(msg);
  }
};


class ImuFilterRosbag
{
  typedef sensor_msgs::Imu              ImuMsg;
  typedef sensor_msgs::MagneticField    MagMsg;

  public:

  ImuFilterRosbag(rosbag::Bag& bag, const std::string& output_imu_topic, const std::string& world_frame){

	  	bag_ = &bag;

	  	// Use time synchronizer to make sure we get properly synchronized images
	  	message_filters::TimeSynchronizer<sensor_msgs::Imu, sensor_msgs::MagneticField> sync(imu_sub_, mag_sub_, 25);
	  	sync.registerCallback(boost::bind(&ImuFilterRosbag::imuMagCallback, this, _1, _2));

		 if (world_frame == "ned") {
			world_frame_ = WorldFrame::NED;
		  } else if (world_frame == "nwu"){
			world_frame_ = WorldFrame::NWU;
		  } else if (world_frame == "enu"){
			world_frame_ = WorldFrame::ENU;
		  } else {
			//ROS_ERROR("The parameter world_frame was set to invalid value '%s'.", world_frame.c_str());
			//ROS_ERROR("Valid values are 'enu', 'ned' and 'nwu'. Setting to 'enu'.");
			world_frame_ = WorldFrame::ENU;
		  }
		  filter_.setWorldFrame(world_frame_);
    }

    virtual ~ImuFilterRosbag(){
    	// Nothing
    }

    void addImuMessage(sensor_msgs::Imu::ConstPtr imu){
		imu_sub_.newMessage(imu);
	}

	void addMagMessage(sensor_msgs::MagneticField::ConstPtr mag){
		mag_sub_.newMessage(mag);
	}


  private:

    rosbag::Bag* bag_;
    std::string output_imu_topic_;

	// Set up fake subscribers to capture Imu and MagneticField messages
	BagSubscriber<sensor_msgs::Imu> imu_sub_;
	BagSubscriber<sensor_msgs::MagneticField> mag_sub_;

    // **** paramaters
    WorldFrame::WorldFrame world_frame_;
    bool stateless_;
    std::string fixed_frame_;
    std::string imu_frame_;
    double constant_dt_;
    geometry_msgs::Vector3 mag_bias_;
    double orientation_variance_;

    // **** state variables
    bool initialized_;
    ros::Time last_time_;

    // **** filter implementation
    ImuFilter filter_;

    // **** member functions
    void imuMagCallback(const ImuMsg::ConstPtr& imu_msg_raw,
                        const MagMsg::ConstPtr& mag_msg){

	  const geometry_msgs::Vector3& ang_vel = imu_msg_raw->angular_velocity;
	  const geometry_msgs::Vector3& lin_acc = imu_msg_raw->linear_acceleration;
	  const geometry_msgs::Vector3& mag_fld = mag_msg->magnetic_field;

	  ros::Time time = imu_msg_raw->header.stamp;
	  imu_frame_ = imu_msg_raw->header.frame_id;

	  /*** Compensate for hard iron ***/
	  geometry_msgs::Vector3 mag_compensated;
	  mag_compensated.x = mag_fld.x - mag_bias_.x;
	  mag_compensated.y = mag_fld.y - mag_bias_.y;
	  mag_compensated.z = mag_fld.z - mag_bias_.z;

	  double roll = 0.0;
	  double pitch = 0.0;
	  double yaw = 0.0;

	  if (!initialized_ || stateless_)
	  {
		// wait for mag message without NaN / inf
		if(!std::isfinite(mag_fld.x) || !std::isfinite(mag_fld.y) || !std::isfinite(mag_fld.z))
		{
		  return;
		}

		geometry_msgs::Quaternion init_q;
		StatelessOrientation::computeOrientation(world_frame_, lin_acc, mag_compensated, init_q);
		filter_.setOrientation(init_q.w, init_q.x, init_q.y, init_q.z);

		last_time_ = time;
		initialized_ = true;
	  }

	  float dt = (time - last_time_).toSec();

	  last_time_ = time;

	  if (!stateless_)
		filter_.madgwickAHRSupdate(
		  ang_vel.x, ang_vel.y, ang_vel.z,
		  lin_acc.x, lin_acc.y, lin_acc.z,
		  mag_compensated.x, mag_compensated.y, mag_compensated.z,
		  dt);

	  publishFilteredMsg(imu_msg_raw);
    }

    void publishFilteredMsg(const ImuMsg::ConstPtr& imu_msg_raw){

    	  double q0,q1,q2,q3;
    	  filter_.getOrientation(q0,q1,q2,q3);

    	  // create and publish filtered IMU message
    	  boost::shared_ptr<ImuMsg> imu_msg =
    	    boost::make_shared<ImuMsg>(*imu_msg_raw);

    	  imu_msg->orientation.w = q0;
    	  imu_msg->orientation.x = q1;
    	  imu_msg->orientation.y = q2;
    	  imu_msg->orientation.z = q3;

    	  imu_msg->orientation_covariance[0] = orientation_variance_;
    	  imu_msg->orientation_covariance[1] = 0.0;
    	  imu_msg->orientation_covariance[2] = 0.0;
    	  imu_msg->orientation_covariance[3] = 0.0;
    	  imu_msg->orientation_covariance[4] = orientation_variance_;
    	  imu_msg->orientation_covariance[5] = 0.0;
    	  imu_msg->orientation_covariance[6] = 0.0;
    	  imu_msg->orientation_covariance[7] = 0.0;
    	  imu_msg->orientation_covariance[8] = orientation_variance_;

    	  // Write msg in rosbag here
    	  bag_->write(output_imu_topic_, imu_msg->header.stamp, imu_msg);
    }
};

int main(int argc, char **argv){

	std::string input_rosbag = "input.bag";
	std::string output_rosbag = "output.bag";
	std::string output_imu_topic = "/imu/data";
	std::string input_imu_topic = "/imu/data_raw";
	std::string input_mag_topic = "/imu/mag";
	std::string world_frame = "nwu";
	bool stateless = false;

	namespace po = boost::program_options;
	po::options_description desc("Allowed options");
	desc.add_options()
	("help,h", "describe arguments")
	("output,o", po::value(&output_rosbag), "set output rosbag file")
	("input,i", po::value(&input_rosbag), "set input rosbag file")
	("output-imu-topic,u", po::value(&output_imu_topic), "set topic of the output Imu messages")
	("input-imu-topic,u", po::value(&input_imu_topic), "set topic of the input Imu messages")
	("input-mag-topic,m", po::value(&input_mag_topic), "set topic of the input MagneticField messages")
	("world-frame,f", po::value(&world_frame), "set the world frame")
	("stateless,f", po::bool_switch(&stateless), "set topic of the input Imu messages");

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	if (vm.count("help")) {
		cout << desc << "\n";
		return 1;
	}

	rosbag::Bag output(output_rosbag, rosbag::bagmode::Write);
	rosbag::Bag input(input_rosbag, rosbag::bagmode::Read);

	ImuFilterRosbag filter(output, output_imu_topic, world_frame);

	rosbag::View view(input);
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	{
		// Detect Imu messages from the given topic
		if (m.getTopic() == input_imu_topic || ("/" + m.getTopic() == input_imu_topic))
		{
		  sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
		  if (imu != NULL)
			  filter.addImuMessage(imu);
		}

		// Detect MagneticField messages from the given topic
		if (m.getTopic() == input_mag_topic || ("/" + m.getTopic() == input_mag_topic))
		{
		  sensor_msgs::MagneticField::ConstPtr mag = m.instantiate<sensor_msgs::MagneticField>();
		  if (mag != NULL)
			  filter.addMagMessage(mag);
		}

		// Write every message to output bag
	    output.write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());
	}

	output.close();
	input.close();

	return 0;
}
