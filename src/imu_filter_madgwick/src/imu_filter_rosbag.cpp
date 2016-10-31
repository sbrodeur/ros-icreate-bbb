#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <iostream>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>

#include "imu_filter_madgwick/imu_filter.h"
#include "imu_filter_madgwick/stateless_orientation.h"
#include "imu_filter_madgwick/ImuFilterMadgwickConfig.h"
#include "geometry_msgs/TransformStamped.h"
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2_msgs/TFMessage.h>

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
	  message_filters::SimpleFilter<M>::signalMessage(msg);
  }
};


class ImuFilterRosbag
{
  typedef sensor_msgs::Imu              ImuMsg;
  typedef sensor_msgs::MagneticField    MagMsg;
  typedef BagSubscriber<ImuMsg> 		ImuSubscriber;
  typedef BagSubscriber<MagMsg> 		MagSubscriber;
  typedef message_filters::sync_policies::ApproximateTime<ImuMsg, MagMsg> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
  typedef imu_filter_madgwick::ImuFilterMadgwickConfig   FilterConfig;

  public:

  ImuFilterRosbag(rosbag::Bag* bag, const std::string& output_imu_topic, const std::string& world_frame,
		  	      const bool& stateless = false, const bool& publish_tf = false, const bool& reverse_tf = false,
				  const std::string& imu_frame = "imu_link", const std::string& fixed_frame = "base_link"){

	  	bag_ = bag;
	  	nb_msg_generated_ = 0;
	  	nb_tf_msg_generated_ = 0;
	  	stateless_ = stateless;
	  	output_imu_topic_ = output_imu_topic;
	  	publish_tf_ = publish_tf;
	  	reverse_tf_ = reverse_tf;
	  	imu_frame_ = imu_frame;
	  	fixed_frame_ = fixed_frame;

		// Set up fake subscribers to capture Imu and MagneticField messages
	  	imu_sub_.reset(new ImuSubscriber());
	  	mag_sub_.reset(new MagSubscriber());

	  	sync_.reset(new Synchronizer(
		  SyncPolicy(50), *imu_sub_, *mag_sub_));
		sync_->registerCallback(boost::bind(&ImuFilterRosbag::imuMagCallback, this, _1, _2));

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

      filter_.setAlgorithmGain(0.1);
    }

    virtual ~ImuFilterRosbag(){
    	// TODO: fix the following error at termination:
    	// terminate called after throwing an instance of 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
    	//  what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
    	// Aborted
    	//
    	// see: https://github.com/ros/ros_comm/issues/318
    	//      http://answers.ros.org/question/143756/rostimer-leads-to-boostlock_error-at-process-cleanup/
    	imu_sub_.reset();
    	mag_sub_.reset();
    	sync_.reset();
    }

    void addImuMessage(sensor_msgs::Imu::ConstPtr imu){
		imu_sub_->newMessage(imu);
	}

	void addMagMessage(sensor_msgs::MagneticField::ConstPtr mag){
		mag_sub_->newMessage(mag);
	}


  private:

    rosbag::Bag* bag_;
    std::string output_imu_topic_;

	boost::shared_ptr<Synchronizer> sync_;
	boost::shared_ptr<ImuSubscriber> imu_sub_;
	boost::shared_ptr<MagSubscriber> mag_sub_;

    // **** paramaters
    WorldFrame::WorldFrame world_frame_;
    bool stateless_;
    geometry_msgs::Vector3 mag_bias_;
    double orientation_variance_;

    // **** state variables
    bool initialized_;
    ros::Time last_time_;
    int nb_msg_generated_;
    int nb_tf_msg_generated_;
    bool publish_tf_;
    bool reverse_tf_;
    std::string imu_frame_;
    std::string fixed_frame_;

    // **** filter implementation
    ImuFilter filter_;

    void reconfigure(FilterConfig& config){
      double gain, zeta;
      gain = config.gain;
      zeta = config.zeta;
      filter_.setAlgorithmGain(gain);
      filter_.setDriftBiasGain(zeta);
      printf("Imu filter gain set to %f", gain);
      printf("Gyro drift bias set to %f", zeta);
      mag_bias_.x = config.mag_bias_x;
      mag_bias_.y = config.mag_bias_y;
      mag_bias_.z = config.mag_bias_z;
      orientation_variance_ = config.orientation_stddev * config.orientation_stddev;
      printf("Magnetometer bias values: %f %f %f", mag_bias_.x, mag_bias_.y, mag_bias_.z);
    }

    // **** member functions
    void imuMagCallback(const ImuMsg::ConstPtr& imu_msg_raw,
                        const MagMsg::ConstPtr& mag_msg){

	  const geometry_msgs::Vector3& ang_vel = imu_msg_raw->angular_velocity;
	  const geometry_msgs::Vector3& lin_acc = imu_msg_raw->linear_acceleration;
	  const geometry_msgs::Vector3& mag_fld = mag_msg->magnetic_field;

	  ros::Time time = imu_msg_raw->header.stamp;

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
	  if (publish_tf_)
	      publishTransform(imu_msg_raw);
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
    	  nb_msg_generated_++;

    	  if (nb_msg_generated_ % 1000 == 0){
    		  printf("Number of filtered imu messages generated: %d \n", nb_msg_generated_);
    	  }
    }

    void publishTransform(const ImuMsg::ConstPtr& imu_msg_raw){

      double q0,q1,q2,q3;
      filter_.getOrientation(q0,q1,q2,q3);
      geometry_msgs::TransformStamped transform;
      transform.header.stamp = imu_msg_raw->header.stamp;
      if (reverse_tf_)
      {
        transform.header.frame_id = imu_frame_;
        transform.child_frame_id = fixed_frame_;
        transform.transform.rotation.w = q0;
        transform.transform.rotation.x = -q1;
        transform.transform.rotation.y = -q2;
        transform.transform.rotation.z = -q3;
      }
      else {
        transform.header.frame_id = fixed_frame_;
        transform.child_frame_id = imu_frame_;
        transform.transform.rotation.w = q0;
        transform.transform.rotation.x = q1;
        transform.transform.rotation.y = q2;
        transform.transform.rotation.z = q3;
      }

		// Write tf message to rosbag
		//tf::tfMessage tf_msg;
        tf2_msgs::TFMessage tf_msg;
      	tf_msg.transforms.push_back(transform);
		bag_->write("/tf", transform.header.stamp, tf_msg);
		nb_tf_msg_generated_++;

		if (nb_tf_msg_generated_ % 1000 == 0){
			  printf("Number of tf messages generated: %d \n", nb_tf_msg_generated_);
		}
    }

};

int main(int argc, char **argv){

	ros::Time::init();

	std::string input_rosbag = "input.bag";
	std::string output_rosbag = "output.bag";
	std::string output_imu_topic = "/imu/data";
	std::string input_imu_topic = "/imu/data_raw";
	std::string input_mag_topic = "/imu/mag";
	std::string world_frame = "nwu";
	std::string imu_frame = "imu_link";
	std::string fixed_frame = "base_link";
	bool stateless = false;
	bool publish_tf = false;
	bool reverse_tf = false;

	namespace po = boost::program_options;
	po::options_description desc("Allowed options");
	desc.add_options()
	("help,h", "describe arguments")
	("output,o", po::value(&output_rosbag), "set output rosbag file")
	("input,i", po::value(&input_rosbag), "set input rosbag file")
	("output-imu-topic,d", po::value(&output_imu_topic), "set topic of the output Imu messages")
	("input-imu-topic,m", po::value(&input_imu_topic), "set topic of the input Imu messages")
	("input-mag-topic,g", po::value(&input_mag_topic), "set topic of the input MagneticField messages")
	("world-frame,w", po::value(&world_frame), "set the world frame")
	("stateless,s", po::bool_switch(&stateless), "set topic of the input Imu messages")
	("publish-tf,t", po::bool_switch(&publish_tf), "set to publish tf messages")
	("reverse-tf,r", po::bool_switch(&reverse_tf), "set to reverse tf messages")
	("imu-frame,f", po::value(&imu_frame), "set the name of the imu frame")
	("fixed-frame,x", po::value(&fixed_frame), "set the name of the fixed frame");

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	if (vm.count("help")) {
		cout << desc << "\n";
		return 1;
	}

	rosbag::Bag output(output_rosbag, rosbag::bagmode::Write);
	rosbag::Bag input(input_rosbag, rosbag::bagmode::Read);

	ImuFilterRosbag filter(&output, output_imu_topic, world_frame, stateless, publish_tf, reverse_tf, imu_frame, fixed_frame);

	int nb_imu_msg_processed = 0;
	int nb_mag_msg_processed = 0;
	int nb_total_msg_processed = 0;
	rosbag::View view(input);
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	{
		// Detect Imu messages from the given topic
		if (m.getTopic() == input_imu_topic || ("/" + m.getTopic() == input_imu_topic))
		{
		  sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
		  if (imu != NULL){
			  filter.addImuMessage(imu);
			  nb_imu_msg_processed++;
		  }
		}

		// Detect MagneticField messages from the given topic
		if (m.getTopic() == input_mag_topic || ("/" + m.getTopic() == input_mag_topic))
		{
		  sensor_msgs::MagneticField::ConstPtr mag = m.instantiate<sensor_msgs::MagneticField>();
		  if (mag != NULL){
			  filter.addMagMessage(mag);
			  nb_mag_msg_processed++;
		  }
		}

		if (m.getTopic() != output_imu_topic){
			// Write every message to output bag
			output.write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());
			nb_total_msg_processed++;
		}

		if (nb_total_msg_processed % 1000 == 0){
			printf("Number of imu messages processed: %d (total %d)\n", nb_imu_msg_processed, nb_total_msg_processed);
			printf("Number of mag messages processed: %d (total %d)\n", nb_mag_msg_processed, nb_total_msg_processed);
		}
	}

	output.close();
	input.close();

	return 0;
}
