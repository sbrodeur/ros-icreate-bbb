/******************************************************************************
 *
 * Copyright (c) 2016, Simon Brodeur
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *  - Neither the name of the NECOTIS research group nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

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
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <tf2_msgs/TFMessage.h>

using namespace std;


#define AXLE_LEN		0.258
#define WHEEL_DIAMETER	0.078

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


class OdometryRosbag
{
  typedef sensor_msgs::Imu              ImuMsg;
  typedef sensor_msgs::JointState    	JointMsg;
  typedef BagSubscriber<ImuMsg> 		ImuSubscriber;
  typedef BagSubscriber<JointMsg> 		JointSubscriber;
  typedef message_filters::sync_policies::ApproximateTime<ImuMsg, JointMsg> SyncPolicy;
  typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

  public:

  OdometryRosbag(rosbag::Bag* bag, const std::string& output_odom_topic, const bool& publish_tf): initialized_(false){

	  	bag_ = bag;
	  	nb_msg_generated_ = 0;
	  	nb_tf_msg_generated_ = 0;
	  	output_odom_topic_ = output_odom_topic;
	  	publish_tf_ = publish_tf;

		// Set frame_id's
		const std::string str_base_baselink("base_link");
		tf_odom_.header.frame_id = "odom";
		tf_odom_.child_frame_id = str_base_baselink;
		odom_msg_.header.frame_id = "odom";
		odom_msg_.child_frame_id = str_base_baselink;

		// Set up fake subscribers to capture Imu and MagneticField messages
	  	imu_sub_.reset(new ImuSubscriber());
	  	joint_sub_.reset(new JointSubscriber());

	  	sync_.reset(new Synchronizer(
		  SyncPolicy(50), *imu_sub_, *joint_sub_));
		sync_->registerCallback(boost::bind(&OdometryRosbag::imuJointCallback, this, _1, _2));

    }

    virtual ~OdometryRosbag(){
    	// TODO: fix the following error at termination:
    	// terminate called after throwing an instance of 'boost::exception_detail::clone_impl<boost::exception_detail::error_info_injector<boost::lock_error> >'
    	//  what():  boost: mutex lock failed in pthread_mutex_lock: Invalid argument
    	// Aborted
    	//
    	// see: https://github.com/ros/ros_comm/issues/318
    	//      http://answers.ros.org/question/143756/rostimer-leads-to-boostlock_error-at-process-cleanup/
    	imu_sub_.reset();
    	joint_sub_.reset();
    	sync_.reset();
    }

    void addImuMessage(sensor_msgs::Imu::ConstPtr imu){
		imu_sub_->newMessage(imu);
	}

	void addJointMessage(sensor_msgs::JointState::ConstPtr joint){
		joint_sub_->newMessage(joint);
	}


  private:

    rosbag::Bag* bag_;
    std::string output_odom_topic_;

    nav_msgs::Odometry odom_msg_;
    geometry_msgs::TransformStamped tf_odom_;

    bool initialized_;
    ros::Time last_time_;
    bool publish_tf_;
    int nb_msg_generated_;
    int nb_tf_msg_generated_;

	double x_;
	double y_;
	double lastLeftWheelDist_;
	double lastRightWheelDist_;

	boost::shared_ptr<Synchronizer> sync_;
	boost::shared_ptr<ImuSubscriber> imu_sub_;
	boost::shared_ptr<JointSubscriber> joint_sub_;

    // **** member functions
    void imuJointCallback(const ImuMsg::ConstPtr& imu_msg,
                          const JointMsg::ConstPtr& joint_state_msg){

		ros::Time time = imu_msg->header.stamp;

		double wheelRadius = WHEEL_DIAMETER / 2.0;
		double leftWheelDist = joint_state_msg->position[0] * wheelRadius;
		double rightWheelDist = joint_state_msg->position[1] * wheelRadius;

		if (!initialized_){
			x_ = 0.0;
			y_ = 0.0;
			lastLeftWheelDist_ = leftWheelDist;
			lastRightWheelDist_ = rightWheelDist;
			last_time_ = time;
			initialized_ = true;
		}

		double deltaLeftWheelDist = leftWheelDist - lastLeftWheelDist_;
		double deltaRightWheelDist = rightWheelDist - lastRightWheelDist_;

		// NOTE: assume we are moving straight (true if dt is small enough)
		double roll, pitch, yaw;
		tf::Quaternion q;
		quaternionMsgToTF(imu_msg->orientation, q);
		tf::Matrix3x3 m(q);
		m.getRPY(roll, pitch, yaw);

		double deltaDist = (deltaLeftWheelDist + deltaRightWheelDist) / 2.0;
		x_ += deltaDist * cos(yaw);
		y_ += deltaDist * sin(yaw);

		// Populate position info
		// NOTE: propagate timestamp from the message
		odom_msg_.header.stamp = time;
		odom_msg_.pose.pose.position.x = x_;
		odom_msg_.pose.pose.position.y = y_;
		odom_msg_.pose.pose.orientation = imu_msg->orientation;

		// Populate velocity info (in the frame of the robot)
		double velocitiyForward = (joint_state_msg->velocity[0] + joint_state_msg->velocity[1]) * wheelRadius / 2.0;
		odom_msg_.twist.twist.linear.x = velocitiyForward;
		odom_msg_.twist.twist.linear.y = 0.0;
		odom_msg_.twist.twist.angular = imu_msg->angular_velocity;

		if (publish_tf_){
			// NOTE: propagate timestamp from the message
			tf_odom_.header.stamp = time;
			tf_odom_.transform.translation.x = x_;
			tf_odom_.transform.translation.y = y_;
			tf_odom_.transform.rotation = imu_msg->orientation;

			// Write tf message to rosbag
			//tf::tfMessage tf_msg;
			tf2_msgs::TFMessage tf_msg;
			tf_msg.transforms.push_back(tf_odom_);
			bag_->write("/tf", tf_odom_.header.stamp, tf_msg);
			nb_tf_msg_generated_++;

			if (nb_tf_msg_generated_ % 1000 == 0){
			  printf("Number of tf messages generated: %d \n", nb_tf_msg_generated_);
			}

		}

		// Write msg in rosbag here
		bag_->write(output_odom_topic_, odom_msg_.header.stamp, odom_msg_);
		nb_msg_generated_++;

		if (nb_msg_generated_ % 1000 == 0){
		  printf("Number of odometry messages generated: %d \n", nb_msg_generated_);
		}

		last_time_ = time;
		lastLeftWheelDist_ = leftWheelDist;
		lastRightWheelDist_ = rightWheelDist;
	}
};

int main(int argc, char **argv){

	ros::Time::init();

	std::string input_rosbag = "input.bag";
	std::string output_rosbag = "output.bag";
	std::string output_odom_topic = "/irobot_create/odom";
	std::string input_imu_topic = "/imu/data";
	std::string input_joint_topic = "/irobot_create/joints";
	bool publish_tf = false;

	namespace po = boost::program_options;
	po::options_description desc("Allowed options");
	desc.add_options()
	("help,h", "describe arguments")
	("output,o", po::value(&output_rosbag), "set output rosbag file")
	("input,i", po::value(&input_rosbag), "set input rosbag file")
	("output-odom-topic,d", po::value(&output_odom_topic), "set topic of the output Odometry messages")
	("input-imu-topic,m", po::value(&input_imu_topic), "set topic of the input Imu messages")
	("input-joint-topic,j", po::value(&input_joint_topic), "set topic of the input JointState messages")
	("publish-tf,t", po::bool_switch(&publish_tf), "set to publish tf messages");

	po::variables_map vm;
	po::store(po::parse_command_line(argc, argv, desc), vm);
	po::notify(vm);

	if (vm.count("help")) {
		cout << desc << "\n";
		return 1;
	}

	rosbag::Bag output(output_rosbag, rosbag::bagmode::Write);
	rosbag::Bag input(input_rosbag, rosbag::bagmode::Read);

	OdometryRosbag odometry(&output, output_odom_topic, publish_tf);

	int nb_imu_msg_processed = 0;
	int nb_joint_msg_processed = 0;
	int nb_total_msg_processed = 0;
	rosbag::View view(input);
	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	{
		// Detect Imu messages from the given topic
		if (m.getTopic() == input_imu_topic || ("/" + m.getTopic() == input_imu_topic))
		{
		  sensor_msgs::Imu::ConstPtr imu = m.instantiate<sensor_msgs::Imu>();
		  if (imu != NULL){
			  odometry.addImuMessage(imu);
			  nb_imu_msg_processed++;
		  }
		}

		// Detect JointState messages from the given topic
		if (m.getTopic() == input_joint_topic || ("/" + m.getTopic() == input_joint_topic))
		{
		  sensor_msgs::JointState::ConstPtr mag = m.instantiate<sensor_msgs::JointState>();
		  if (mag != NULL){
			  odometry.addJointMessage(mag);
			  nb_joint_msg_processed++;
		  }
		}

		if (m.getTopic() != output_odom_topic){
			// Write every message to output bag
			output.write(m.getTopic(), m.getTime(), m, m.getConnectionHeader());
			nb_total_msg_processed++;
		}

		if (nb_total_msg_processed % 1000 == 0){
			printf("Number of imu messages processed: %d (total %d)\n", nb_imu_msg_processed, nb_total_msg_processed);
			printf("Number of joint messages processed: %d (total %d)\n", nb_joint_msg_processed, nb_total_msg_processed);
		}
	}

	output.close();
	input.close();

	return 0;
}
