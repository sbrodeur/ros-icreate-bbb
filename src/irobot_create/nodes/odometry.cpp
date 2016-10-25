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

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <iostream>
#include <linux/input.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include "tf2_ros/transform_broadcaster.h"

#include "create/create.h"

using namespace std;

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, sensor_msgs::JointState> SyncPolicy;
typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;
typedef message_filters::Subscriber<sensor_msgs::Imu> ImuSubscriber;
typedef message_filters::Subscriber<sensor_msgs::JointState> JointStateSubscriber;

#define AXLE_LEN		0.258
#define WHEEL_DIAMETER	0.078

class OdometryNode {

    public:
        ros::NodeHandle node_;

        ros::Publisher odom_pub_;
        tf2_ros::TransformBroadcaster tf_broadcaster_;
		nav_msgs::Odometry odom_msg_;
		geometry_msgs::TransformStamped tf_odom_;

		boost::shared_ptr<Synchronizer> sync_;
		boost::shared_ptr<ImuSubscriber> imu_subscriber_;
		boost::shared_ptr<JointStateSubscriber> joint_subscriber_;

	    bool initialized_;
	    ros::Time last_time_;
	    bool publish_tf_;

		double x_;
		double y_;
		double lastLeftWheelDist_;
		double lastRightWheelDist_;

        std::string inputJoints_;
        std::string inputImu_;

        OdometryNode() : node_("~"), initialized_(false){

        	int queue_size;
        	node_.param("input_joints", inputJoints_, std::string("/irobot_create/joints"));
        	node_.param("input_imu", inputImu_, std::string("/imu/data"));
        	node_.param("queue_size", queue_size, 10);
        	node_.param("publish_tf", publish_tf_, false);

			// Set frame_id's
			const std::string str_base_baselink("base_link");
			tf_odom_.header.frame_id = "odom";
			tf_odom_.child_frame_id = str_base_baselink;
			odom_msg_.header.frame_id = "odom";
			odom_msg_.child_frame_id = str_base_baselink;

			imu_subscriber_.reset(new ImuSubscriber(node_, inputImu_, queue_size));
			joint_subscriber_.reset(new JointStateSubscriber(node_, inputJoints_, queue_size));

			sync_.reset(new Synchronizer(
			  SyncPolicy(queue_size), *imu_subscriber_, *joint_subscriber_));
			sync_->registerCallback(boost::bind(&OdometryNode::callback, this, _1, _2));

			odom_pub_ = node_.advertise<nav_msgs::Odometry>("/irobot_create/odom", 30);
        }

        virtual ~OdometryNode() {
        	// empty
        }

        void callback(const sensor_msgs::Imu::ConstPtr& imu_msg,
                	  const sensor_msgs::JointState::ConstPtr& joint_state_msg){

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
				tf_broadcaster_.sendTransform(tf_odom_);
			}

			odom_pub_.publish(odom_msg_);

			last_time_ = time;
			lastLeftWheelDist_ = leftWheelDist;
			lastRightWheelDist_ = rightWheelDist;
        }

        bool spin() {
        	ros::spin();
            return true;
        }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "odometry");

    OdometryNode a;
    a.spin();
    return 0;
}
