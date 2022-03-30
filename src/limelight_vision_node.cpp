#include "ros/ros.h"
#include "std_msgs/String.h"

#include "limelight_vision_node/Limelight_Info.h"
#include "limelight_vision_node/Limelight_Status.h"
#include "limelight_vision_node/Limelight.h"
#include "limelight_vision_node/Limelight_Control.h"

#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <rio_control_node/Robot_Status.h>
#include <geometry_msgs/TransformStamped.h>
#include <angles/angles.h>
#include <math.h>

#include "ck_utilities/MovingAverage.hpp"
#include "ck_utilities/NTHelper.hpp"

#include <thread>
#include <string>
#include <mutex>
#include <vector>

#define STR_PARAM(s) #s
#define CKSP(s) ckgp( STR_PARAM(s) )

#define ENABLE_POSITION_PUBLISHING

// #define ODOMETRY_ONLY_AUTO

std::string ckgp(std::string instr)
{
	std::string retVal = ros::this_node::getName();
	retVal += "/" + instr;
	return retVal;
}


ros::NodeHandle* node;
std::mutex limelightMutex;
std::vector<std::string> limelight_names;
tf2_ros::TransformBroadcaster* tfBroadcaster;
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener* tfListener;

enum class RobotState
{
	DISABLED,
	AUTONOMOUS,
	TELEOP,
	TEST,
};

static RobotState robot_state = RobotState::DISABLED;

void robot_status_callback(const rio_control_node::Robot_Status &msg)
{
	if (msg.robot_state == rio_control_node::Robot_Status::AUTONOMOUS)
	{
		robot_state = RobotState::AUTONOMOUS;
	}
	else if (msg.robot_state == rio_control_node::Robot_Status::DISABLED)
	{
		robot_state = RobotState::DISABLED;
	}
	else if (msg.robot_state == rio_control_node::Robot_Status::TEST)
	{
		robot_state = RobotState::TEST;
	}
	else if (msg.robot_state == rio_control_node::Robot_Status::TELEOP)
	{
		robot_state = RobotState::TELEOP;
	}
}

static bool enable_publish_position_data = false;

inline bool time_not_timed_out(ros::Time& checkedTime, const double& timeout)
{
	return ((ros::Time::now() - checkedTime) < ros::Duration(timeout));
}

void publish_limelight_data()
{
	static ros::Publisher limelight_pub = node->advertise<limelight_vision_node::Limelight_Status>("LimelightStatus", 1);
	static limelight_vision_node::Limelight_Status limelightStatus;
	//Transform to limelight from baselink
	//Transform from limelight to target
	//Take x from hub link transform
	//Statically publish limelight tf2
	//transformStamped.header.frame_id = "hub_link"; //Existing frame of reference
	//transformStamped.child_frame_id = "hub_full_height";	//New frame

	ros::Rate rate(100);

	while (ros::ok())
	{
		{
			std::lock_guard<std::mutex> lock(limelightMutex);
			limelightStatus.limelights.clear();
			for (const std::string& s : limelight_names)
			{
				ros::Time response_time(0);
				limelight_vision_node::Limelight_Info limelightInfo;
				limelightInfo.name = s;

				double default_val = 0;
				bool limelight_data_valid = false;
				const double timeout = 0.2;
				
				double tv = 0;
				ck::nt::get(tv, response_time, s, "tv", default_val);
				limelightInfo.target_valid = tv > 0 ? true : false;
				limelight_data_valid |= time_not_timed_out(response_time, timeout);

				ck::nt::get(limelightInfo.target_dx_deg, response_time, s, "tx", default_val);
				limelightInfo.target_dx_deg = -limelightInfo.target_dx_deg;
				double temp_tx = angles::from_degrees(limelightInfo.target_dx_deg);
				limelight_data_valid |= time_not_timed_out(response_time, timeout);

				ck::nt::get(limelightInfo.target_dy_deg, response_time, s, "ty", default_val);
				limelightInfo.target_dy_deg = -limelightInfo.target_dy_deg;
				double temp_ty = angles::from_degrees(limelightInfo.target_dy_deg);
				limelight_data_valid |= time_not_timed_out(response_time, timeout);

				double tx = temp_ty;
				double ty = -temp_tx;


				static ck::MovingAverage txAverage(10);
				static ck::MovingAverage tyAverage(10);

				txAverage.addSample(tx);
				tx = txAverage.getAverage();

				tyAverage.addSample(ty);
				ty = tyAverage.getAverage();


				ck::nt::get(limelightInfo.target_area, response_time, s, "ta", default_val);
				limelight_data_valid |= time_not_timed_out(response_time, timeout);

				ck::nt::get(limelightInfo.target_skew, response_time, s, "ts", default_val);
				limelight_data_valid |= time_not_timed_out(response_time, timeout);

				ck::nt::get(limelightInfo.target_latency, response_time, s, "tl", default_val);
				limelight_data_valid |= time_not_timed_out(response_time, timeout);

				limelightInfo.target_valid &= limelight_data_valid;
				limelightStatus.limelights.push_back(limelightInfo);


				//Publish tf2 transform for limelight to hub
				std::string limelightFrameName = s + "_link";
				std::string limelightUnalignedFrameName = s + "_unaligned";
				tf2::Stamped<tf2::Transform> hubLinkStamped;
				tf2::Stamped<tf2::Transform> limelightToLimelightUnalignedStamped;
				
				try
				{
					tf2::convert(tfBuffer.lookupTransform(limelightFrameName, limelightUnalignedFrameName, ros::Time(0)), limelightToLimelightUnalignedStamped);
					tf2::convert(tfBuffer.lookupTransform("hub_full_height", limelightUnalignedFrameName, ros::Time(0)), hubLinkStamped);

					geometry_msgs::TransformStamped transformStamped;

					transformStamped.header.stamp = ros::Time::now();
					transformStamped.header.frame_id = limelightUnalignedFrameName;
					transformStamped.child_frame_id = limelightFrameName + "_hub";

					tf2::Vector3 x_axis = {1, 0, 0};
					tf2::Vector3 y_axis = {0, 1, 0};
					tf2::Vector3 z_axis = {0, 0, 1};
					(void)x_axis;
					(void)y_axis;
					(void)z_axis;

					tf2::Vector3 position(0, 0, 0);
					position.setX(hubLinkStamped.getOrigin().length() + (24 * 0.0254));
					
					position = position.rotate(z_axis, tx);
					position = position.rotate(y_axis, ty);
					(void)ty;

					tf2::Matrix3x3 qM(limelightToLimelightUnalignedStamped.getRotation());

					position = position * qM;

					transformStamped.transform.translation.x = position.getX();
					transformStamped.transform.translation.y = position.getY();
					transformStamped.transform.translation.z = position.getZ();

					tf2::Quaternion q;
					q.setRPY(0, 0, 0);
					transformStamped.transform.rotation.x = q.x();
					transformStamped.transform.rotation.y = q.y();
					transformStamped.transform.rotation.z = q.z();
					transformStamped.transform.rotation.w = q.w();

					tfBroadcaster->sendTransform(transformStamped);

					if (enable_publish_position_data && limelightInfo.target_valid)
					{
						try
						{
							tf2::Stamped<tf2::Transform> hub_to_limelight_unaligned_through_robot_transform;
							tf2::convert(tfBuffer.lookupTransform("hub_full_height", limelightUnalignedFrameName, ros::Time(0)), hub_to_limelight_unaligned_through_robot_transform);

							nav_msgs::Odometry odometry_data;
							odometry_data.header.stamp = ros::Time::now();
							odometry_data.header.frame_id = "hub_full_height";
							odometry_data.child_frame_id = "base_link";

							tf2::Vector3 hub_to_limelight_unaligned_translation;
							hub_to_limelight_unaligned_translation.setX(-transformStamped.transform.translation.x);
							hub_to_limelight_unaligned_translation.setY(-transformStamped.transform.translation.y);
							hub_to_limelight_unaligned_translation.setZ(-transformStamped.transform.translation.z);

							tf2::Matrix3x3 rotation_matrix(hub_to_limelight_unaligned_through_robot_transform.inverse().getRotation());

							tf2::Vector3 result_translation =  hub_to_limelight_unaligned_translation * rotation_matrix;

							odometry_data.pose.pose.position.x = result_translation.getX();
							odometry_data.pose.pose.position.y = result_translation.getY();
							odometry_data.pose.pose.position.z = 0;

							odometry_data.twist.twist.linear.x = 0;
							odometry_data.twist.twist.linear.y = 0;
							odometry_data.twist.twist.linear.z = 0;

							odometry_data.twist.twist.angular.x = 0;
							odometry_data.twist.twist.angular.y = 0;
							odometry_data.twist.twist.angular.z = 0;

							odometry_data.pose.covariance =
							{   0.2, 0.0, 0.0, 0.0, 0.0, 0.0,
								0.0, 0.2, 0.0, 0.0, 0.0, 0.0,
								0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
								0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
								0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
								0.0, 0.0, 0.0, 0.0, 0.0, 0.0,};

							odometry_data.twist.covariance =
							{   0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
								0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
								0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
								0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
								0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
								0.0, 0.0, 0.0, 0.0, 0.0, 0.0,};

							static ros::Publisher odometry_publisher = node->advertise<nav_msgs::Odometry>("/LimelightOdometry", 1);
#ifdef ODOMETRY_ONLY_AUTO
							if (robot_state == RobotState::TELEOP) {
#endif
							odometry_publisher.publish(odometry_data);
#ifdef ODOMETRY_ONLY_AUTO
							}
#endif
						}
						catch ( ... )
						{
							static int32_t warn_limiter = 1;
							warn_limiter ++;
							warn_limiter = warn_limiter % 500;
							if (warn_limiter == 0)
							{
								ROS_WARN("Can't lookup limelight to baselink transform");
							}
						}
					}
				}
				catch (tf2::TransformException &ex)
				{
					static int32_t warn_limiter = 1;
					warn_limiter++;
					warn_limiter = warn_limiter % 500;
					if(warn_limiter == 0)
					{
						ROS_WARN("Warning - hub full height or limelight alignment frame not published yet");
					}
				}
			}
			limelight_pub.publish(limelightStatus);
		}

		rate.sleep();
	}
}

void limelightControlCallback(const limelight_vision_node::Limelight_Control& msg)
{
	for (const limelight_vision_node::Limelight& ll : msg.limelights)
	{
		bool setSuccess = true;
		setSuccess &= ck::nt::set(ll.name, "ledMode", ll.ledMode);
		setSuccess &= ck::nt::set(ll.name, "camMode", ll.camMode);
		setSuccess &= ck::nt::set(ll.name, "pipeline", ll.pipeline);
		setSuccess &= ck::nt::set(ll.name, "stream", ll.stream);
		setSuccess &= ck::nt::set(ll.name, "snapshot", ll.snapshot);

		if (!setSuccess)
		{
			ROS_WARN("Failed to set values for limelight: %s", ll.name.c_str());
		}
	}
}

int main(int argc, char **argv)
{
	/**
	 * The ros::init() function needs to see argc and argv so that it can perform
	 * any ROS arguments and name remapping that were provided at the command line.
	 * For programmatic remappings you can use a different version of init() which takes
	 * remappings directly, but for most command-line programs, passing argc and argv is
	 * the easiest way to do it.  The third argument to init() is the name of the node.
	 *
	 * You must call one of the versions of ros::init() before using any other
	 * part of the ROS system.
	 */
	ros::init(argc, argv, "limelight_vision_node");

	ros::NodeHandle n;
	node = &n;
	tfBroadcaster = new tf2_ros::TransformBroadcaster();
	tfListener = new tf2_ros::TransformListener(tfBuffer);

	bool required_params_found = true;
	required_params_found &= n.getParam(CKSP(limelight_names), limelight_names);
	required_params_found &= n.getParam(CKSP(enable_publish_position_data), enable_publish_position_data);
	if (!required_params_found)
	{
		ROS_ERROR("Missing required parameters for node %s. Please check the list and make sure all required parameters are included", ros::this_node::getName().c_str());
		return 1;
	}

	std::thread limelightSendThread(publish_limelight_data);
	ros::Subscriber limelightControl = node->subscribe("/LimelightControl", 100, limelightControlCallback);
	ros::Subscriber robot_status_subscriber = node->subscribe("/RobotStatus", 100, robot_status_callback);

	ros::spin();

	limelightSendThread.join();

	return 0;
}