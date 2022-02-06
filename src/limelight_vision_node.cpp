#include "ros/ros.h"
#include "std_msgs/String.h"

#include "network_tables_node/NTGetDouble.h"
#include "network_tables_node/NTSetDouble.h"

#include "limelight_vision_node/Limelight_Info.h"
#include "limelight_vision_node/Limelight_Status.h"
#include "limelight_vision_node/Limelight.h"
#include "limelight_vision_node/Limelight_Control.h"

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <angles/angles.h>
#include <math.h>

#include <thread>
#include <string>
#include <mutex>
#include <vector>

#define STR_PARAM(s) #s
#define CKSP(s) ckgp( STR_PARAM(s) )
std::string ckgp(std::string instr)
{
	std::string retVal = ros::this_node::getName();
	retVal += "/" + instr;
	return retVal;
}


ros::NodeHandle* node;
ros::ServiceClient nt_getdouble_client;
ros::ServiceClient nt_setdouble_client;
std::mutex limelightMutex;
std::vector<std::string> limelight_names;
tf2_ros::TransformBroadcaster* tfBroadcaster;
tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener* tfListener;
ros::ServiceClient& getNTGetDoubleSrv()
{
	if (!nt_getdouble_client)
	{
		nt_getdouble_client = node->serviceClient<network_tables_node::NTGetDouble>("nt_getdouble", true);
	}
	return nt_getdouble_client;
}

ros::ServiceClient& getNTSetDoubleSrv()
{
	if (!nt_setdouble_client)
	{
		nt_setdouble_client = node->serviceClient<network_tables_node::NTSetDouble>("nt_setdouble", true);
	}
	return nt_setdouble_client;
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
			ros::ServiceClient& nt_getdouble_localclient = getNTGetDoubleSrv();
			if (nt_getdouble_localclient)
			{
				for (const std::string& s : limelight_names)
				{
					network_tables_node::NTGetDouble ntmsg;
					ntmsg.request.table_name = s;
					ntmsg.request.default_value = 0;

					limelight_vision_node::Limelight_Info limelightInfo;
					limelightInfo.name = s;
					
					ntmsg.request.entry_name = "tv";
					ntmsg.response.output = 0;
					nt_getdouble_localclient.call(ntmsg);
					limelightInfo.target_valid = ntmsg.response.output > 0 ? true : false;

					ntmsg.request.entry_name = "tx";
					ntmsg.response.output = 0;
					nt_getdouble_localclient.call(ntmsg);
					limelightInfo.target_dx_deg = -ntmsg.response.output;
					double tx = angles::from_degrees(-ntmsg.response.output);

					ntmsg.request.entry_name = "ty";
					ntmsg.response.output = 0;
					nt_getdouble_localclient.call(ntmsg);
					limelightInfo.target_dy_deg = -ntmsg.response.output;
					double ty = angles::from_degrees(-ntmsg.response.output);

					ntmsg.request.entry_name = "ta";
					ntmsg.response.output = 0;
					nt_getdouble_localclient.call(ntmsg);
					limelightInfo.target_area = ntmsg.response.output;

					ntmsg.request.entry_name = "ts";
					ntmsg.response.output = 0;
					nt_getdouble_localclient.call(ntmsg);
					limelightInfo.target_skew = ntmsg.response.output;

					ntmsg.request.entry_name = "tl";
					ntmsg.response.output = 0;
					nt_getdouble_localclient.call(ntmsg);
					limelightInfo.target_latency = ntmsg.response.output;

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
					}
					catch (tf2::TransformException &ex)
					{
						ROS_WARN("%s", ex.what());
					}
					//tf2Scalar distanceToHub = tf2::tf2Distance(tf2::Vector3(0, 0, 0), hubLinkStamped.getOrigin());
					
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
					//position.setX(hubLinkStamped.getOrigin().length());
					position.setX(1);
					
					position = position.rotate(z_axis, tx);
					position = position.rotate(y_axis, ty);
					(void)ty;

					tf2::Matrix3x3 qM(limelightToLimelightUnalignedStamped.getRotation());

					position = position * qM;

					double questionable = hubLinkStamped.getOrigin().getZ();
					double prevZ = position.getZ();
					double hmmm = position.getZ()/std::fabs(hubLinkStamped.getOrigin().getZ());
					position /= position.getZ()/std::fabs(hubLinkStamped.getOrigin().getZ());
					double posLength = position.length();

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

					ROS_INFO("Limelight tx: %lf, ty: %lf, x: %lf, y: %lf, z: %lf, length: %lf, ques: %lf, prev: %lf, poslen: %lf, hmmm: %lf", tx, ty, position.getX(), position.getY(), position.getZ(), hubLinkStamped.getOrigin().length(), questionable, prevZ, posLength, hmmm);
				}
			}
			limelight_pub.publish(limelightStatus);
		}

		rate.sleep();
	}
}

void limelightControlCallback(const limelight_vision_node::Limelight_Control& msg)
{
	ros::ServiceClient& nt_setdouble_localclient = getNTSetDoubleSrv();
	if (nt_setdouble_localclient)
	{
		for (const limelight_vision_node::Limelight& ll : msg.limelights)
		{
			bool setSuccess = true;

			network_tables_node::NTSetDouble ntmsg;
			ntmsg.request.table_name = ll.name;

			ntmsg.request.entry_name = "ledMode";
			ntmsg.request.value = ll.ledMode;
			setSuccess &= nt_setdouble_localclient.call(ntmsg);

			ntmsg.request.entry_name = "camMode";
			ntmsg.request.value = ll.camMode;
			setSuccess &= nt_setdouble_localclient.call(ntmsg);

			ntmsg.request.entry_name = "pipeline";
			ntmsg.request.value = ll.pipeline;
			setSuccess &= nt_setdouble_localclient.call(ntmsg);

			ntmsg.request.entry_name = "stream";
			ntmsg.request.value = ll.stream;
			setSuccess &= nt_setdouble_localclient.call(ntmsg);

			ntmsg.request.entry_name = "snapshot";
			ntmsg.request.value = ll.snapshot;
			setSuccess &= nt_setdouble_localclient.call(ntmsg);

			if (!setSuccess)
			{
				ROS_WARN("Failed to set values for limelight: %s", ll.name.c_str());
			}
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
	if (!required_params_found)
	{
		ROS_ERROR("Missing required parameters. Please check the list and make sure all required parameters are included");
		return 1;
	}

	getNTGetDoubleSrv();
	getNTSetDoubleSrv();

	std::thread limelightSendThread(publish_limelight_data);
	ros::Subscriber limelightControl = node->subscribe("/LimelightControl", 100, limelightControlCallback);

	ros::spin();

	limelightSendThread.join();

	return 0;
}