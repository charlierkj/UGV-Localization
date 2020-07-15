// ROS
#include "ros/ros.h"

// message
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"

// robot_localization
#include "robot_localization/navsat_conversions.h"


class GPSOdom
{
	private:
		ros::NodeHandle nh;
		ros::Subscriber sub;
		ros::Publisher pub;

		std::string topic_gps_, topic_odom_; // topic to subscribe, and to publish
		double lat_, lon_, yaw_; // origin of map frame
		std::string frame_id_, child_frame_id_;

		tf2::Transform tf_map2utm;

	public:
		GPSOdom(ros::NodeHandle& nh, 
				std::string topic_gps, std::string topic_odom, 
				double lat, double lon, double yaw, 
				std::string frame_id, std::string child_frame_id);
		~GPSOdom();
		void callback(const sensor_msgs::NavSatFix& msg_gps);