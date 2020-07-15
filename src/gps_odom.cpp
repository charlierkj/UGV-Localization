// header
#include "ugv_localization/gps_odom.hpp"

GPSOdom::GPSOdom(ros::NodeHandle& nh, 
				 std::string topic_gps, std::string topic_odom, 
				 double lat, double lon, double yaw, 
				 std::string frame_id, std::string child_frame_id) : 
	nh_(nh), 
	topic_gps_(topic_gps), topic_odom_(topic_odom), 
	lat_(lat), lon_(lon), yaw_(yaw),
	frame_id_(frame_id), child_frame_id_(child_frame_id)
	{
		sub = nh_.subscribe(topic_gps_, 10, &GPSOdom::callback, this);
		pub = nh_.advertise<nav_msgs::Odometry>(topic_odom_, 10);

		double utm_x, utm_y;
		std::string utm_zone_tmp;
		RobotLocalization::NavsatConversions::LLtoUTM(lat_, lon_, utm_y, utm_x, utm_zone_tmp);
		
		tf2::Transform tf_utm2map;
		tf_utm2map.setOrigin(tf2::Vector3(utm_x, utm_y, 0.0));
		tf_utm2map.setRotation(tf2::Quaternion(yaw_, 0.0, 0.0)); // yaw, pitch, roll
		tf_map2utm = tf_utm2map.inverse();
