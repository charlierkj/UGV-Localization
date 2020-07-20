// header
#include "ugv_localization/gps_odom.hpp"

GPSOdom::GPSOdom(ros::NodeHandle& nh, 
				 std::string topic_gps, std::string topic_odom, 
				 double lat, double lon, double yaw, 
				 std::string frame_id, std::string child_frame_id,
				 double cov_threshold) : 
	nh_(nh), 
	topic_gps_(topic_gps), topic_odom_(topic_odom), 
	lat_(lat), lon_(lon), yaw_(yaw),
	frame_id_(frame_id), child_frame_id_(child_frame_id),
	cov_threshold_(cov_threshold)
{
	sub = nh_.subscribe(topic_gps_, 10, &GPSOdom::callback, this);
	pub = nh_.advertise<nav_msgs::Odometry>(topic_odom_, 10);

	double utm_x, utm_y;
	std::string utm_zone_tmp;
	RobotLocalization::NavsatConversions::LLtoUTM(lat_, lon_, utm_y, utm_x, utm_zone_tmp);
	
	// map to utm
	tf2::Transform tf_utm2map;
	tf_utm2map.setOrigin(tf2::Vector3(utm_x, utm_y, 0));
	tf2::Quaternion q;
	q.setRPY(0, 0, yaw_);
	tf_utm2map.setRotation(q); // yaw, pitch, roll
	tf_map2utm = tf_utm2map.inverse();
}

GPSOdom::~GPSOdom(){}

void GPSOdom::callback(const sensor_msgs::NavSatFix& msg_gps)
{
	
	if ((msg_gps.position_covariance[0] > cov_threshold_) 
    && (msg_gps.position_covariance[4] > cov_threshold_))
	{	return; }

	double lat = msg_gps.latitude;
	double lon = msg_gps.longitude;

	double utm_x, utm_y;
	std::string utm_zone_tmp;
	RobotLocalization::NavsatConversions::LLtoUTM(lat, lon, utm_y, utm_x, utm_zone_tmp);
	
	// utm to base_link
	tf2::Transform tf_utm2base_link;
	tf_utm2base_link.setOrigin(tf2::Vector3(utm_x, utm_y, 0));
	tf2::Quaternion q;
	q.setRPY(0, 0, 0);
	tf_utm2base_link.setRotation(q);
	
	// map to base_link
	tf2::Transform tf_map2base_link;
	tf_map2base_link = tf_map2utm * tf_utm2base_link;
	
	// coordinates in map frame
	double map_x, map_y;
	tf2::Vector3 trans;
	trans = tf_map2base_link.getOrigin();
	map_x = trans.getX();
	map_y = trans.getY();

	// publish
	nav_msgs::Odometry msg_odom;
	msg_odom.header.seq = msg_gps.header.seq;
	msg_odom.header.stamp = msg_gps.header.stamp;
	msg_odom.header.frame_id = frame_id_;
	msg_odom.child_frame_id = child_frame_id_;
	
	msg_odom.pose.pose.position.x = map_x;
	msg_odom.pose.pose.position.y = map_y;
	msg_odom.pose.pose.position.z = 0;
	msg_odom.pose.pose.orientation.x = 0;
	msg_odom.pose.pose.orientation.y = 0;
	msg_odom.pose.pose.orientation.z = 0;
	msg_odom.pose.pose.orientation.w = 1;

	msg_odom.pose.covariance[0] = msg_gps.position_covariance[4];
	msg_odom.pose.covariance[7] = msg_gps.position_covariance[0];
	msg_odom.pose.covariance[14] = msg_gps.position_covariance[8];
	
	pub.publish(msg_odom);
}


	
