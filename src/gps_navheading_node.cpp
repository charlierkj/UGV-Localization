#include "ros/ros.h"
#include "tf/transform_broadcaster.h"

// message
#include "sensor_msgs/Imu.h"
#include "ublox_msgs/NavRELPOSNED9.h"

class GPSNavHeadingSubPub
{
	private:
		ros::NodeHandle nh_;
		ros::Subscriber sub_;
		ros::Publisher pub_;

		std::string topic_out_;

	public:
		GPSNavHeadingSubPub(ros::NodeHandle& nh, std::string topic_out) : nh_(nh), topic_out_(topic_out) {
			sub_ = nh_.subscribe("gps/navrelposned", 10, &GPSNavHeadingSubPub::callbackNavRelPosNed, this);
			pub_ = nh_.advertise<sensor_msgs::Imu>("gps/navheading/new", 10);
		}

		~GPSNavHeadingSubPub() {}

		void callbackNavRelPosNed(const ublox_msgs::NavRELPOSNED9 &m)
		{
			sensor_msgs::Imu imu_;
    		imu_.header.stamp = ros::Time::now();
    		imu_.header.frame_id = "gps";
    		imu_.linear_acceleration_covariance[0] = -1;
    		imu_.angular_velocity_covariance[0] = -1;
    		// Transform angle since ublox is representing heading as NED but ROS uses ENU as convention (REP-103).
    		// Also convert the base-to-rover angle to a robot-to-base angle (consistent with frame_id)
    		double heading = - (static_cast<double>(m.relPosHeading) * 1e-5 / 180.0 * M_PI) - M_PI_2;
    		tf::Quaternion orientation;
    		orientation.setRPY(0, 0, heading);
    		imu_.orientation.x = orientation[0];
    		imu_.orientation.y = orientation[1];
    		imu_.orientation.z = orientation[2];
    		imu_.orientation.w = orientation[3];
    		imu_.orientation_covariance[0] = 1000.0;
    		imu_.orientation_covariance[4] = 1000.0;
    		imu_.orientation_covariance[8] = 1000.0;
    		// When heading is reported to be valid, use accuracy reported in 1e-5 deg units
    		if (m.flags & ublox_msgs::NavRELPOSNED9::FLAGS_REL_POS_HEAD_VALID)
    		{
				double cov1, cov2, cov3;
      			cov1 = pow(m.accHeading * 1e-5 / 180.0 * M_PI, 2);
				cov2 = pow(m.accLength * 1e-4, 2);
				cov3 = pow((static_cast<double>(m.relPosLength) + 0.01 * static_cast<double>(m.relPosHPLength) - 70) * 1e-2, 2);

				double cov = std::max(std::max(cov1, cov2), cov3);
				imu_.orientation_covariance[8] = 100 * cov;
				//std::cout << cov1 << "        " << cov2 << "        " << cov3 << std::endl; 
    		}
    		pub_.publish(imu_);
		}
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "gps_navheading_node");
	ros::NodeHandle nh;

	std::string topic_out;
	topic_out = "gps/navheading/new";

	GPSNavHeadingSubPub gps_navheading_subpub(nh, topic_out);

	ros::spin();
	return 0;
}
	

