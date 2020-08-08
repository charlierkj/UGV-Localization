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

		double last_time_;
		double cov_thresh_;

	public:
		GPSNavHeadingSubPub(ros::NodeHandle& nh, std::string topic_out, double cov_thresh) : nh_(nh), topic_out_(topic_out), cov_thresh_(cov_thresh) {
			sub_ = nh_.subscribe("gps/navrelposned", 10, &GPSNavHeadingSubPub::callbackNavRelPosNed, this);
			pub_ = nh_.advertise<sensor_msgs::Imu>(topic_out_, 10);

			last_time_ = -1;
		}

		~GPSNavHeadingSubPub() {}

		void callbackNavRelPosNed(const ublox_msgs::NavRELPOSNED9 &m)
		{
			double time_scale, curr_time;
			curr_time = ros::Time::now().toSec();
			if (last_time_ == -1)
			{	time_scale = 1;	}
			else
			{
	  			double dt;
	 			dt = curr_time - last_time_;
				time_scale = sqrt(dt / 0.2);
	 		}

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
				//imu_.orientation_covariance[8] = 100 * cov;
				imu_.orientation_covariance[8] = 100 * time_scale * cov;
				//std::cout << cov1 << "        " << cov2 << "        " << cov3 << std::endl; 
    		}

			if (imu_.orientation_covariance[8] < cov_thresh_)
			{
    			pub_.publish(imu_);
				last_time_ = curr_time;
			}
		}
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "gps_navheading_node");
	ros::NodeHandle nh;

	std::string node_name = ros::this_node::getName();

	std::string topic_out;
	double cov_thresh;

    nh.getParam(node_name+"/topic_out", topic_out);
	nh.getParam(node_name+"/cov_thresh", cov_thresh);

	GPSNavHeadingSubPub gps_navheading_subpub(nh, topic_out, cov_thresh);

	ros::spin();
	return 0;
}
	

