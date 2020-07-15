#!/usr/bin/env python
import rospy
import yaml
from sensor_msgs.msg import NavSatFix
def gps_publisher():
    pub = rospy.Publisher('/world/fix', NavSatFix, queue_size=1)
    rospy.init_node('world_frame_gps_publisher', anonymous=True)
    #yaml_path = rospy.get_param('~strfile_params')
    #yaml_stream = file(yaml_path, 'r')
    #yaml_node = yaml.load(yaml_stream)
    #yaml_stream.close()
    rate = rospy.Rate(10) # 10hz
    msg = NavSatFix()
    msg.header.seq = 0
    msg.header.frame_id = "world"
    msg.status.status = 1
    msg.latitude = 39.326465
    msg.longitude = -76.621554
    while not rospy.is_shutdown():
        msg.header.seq = msg.header.seq+1
        msg.header.stamp = rospy.Time.now()
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        gps_publisher()
    except rospy.ROSInterruptException:
        pass
