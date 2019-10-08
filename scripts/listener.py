import rospy
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2 
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Imu
from sensor_msgs.msg import NavSatFix

import numpy as np

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.data)

def pointcloud2_to_array(cloud_msg):
    '''
    Point Cloud 2 type, creates a np array
    '''
    dat = pc2.read_points(cloud_msg, field_names = ("x", "y", "z"), skip_nans=True)
    arr = list(dat)
    nparray = np.array(arr)
    return 

def imu_listener(imu_msg):
    '''
    Imu type message
    '''
    # print imu_msg.orientation.x
    # print imu_msg.angular_velocity.x
    # print imu_msg.linear_acceleration_x
    return

def gps_listener(gps):
    '''
    gps listener for NavSatFix message
    '''
    # print gps.latitude

def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=False)

    rospy.Subscriber('/camera/depth/points',PointCloud2 ,pointcloud2_to_array)
    rospy.Subscriber('/test/imu',Imu ,imu_listener)
    rospy.Subscriber('/fix/',NavSatFix,gps_listener)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    listener()
