import rospy
from geometry_msgs.msg import PoseStamped


def callback(data):
    rospy.loginfo(data.pose)


def listener():
    rospy.init_node('listener', anonymous=True)
    rospy.Subscriber("Detection", PoseStamped, callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
