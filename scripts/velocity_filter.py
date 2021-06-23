import rospy

from geometry_msgs.msg import Twist


class VelocityFilter(object):
    def __init__(self):
        rospy.init_node("velocity_filter")

        self.linear_x_ratio = rospy.get_param("linear_x_ratio", 1.0)
        self.angular_z_ratio = rospy.get_param("angular_z_ratio", 0.3)

        self._cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/raw_cmd_vel", Twist, self._cmd_cb)

    def _cmd_cb(self, msg):
        msg.linear.x = *= self.linear_x_ratio
        msg.angular.z *= self.angular_z_ratio
        self._cmd_pub.publish = msg


if __name__ == "__main__":
    velocity_filter = VelocityFilter()
    rospy.spin()