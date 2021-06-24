#include "slam_odom.h"

namespace slam_odom
{
    SlamOdomPublisher::SlamOdomPublisher(ros::NodeHandle nh)
        : nh_(nh),
          first_run_(true)
    {
        SlamOdomPublisher::readParameters();
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>("odom", 1);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>();
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    }

    SlamOdomPublisher::~SlamOdomPublisher() = default;

    bool SlamOdomPublisher::readParameters()
    {
        nh_.param("rate", rate_, 10.0);

        return true;
    }

    void SlamOdomPublisher::run()
    {
        ros::Rate loop(rate_);
        while (ros::ok())
        {
            bool listen_flag = SlamOdomPublisher::getRobotPose();
            if (listen_flag)
            {
                SlamOdomPublisher::publishOdom();
            }
            loop.sleep();
        }
    }

    bool SlamOdomPublisher::getRobotPose()
    {
        try
        {
            transform_msg_ = tf_buffer_->lookupTransform("odom", "base_link", ros::Time(0));
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            return false;
        }
        if (first_run_)
        {
            last_transform_msg_ = transform_msg_;
            first_run_ = false;
        }

        return true;
    }

    bool SlamOdomPublisher::publishOdom()
    {
        double dt = transform_msg_.header.stamp.toSec() - last_transform_msg_.header.stamp.toSec();
        if (dt < 1e-6)
        {
            return false;
        }

        tf2::Stamped<tf2::Transform> poseTmp;

        poseTmp.stamp_ = transform_msg_.header.stamp;

        poseTmp.setOrigin(
            tf2::Vector3(transform_msg_.transform.translation.x,
                         transform_msg_.transform.translation.y,
                         transform_msg_.transform.translation.z));

        tf2::Quaternion orientation;

        tf2::fromMsg(transform_msg_.transform.rotation, orientation);
        if (fabs(orientation.length() - 1.0) > 0.01)
        {
            ROS_WARN_ONCE("An input was not normalized, this should NOT happen, but will normalize.");
            orientation.normalize();
        }

        poseTmp.setRotation(orientation);

        tf2::Transform last_pose;
        last_pose.setOrigin(
            tf2::Vector3(last_transform_msg_.transform.translation.x,
                         last_transform_msg_.transform.translation.y,
                         last_transform_msg_.transform.translation.z));

        tf2::Quaternion last_orientation;
        tf2::fromMsg(last_transform_msg_.transform.rotation, last_orientation);
        if (fabs(last_orientation.length() - 1.0) > 0.01)
        {
            ROS_WARN_ONCE("An input was not normalized, this should NOT happen, but will normalize.");
            last_orientation.normalize();
        }
        last_pose.setRotation(last_orientation);

        poseTmp.setData(last_pose.inverseTimes(poseTmp));

        double linear_x = poseTmp.getOrigin().getX() / dt;
        double linear_y = poseTmp.getOrigin().getY() / dt;
        double linear_z = poseTmp.getOrigin().getZ() / dt;

        double angular_x = 0;
        double angular_y = 0;
        double angular_z = 0;

        SlamOdomPublisher::quatToRPY(
            poseTmp.getRotation(), angular_x, angular_y, angular_z);

        angular_x /= dt;
        angular_y /= dt;
        angular_z /= dt;

        nav_msgs::Odometry odom_msg;
        odom_msg.header.frame_id = "odom";
        odom_msg.header.stamp = transform_msg_.header.stamp;
        odom_msg.child_frame_id = "base_link";

        odom_msg.pose.pose.position.x = transform_msg_.transform.translation.x;
        odom_msg.pose.pose.position.y = transform_msg_.transform.translation.y;
        odom_msg.pose.pose.position.z = transform_msg_.transform.translation.z;
        odom_msg.pose.pose.orientation = transform_msg_.transform.rotation;

        odom_msg.twist.twist.linear.x = linear_x;
        odom_msg.twist.twist.linear.y = linear_y;
        odom_msg.twist.twist.linear.z = linear_z;
        odom_msg.twist.twist.angular.x = angular_x;
        odom_msg.twist.twist.angular.y = angular_y;
        odom_msg.twist.twist.angular.z = angular_z;

        odom_pub_.publish(odom_msg);

        return true;
    }

    void SlamOdomPublisher::quatToRPY(const tf2::Quaternion &quat, double &roll, double &pitch, double &yaw)
    {
        tf2::Matrix3x3 orTmp(quat);
        orTmp.getRPY(roll, pitch, yaw);
    }

}