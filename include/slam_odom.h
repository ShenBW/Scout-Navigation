#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>

namespace slam_odom
{
    class SlamOdomPublisher
    {

    public:
        explicit SlamOdomPublisher(ros::NodeHandle nh);

        virtual ~SlamOdomPublisher();

        bool readParameters();

        void run();

        bool getRobotPose();

        bool publishOdom();

        void quatToRPY(const tf2::Quaternion &quat, double &roll, double &pitch, double &yaw);

    private:
        ros::NodeHandle nh_;

        std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        ros::Publisher odom_pub_;

        geometry_msgs::TransformStamped last_transform_msg_;
        geometry_msgs::TransformStamped transform_msg_;

        double rate_;
        bool first_run_;
    };

} // namespace slam_odom
