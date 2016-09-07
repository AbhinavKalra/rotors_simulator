/****************************************************************************

Conversion from a quaternion to roll, pitch and yaw.

Nodes:
subscribed /pose (message of type geometry_msgs::PoseStamped)
published /orientation_rpy (message of type geometry_msgs::Vector3.h)

Based on http://answers.ros.org/question/11545/plotprint-rpy-from-quaternion/?answer=58479#post-id-58479.

****************************************************************************/

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_datatypes.h"

// Here I use global publisher and subscriber, since I want to access the
// publisher in the function MsgCallback:
ros::Publisher rpy_publisher;
ros::Subscriber quat_subscriber;

// Function for conversion of quaternion to roll pitch and yaw. The angles
// are published here too.
void MsgCallback(const geometry_msgs::PoseStamped msg)
{
    // the incoming geometry_msgs::PoseStamped is transformed to a tf::Quaterion
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg.pose.orientation, quat);

    // the tf::Quaternion has a method to acess roll pitch and yaw
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

    // the found angles are written in a geometry_msgs::Vector3
    geometry_msgs::Vector3 rpy;
    rpy.x = roll * 180.0 / M_PI;
    rpy.y = pitch * 180.0 / M_PI;
    rpy.z = yaw * 180.0 / M_PI;

    // this Vector is then published:
    rpy_publisher.publish(rpy);
    ROS_DEBUG("published rpy angles: roll=%f pitch=%f yaw=%f", rpy.x, rpy.y, rpy.z);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "quaternion_to_rpy");
    ros::NodeHandle n;
    rpy_publisher = n.advertise<geometry_msgs::Vector3>("orientation_rpy", 1);
    quat_subscriber = n.subscribe("pose", 1, MsgCallback);

    // check for incoming quaternions untill ctrl+c is pressed
    ROS_DEBUG("waiting for quaternion");
    ros::spin();
    return 0;
}