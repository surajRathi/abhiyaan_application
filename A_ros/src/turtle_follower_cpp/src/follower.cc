#include <ros/ros.h>

#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>

#define MIN_DIST 1.5
#define VEL_MAX 2

const double pi = atan(1) * 4;

std::string TURTLE_FRAME(const std::string &name) {
    return "turtle_" + name;
}

std::string VEL_TOPIC(const std::string &name) {
    return "/" + name + "/cmd_vel";
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "follower");
    ros::NodeHandle node;
    ROS_INFO("Started...");

    if (argc != 3) {
        ROS_ERROR("Invalid number of arguments.");
        return 1;
    }

    std::string turtle_frame = TURTLE_FRAME(argv[1]);
    std::string goal_frame = TURTLE_FRAME(argv[2]);
    std::string vel_topic = VEL_TOPIC(argv[1]);

    const double min_dist_sq = MIN_DIST * MIN_DIST;
    double x = 0, y = 0, vr = 0, vl = 0, theta = 0;

    ros::Publisher publisher = node.advertise<geometry_msgs::Twist>(vel_topic, 1);

    tf2_ros::Buffer buffer;
    tf2_ros::TransformListener listener(buffer);

    geometry_msgs::TransformStamped trans;
    geometry_msgs::Twist vel;
    vel.linear.y = 0;
    vel.linear.z = 0;
    vel.angular.x = 0;
    vel.angular.y = 0;

    ROS_INFO((goal_frame + turtle_frame).c_str());

    ros::Duration(1.0).sleep();
    ros::Rate rate(60);
    while (ros::ok()) {
        try {
            trans = buffer.lookupTransform(turtle_frame, goal_frame, ros::Time(0));
        } catch (tf2::TransformException &e) {
            ROS_WARN("%s", e.what());
            rate.sleep();
            continue;
        }
        x = trans.transform.translation.x;
        y = trans.transform.translation.y;
        if (x * x + y * y > min_dist_sq) {
            theta = atan2(y, x);

            vel.linear.x = VEL_MAX * (pi - abs(theta)) / pi;
            vel.angular.z = theta;
        } else {
            vel.linear.x = 0;
            vel.angular.z = 0;
        }
        publisher.publish(vel);
        rate.sleep();
    }

    return 0;
}