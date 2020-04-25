#include <ros/ros.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <geometry_msgs/TransformStamped.h>
#include <turtlesim/Pose.h>

#define WORLD_FRAME "world"
#define MAP_FRAME "map"

std::string TURTLE_FRAME(const std::string &name) {
    return "turtle_" + name;
}

std::string POSE_TOPIC(const std::string &name) {
    return "/" + name + "/pose";
}

#define MAP_X 5.5
#define MAP_Y 5.5

tf2_ros::TransformBroadcaster *broadcaster;

void callback(const turtlesim::Pose::ConstPtr &p, geometry_msgs::TransformStamped *t) {
    t->header.stamp = ros::Time::now();
    t->transform.translation.x = p->x;
    t->transform.translation.y = p->y;
    t->transform.rotation.z = sin(p->theta / 2);
    t->transform.rotation.w = cos(p->theta / 2);

    broadcaster->sendTransform(*t);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "state_publisher");
    ros::NodeHandle node;
    ROS_INFO("Started...");

    //<editor-fold desc="Map Transform">
    geometry_msgs::TransformStamped trans;
    trans.child_frame_id = MAP_FRAME;

    trans.header.frame_id = WORLD_FRAME;
    trans.header.stamp = ros::Time::now();

    // Are they auto-inited to 0?
    trans.transform.translation.z = 0;
    trans.transform.rotation.x = 0;
    trans.transform.rotation.y = 0;

    trans.transform.translation.x = MAP_X;
    trans.transform.translation.y = MAP_Y;
    trans.transform.rotation.z = 0;
    trans.transform.rotation.w = 1;

    tf2_ros::StaticTransformBroadcaster staticBroadcaster;
    staticBroadcaster.sendTransform(trans);
    ROS_INFO("Map transform broadcasted.");
    //</editor-fold>


    tf2_ros::TransformBroadcaster br;
    broadcaster = &br;

    std::vector<ros::Subscriber> subs; // If i dont store them they apparently go away.
    std::vector<geometry_msgs::TransformStamped *> ts; // For deleting the Transforms

    for (++argv; *argv; ++argv) {
        if (*argv[0] == '_') continue; // Why isn't ros removing the '__log:=...' arg ?

        ROS_INFO("Publishing transform for the turtle: %s", *argv);

        // If i directly initialize object, in every iteration, it is stored in the same point in memory. !!!; thus use new and delete
        geometry_msgs::TransformStamped *t = new geometry_msgs::TransformStamped();
        t->child_frame_id = TURTLE_FRAME(*argv);
        t->header.frame_id = WORLD_FRAME;

        // Are they auto-inited to 0?
        t->transform.translation.z = 0;
        t->transform.rotation.x = 0;
        t->transform.rotation.y = 0;

        ts.push_back(t);

        // Or should i just use a class?
        // https://answers.ros.org/question/11810/how-to-pass-arguments-tofrom-subscriber-callback-functions/
        const boost::function<void(const turtlesim::Pose::ConstPtr &)> bound_callback = boost::bind(callback, _1, t);
        // https://answers.ros.org/question/12045/how-to-deliver-arguments-to-a-callback-function/
        subs.push_back(node.subscribe<turtlesim::Pose &>(POSE_TOPIC(*argv), 1, bound_callback));
    }

    ros::spin();

    for (geometry_msgs::TransformStamped *t: ts) delete t;
    return 0;
}