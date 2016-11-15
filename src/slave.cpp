#include <ros/ros.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <queue>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
std::queue<geometry_msgs::Point> points;


bool stop_callback(std_srvs::Empty::Request &req,
                   std_srvs::Empty::Response &resp) {
    ROS_INFO("[!] Slave: Stopping...");
    ros::shutdown();

    return true;
}


bool start_callback(std_srvs::Empty::Request &req,
                    std_srvs::Empty::Response &resp) {
    ROS_INFO("[*] Slave: Starting...");
    MoveBaseClient slave_client("mb", true);
    while(!slave_client.waitForServer(ros::Duration(5.0))){
        ROS_INFO("[*] Slave: Waiting for the move_base action server");
    }

    while (!points.empty()) {
        geometry_msgs::Point point = points.front();
        points.pop();

        // TODO: send
        // TODO: wait for result
        // TODO: sleep 3 seconds

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position = point;
        // tf::Quaternion q(90, 0, 0);
        // goal.target_pose.pose.orientation.x = (double)q.x();
        // goal.target_pose.pose.orientation.y = (double)q.y();
        // goal.target_pose.pose.orientation.z = (double)q.z();
        // goal.target_pose.pose.orientation.w = (double)q.w();
        goal.target_pose.pose.orientation.w = 1;
        // goal.target_pose.pose.orientation.y = 0.707;

        ROS_INFO("[+] Slave: sending goal-point {%f, %f}", point.x, point.y);
        slave_client.sendGoal(goal);
        slave_client.waitForResult();

        ros::Duration(3).sleep();
    }

    ROS_INFO("[!] Slave: no more points");

    return true;
}


void point_callback(const geometry_msgs::Point::ConstPtr& point) {
    ROS_INFO("[+] Slave: pushing point");
    points.push(*point);
}


int main(int argc, char **argv) {
    ros::init(argc, argv, "slave_node");
	ros::NodeHandle node_slave;

    ros::ServiceServer service_stop = node_slave.advertiseService(
        "slave/stop", stop_callback);
    ros::ServiceServer service_start = node_slave.advertiseService(
        "slave/start", start_callback);
    ros::Subscriber sub = node_slave.subscribe(
        "slave/point", 1000, point_callback);
    ros::Rate rate(1);
    ROS_INFO("[+] Slave ready");

    while (ros::ok()) {
        ROS_INFO("[*] Slave: OK");

        ros::spinOnce();
        rate.sleep();
    }
}
