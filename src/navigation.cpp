/*
 *  Created on: 12.05.2016
 *       Email: Nicko_Dema@protonmail.com
 *              ITMO University
 *              Robotics Engineering Department
 */

#include <navigation.h>

Navi::Navi(std::string node_name): nh_("~")
    //dest_as(nh_, node_name, boost::bind(&Navi::execute_cb, this, _1), false),
    //action_name_d_(node_name+"_dest"),
    //move_base_ac("move_base", true),
{
    //dest_as.start();
    twist_pub = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 2);

    set_twist_srv = nh_.advertiseService("set_twist", &Navi::set_twist_cb, this);
    stop_srv = nh_.advertiseService("stop_moving", &Navi::stop_cb, this);

    mode = 0;

}

// void execute_cb(const navigation_step::DestGoalConstPtr &goal)
// {
//
// }

// void Navi::clkd_pnt_callback (const geometry_msgs::PointStamped::ConstPtr& stamped);
// {
//     //----------------------
// }

bool Navi::set_twist_cb (navigation_step::Twist::Request& req,
                         navigation_step::Twist::Response& res)
{
    if ((mode & move_to_point) != 0)
    {
        ROS_INFO("[Navi]: Somebody's trying to move base while move_base work.");
        return false;
    }
    else
    {
        mode |= twisting;
        ROS_INFO("[Navi]: Twisting on");
        twist_msg.linear.x = req.twist.linear.x;
        twist_msg.linear.y = req.twist.linear.y;
        twist_msg.angular.z = req.twist.angular.z;
        return true;
    }
}


bool Navi::stop_cb (std_srvs::Empty::Request& req,
                    std_srvs::Empty::Response& res)
{
    mode &= ~twisting;
    ROS_INFO("[Navi]: Twisting off");
    twist_msg.linear.x = 0;
    twist_msg.linear.y = 0;
    twist_msg.angular.z = 0;
    twist_pub.publish(twist_msg);
    return true;
}

void Navi::spin()
{
    ros::Rate R(50);
    while(nh_.ok())
    {
        ros::spinOnce();
        if ((mode & twisting) == 0x02)
        {
            twist_pub.publish(twist_msg);
        }
        R.sleep();
    }
}


Navi::~Navi()
{
    set_twist_srv.shutdown();
    stop_srv.shutdown();
}



// typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
// std::queue<geometry_msgs::Point> points;
//
// //ЗДЕСЬ МНОГО ЧЕРНОВОЙ ХЕРНИ
//
// bool start_callback(std_srvs::Empty::Request &req,
//                     std_srvs::Empty::Response &resp) {
//     ROS_INFO("[*] Slave: Starting...");
//     MoveBaseClient slave_client("move_base", true);
//     while(!slave_client.waitForServer(ros::Duration(5.0))){
//         ROS_INFO("[*] Slave: Waiting for the move_base action server");
//     }
//
//     while (!points.empty()) {
//         geometry_msgs::Point point = points.front();
//         points.pop();
//
//         // TODO: send
//         // TODO: wait for result
//         // TODO: sleep 3 seconds
//
//         move_base_msgs::MoveBaseGoal goal;
//         goal.target_pose.header.frame_id = "map";
//         goal.target_pose.header.stamp = ros::Time::now();
//         goal.target_pose.pose.position = point;
//         // tf::Quaternion q(90, 0, 0);
//         // goal.target_pose.pose.orientation.x = (double)q.x();
//         // goal.target_pose.pose.orientation.y = (double)q.y();
//         // goal.target_pose.pose.orientation.z = (double)q.z();
//         // goal.target_pose.pose.orientation.w = (double)q.w();
//         goal.target_pose.pose.orientation.w = 1;
//         // goal.target_pose.pose.orientation.y = 0.707;
//
//         ROS_INFO("[+] Slave: sending goal-point {%f, %f}", point.x, point.y);
//         slave_client.sendGoal(goal);
//         slave_client.waitForResult();
//
//         ros::Duration(3).sleep();
//     }
//
//     ROS_INFO("[!] Slave: no more points");
//
//     return true;
// }
