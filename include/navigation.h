/*
 * Code reorganization for navigation step.
 *
 *  Created on: 12.05.2016
 *       Email: Nicko_Dema@protonmail.com
 *              ITMO University
 *              Robotics Engineering Department
 */

#ifndef NAV_STEP_
#define NAV_STEP_

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

calss Navi
{
    public:
        Navi();
        ~Navi();
        spin();

    private:
        ros::NodeHandle nh;

        ros::Subscriber clked_pnt_sub;          //points from MoveBaseServer
        //ros::Subscriber pnt_sub;                //points from any other nodes

        //ACTIONLIB should used to communicate with CONTROL_NODE
        // +++++++++++++
        ros::ServiceServer start_srv;
        bool start_callback (std_srvs::Empty::Request& req, std_srvs::Empty::Response&);
        // +++++++++++++

        void clkd_pnt_callback (const geometry_msgs::PointStamped::ConstPtr&);

}



#endif  /*NAV_STEP_*/
