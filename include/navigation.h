/*
 *  Created on: 12.05.2016
 *       Email: Nicko_Dema@protonmail.com
 *              ITMO University
 *              Robotics Engineering Department
 */

#ifndef NAV_STEP_
#define NAV_STEP_

#include <unordered_map>
#include <sys/stat.h>
#include <fstream>
#include <cstdlib>
#include <regex>

#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <tf/tf.h>
#include <std_srvs/Empty.h>
#include <ros/callback_queue.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/server/simple_action_server.h>

#include <navigation_step/DestAction.h>
#include <navigation_step/Twist.h>
#include <navigation_step/Manual.h>
#include <navigation_step/Dict.h>

class Navi
{
    public:
        Navi(std::string);
        ~Navi();
        void spin();

    protected:
        ros::NodeHandle nh_;

        unsigned char mode;
        enum modes
        {
            move_to_point  = 0x01,
            twisting       = 0x02,
            manual         = 0x04,
            ld_pnts        = 0x08,      //This flag means the dictionary had been loaded
            res_dict       = 0x16       //This flag is set when the reserve dict is used
            // ++++
        };

        std::unordered_map<std::string, std::pair<double, double>> points;
        std::string pnt_extr_regex;
        std::string dfile_path;
        std::fstream dict_fs;

        // ACTIONLIB used to communicate with CONTROL_NODE
        // actionlib::SimpleActionServer<navigation_step::DestAction> dest_as;
        // void execute_cb(const navigation_step::DestGoalConstPtr&)        // &goal??
        // std::string action_name_d_;

        //ACTIONLIB client for move_base
        // actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> move_base_ac;

        ros::ServiceServer stop_srv;
        ros::ServiceServer manual_srv;
        ros::ServiceServer set_twist_srv;
        ros::ServiceServer dict_srv;

        bool stop_cb (std_srvs::Empty::Request&, std_srvs::Empty::Response&);
        bool set_twist_cb (navigation_step::Twist::Request&, navigation_step::Twist::Response&);
        bool manual_cb (navigation_step::Manual::Request&, navigation_step::Manual::Response&);
        bool dict_cb (navigation_step::Dict::Request&, navigation_step::Dict::Response&);

        ros::Publisher twist_pub;
        geometry_msgs::Twist twist_msg;

        bool load_points();
        bool init_dict_load();

};

#endif  /*NAV_STEP_*/
