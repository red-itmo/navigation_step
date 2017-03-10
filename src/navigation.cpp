/*
 *  Created on: 12.05.2016
 *       Email: Nicko_Dema@protonmail.com
 *              ITMO University
 *              Robotics Engineering Department
 */

#include <navigation.h>

Navi::Navi(std::string node_name): nh_("~"),
    dest_as(nh_, node_name, boost::bind(&Navi::execute_cb, this, _1), false),
    dest_as_name(node_name+"_dest_as"),
    move_base_ac("move_base", true),
    pnt_extr_regex("^[\\s]*([[:alnum:]]+)[\\s]*->[\\s]*([\\+|-]?(\\d+\\.?\\d*)|(\\.\\d+))"
               "[\\s]*:{1}[\\s]*([\\+|-]?(\\d+\\.?\\d*)|(\\.\\d+))")
{
    dest_as.start();
    twist_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 2, true); //??

    set_twist_srv = nh_.advertiseService("set_twist", &Navi::set_twist_cb, this);
    manual_srv = nh_.advertiseService("manual",&Navi::manual_cb, this);
    stop_srv = nh_.advertiseService("stop", &Navi::stop_cb, this);
    dict_srv = nh_.advertiseService("dict",&Navi::dict_cb, this);
    mode_srv = nh_.advertiseService("mode",&Navi::mode_cb, this);
    set_orientation_srv = nh_.advertiseService("set_orientation",
                                                &Navi::set_orientation_cb, this);

    point_catcher_base_srv = nh_.advertiseService("point_catcher_base",&Navi::point_catcher_base_cb, this);
    point_catcher_srv = nh_.advertiseService("point_catcher",&Navi::point_catcher_cb, this);
    point_catcher_cli = nh_.serviceClient<navigation_step::PointData>("point_data");

    mode = 0x00;
    if (init_dict_load()) mode |= ld_pnts;
}

void Navi::execute_cb(const navigation_step::DestGoalConstPtr &goal)
{

    navigation_step::DestResult result;
    result.has_got = false;
    ros::Rate r(4);

    while(!move_base_ac.waitForServer(ros::Duration(10.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    if (!move_base_ac.isServerConnected())
    {
        ROS_INFO("Connection to move_base action server filed.");
        result.has_got = false;
        dest_as.setAborted(result);
    }
    else
    {
        actionlib::SimpleClientGoalState state_as = move_base_ac.getState();
        bool completed = false;

        move_base_msgs::MoveBaseGoal goal_mb;
        goal_mb.target_pose.header.frame_id = "/map";
        goal_mb.target_pose.header.stamp = ros::Time();
        goal_mb.target_pose.pose.position.x = points[goal->dest_loc].first;
        goal_mb.target_pose.pose.position.y = points[goal->dest_loc].second;
        goal_mb.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(points[goal->orientation].first);
        ROS_INFO("Reseived goal: %s", goal->dest_loc.c_str());
        ROS_INFO("Point: %lf %lf",points[goal->dest_loc].first,
                                      points[goal->dest_loc].second);

        move_base_ac.sendGoal(goal_mb);
        while(ros::ok() && !completed)
        {
            //
            if(dest_as.isPreemptRequested() || !ros::ok())
            {
                dest_as.setPreempted(result);
                ROS_INFO("%s: Preempted/Canceled", dest_as_name.c_str());
                completed = true;
            }

            state_as = move_base_ac.getState();
            //PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST.
            //move_base_ac.waitForResult();
            if(state_as.state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
            {
                ros::Duration(goal->duration).sleep();
                result.has_got = true;
                dest_as.setSucceeded(result);
                completed = true;
            }
            else if (state_as.state_ == actionlib::SimpleClientGoalState::PENDING || 
                     state_as.state_ == actionlib::SimpleClientGoalState::ACTIVE)
            {
                r.sleep();   
            }
            else
            {
                dest_as.setAborted(result);
                ROS_INFO("Move_base has ended with one of this terminal states: ");
                ROS_INFO("RECALLED, REJECTED, PREEMPTED, ABORTED or LOST.");
                completed = true;
            }
        }
    }
}


bool Navi::set_twist_cb (navigation_step::Twist::Request&  req,
                         navigation_step::Twist::Response& res)
{
    if ((mode & move_to_point) == move_to_point) {
        ROS_INFO("[Navi]: Somebody's trying to move base while move_base work.");
        return false;
    }
    else
    {
        mode |= twisting;
        twist_msg.linear.x = req.twist.linear.x;
        twist_msg.linear.y = req.twist.linear.y;
        twist_msg.angular.z = req.twist.angular.z;
        ROS_INFO("[Navi]: Twisting mode is set.");
        return true;
    }
}

bool Navi::point_catcher_cb (std_srvs::Empty::Request&  req,
                             std_srvs::Empty::Response& res)
{
    if ((mode & manual) == manual)
    {
        navigation_step::PointData srv;
        if (point_catcher_cli.call(srv))
        {
            tf::TransformListener listener;
            tf::StampedTransform transform;
            static tf::TransformBroadcaster br;
            transform.setOrigin(tf::Vector3(srv.response.position.x, srv.response.position.y, 0.0));
            transform.setRotation(tf::Quaternion(0, 0, 0, 1));
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                             "/base_link", srv.response.name));
            try{
                ros::Time now = ros::Time(0);
                listener.waitForTransform("/map", srv.response.name, now, ros::Duration(40));
                listener.lookupTransform("/map", srv.response.name, now, transform);
            }
            catch (tf::TransformException &ex)
            {
                ROS_ERROR("%s",ex.what());
                return false;
            }
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(),
                             "/map", srv.response.name));

            points[srv.response.name] = std::make_pair(transform.getOrigin().x()
                                                      ,transform.getOrigin().y());
            dict_fs << srv.response.name << " -> " << std::to_string(transform.getOrigin().x())
                    << " : " << std::to_string(transform.getOrigin().y()) << std::endl;
        }
        else
        {
            ROS_INFO("[Navi]: Failed to call service \"point_data\".");
            return false;
        }
        return true;
    }
    else
    {
        ROS_INFO("[Navi]: You are not in the manual mode.");
        return false;
    }

}

bool Navi::point_catcher_base_cb (navigation_step::BasePoint::Request&  req,
                                  navigation_step::BasePoint::Response& res)
{
    if ((mode & manual) == manual)
    {
        tf::TransformListener listener;
        tf::StampedTransform transform;
        static tf::TransformBroadcaster br;
        try{
            ros::Time now = ros::Time(0);
            listener.waitForTransform("/map", "/base_link", now, ros::Duration(20));
            listener.lookupTransform("/map", "/base_link", now, transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            return false;
        }
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", req.name));
        points[req.name] = std::make_pair(transform.getOrigin().x(),
                                          transform.getOrigin().y());
        dict_fs << req.name << " -> " << std::to_string(transform.getOrigin().x())
                << " : " << std::to_string(transform.getOrigin().y()) << std::endl;
        ROS_INFO("[Navi]: %lf : %lf", transform.getOrigin().x(), transform.getOrigin().y());
        return true;
    }
    else
    {
        ROS_INFO("[Navi]: You are not in the manual mode.");
        return false;
    }

}

bool Navi::set_orientation_cb (navigation_step::SetOrientation::Request&  req,
                               navigation_step::SetOrientation::Response& res)
{
    if ((mode & manual) == manual)
    {
        tf::TransformListener listener;
        tf::StampedTransform transform;
        double roll, pitch, yaw;
        try
        {
            ros::Time now = ros::Time(0);
            listener.waitForTransform("/map", "/base_link", now, ros::Duration(20));
            listener.lookupTransform("/map", "/base_link", now, transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s",ex.what());
            return false;
        }
        transform.getBasis().getRPY(roll, pitch, yaw);
        points[req.orientation] = std::make_pair(yaw,0);
        dict_fs << req.orientation << " -> " << std::to_string(yaw) << " : 0" << std::endl;
        std::string str = std::to_string(yaw);
        ROS_INFO("[Navi]: Set orientation %s %s.", req.orientation.c_str(),str.c_str());
        return true;
    }
    else
    {
        ROS_INFO("[Navi]: You are not in the manual mode.");
        return false;
    }
}

bool Navi::mode_cb (std_srvs::Empty::Request&  req,
                    std_srvs::Empty::Response& res)
{
    std::string str = std::to_string(mode);
    ROS_INFO("[Navi]: Mode: %s", str.c_str());
    return true;
}


//The function turn on the manual mode
//As an argument function can get one of this strings:
// current / new / reserve / movement_only
//To turn it off call "stop" service
bool Navi::manual_cb (navigation_step::Manual::Request&  req,
                      navigation_step::Manual::Response& res)
{
    //checking for collision of the modes
    if (((mode & move_to_point) == move_to_point) || ((mode & twisting) == twisting))
    {
        ROS_INFO("[Navi]: Call stop service!");
        return false;
    }
    else
    {
        mode &= ~ld_pnts;

        if (req.dict == "current")
        {
            //Если точки уже были загружены, то используем основной
            if ((mode & res_dict) == res_dict)
            {
                //работаем с резервным
                struct stat buf;
                std::string dname(dfile_path+"/points.dict.res");
                if (stat(dname.c_str(), &buf) == 0)
                {
                    dict_fs.open(dname, std::fstream::out | std::fstream::app | std::fstream::in);
                    if (dict_fs.is_open())
                    {
                        ROS_INFO("[Navi]: Manual mode on. The reserve dictionary is used.");
                        mode |= manual;
                    }
                    else
                    {
                        ROS_INFO("[Navi]: The reserve dictionary exists but can't be read. MAYDAY!");
                        dict_fs.clear();
                        return false;
                    }
                }
            }
            else
            {
                dict_fs.open(dfile_path+"/points.dict", std::fstream::out |
                                                        std::fstream::app |
                                                        std::fstream::in);
                if (dict_fs.is_open())
                {
                    mode |= manual;
                    ROS_INFO("[Navi]: Manual mode on.");
                }
                else
                {
                    ROS_INFO("[Navi]: The main dictionary exists but file can't be read. MAYDAY!");
                    dict_fs.clear();
                    return false;
                }
            }
        }

        //choosing a new dictionary
        else if (req.dict == "new")
        {
            boost::filesystem::path dir_name(dfile_path);
            if(!(boost::filesystem::exists(dir_name)))
                boost::filesystem::create_directory(dir_name);
            //Проверка наличия файла, если есть, то сохраняем как резервный
            struct stat buf;
            std::string dname(dfile_path+"/points.dict");
            if (stat(dname.c_str(), &buf) == 0)
            {
                std::string buff;
                std::fstream res_dict_fs;
                res_dict_fs.open(dfile_path+"/points.dict.res", std::fstream::out |
                                                                std:: fstream::trunc);
                dict_fs.open(dfile_path+"/points.dict", std::fstream::in);
                while (std::getline(dict_fs, buff)) res_dict_fs << buff << std::endl;
                res_dict_fs.close();
                dict_fs.close();
                ROS_INFO("[Navi]: Save a current as a reserve.");
            }
            dict_fs.open(dfile_path+"/points.dict", std::fstream::out |
                                                    std:: fstream::trunc);
            if (dict_fs.is_open())
            {
                mode |= manual;
                mode &= ~(res_dict);
                dict_fs << "# Files of this type may contain\n"
                        << "# comments like this one." << std::endl;
                ROS_INFO("[Navi]: A new dictionary is created. Manual mode on.");
            }
            else
            {
                ROS_INFO("[Navi]: Can't create a new dictionary. MAYDAY!");
                return false;
            }
        }

        //Используем резервный словарь
        else if (req.dict == "reserve")
        {
            struct stat buf;
            std::string dname(dfile_path+"/points.dict.res");
            if (stat(dname.c_str(), &buf) == 0)
            {
                dict_fs.open(dname, std::fstream::out | std::fstream::app | std::fstream::in);
                if (dict_fs.is_open())
                {
                    ROS_INFO("[Navi]: Manual mode on. The reserve dictionary is used.");
                    mode |= manual | res_dict;
                }
                else
                {
                    ROS_INFO("[Navi]: The reserve dictionary exists but can't be read. MAYDAY!");
                    dict_fs.clear();
                }
            }
        }

        //Подрежим без редактирования словаря (для отметки виртуальных препятствий например)
        else if (req.dict == "movement_only")
        {
            mode |= manual;
            ROS_INFO("[Navi]: Manual mode on. None of dictionaries is used.");
        }
        else
        {
            ROS_INFO("[Navi]: A Wrong value in request's field.");
            return false;
        }
        return true;
    }
}

bool Navi::stop_cb (std_srvs::Empty::Request&  req,
                    std_srvs::Empty::Response& res)
{
    if ((mode & twisting) == twisting)
    {
        twist_msg.linear.x = 0;
        twist_msg.linear.y = 0;
        twist_msg.angular.z = 0;
        twist_pub.publish(twist_msg);
        ROS_INFO("[Navi]: Twisting has been stoped.");
    }
    if ((mode & manual) == manual)
    {
        dict_fs.close();
        load_points();
        ROS_INFO("[Navi]: Manual mode off.");
    }

    mode &= ~(twisting | manual);
    return true;
}

bool Navi::dict_cb (navigation_step::Dict::Request&  req,
                    navigation_step::Dict::Response& res)
{
    ROS_INFO("[Navi]: Reloading dictionary..");
    if (req.dict == "main") {
        mode &= ~res_dict;
    }
    else if (req.dict == "reserve") {
        mode |= res_dict;
    }
    else
    {
        ROS_INFO("[Navi]: A Wrong value in request's field.");
        return false;
    }
    return load_points();
}

void Navi::spin()
{
    ros::Rate R(50);
    while(nh_.ok())
    {
        ros::spinOnce();
        if ((mode & twisting) == twisting) twist_pub.publish(twist_msg);
        //if ((mode &

        R.sleep();
    }
}

bool Navi::init_dict_load()
{
    std::string line;
    //Попытка найти словарь в соответствующей директории пакета.
    std::regex reg("/home.+/src", std::regex_constants::ECMAScript |
                                  std::regex_constants::icase);
    std::smatch res;
    std::string pac_path = std::getenv("ROS_PACKAGE_PATH");

    if (std::regex_search(pac_path,res, reg))
        dfile_path = res.str()+"/navigation_step/dict";

    ROS_INFO("[Navi]: Trying to load a dictionary (dict/points.dict).");
    dict_fs.open(dfile_path+"/points.dict", std::fstream::in);
    if (!dict_fs.is_open())
    {
        ROS_INFO("[Navi]: List of points has not been created yet.");
        ROS_INFO("[Navi]: Call a manual service with a 'new' parameter to do it.");

        dict_fs.clear();
        return false;
    }
    else
    {
        //Переопределяем регулярное выражение для получения значений точек
        reg.assign(pnt_extr_regex, std::regex_constants::ECMAScript | std::regex_constants::icase);
        while (std::getline(dict_fs, line))
        {
            if (std::regex_search(line, res, reg)) {
                points[res[1]] = std::make_pair(std::stod(res[2]),std::stod(res[5]));
            }
        }
        ROS_INFO("[Navi]: %lu point(s) has been loaded.", points.size());

        dict_fs.close();
    }
    return true;
}

//В функцию передается имя загружаемого словаря
bool Navi::load_points()
{
    std::string dname;
    if((mode & res_dict) == res_dict) {
        dname = "/points.dict.res";
    }
    else {
        dname = "/points.dict";
    }
    points.clear();
    dict_fs.open(dfile_path+dname, std::fstream::in);
    if (!dict_fs.is_open())
    {
        ROS_INFO("[Navi]: Can't open %s to load points. MAYDAY!", dname.c_str());
        mode &= ~(ld_pnts);
        dict_fs.clear();
        return false;
    }
    else
    {
        std::string line;
        //Попытка найти словарь в соответствующей директории пакета.
        std::regex reg(pnt_extr_regex, std::regex_constants::ECMAScript |
                                       std::regex_constants::icase);
        std::smatch res;

        while (std::getline(dict_fs, line))
        {
            if (std::regex_search(line, res, reg)) {
                points[res[1]] = std::make_pair(std::stod(res[2]),std::stod(res[5]));
            }
        }
        ROS_INFO("[Navi]: %lu point(s) has been loaded from %s.", points.size(), dname.c_str());
        dict_fs.close();
        mode |= ld_pnts;
        return true;
    }
}

Navi::~Navi()
{
    twist_pub.shutdown();
    set_twist_srv.shutdown();
    stop_srv.shutdown();
    manual_srv.shutdown();
}
