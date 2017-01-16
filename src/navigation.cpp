/*
 *  Created on: 12.05.2016
 *       Email: Nicko_Dema@protonmail.com
 *              ITMO University
 *              Robotics Engineering Department
 */

#include <navigation.h>

Navi::Navi(std::string node_name): nh_("~")
    // ,dest_as(nh_, node_name, boost::bind(&Navi::execute_cb, this, _1), false),
    // action_name_d_(node_name+"_dest_as"),
    // move_base_ac("move_base", true),
    , pnt_extr_regex("^[\\s]*([[:alnum:]]+)[\\s]*->[\\s]*([\\+|-]?(\\d+\\.?\\d*)|(\\.\\d+))"
               "[\\s]*:{1}[\\s]*([\\+|-]?(\\d+\\.?\\d*)|(\\.\\d+))")
{
    // dest_as.start();
    twist_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 2, true); //??

    set_twist_srv = nh_.advertiseService("set_twist", &Navi::set_twist_cb, this);
    manual_srv = nh_.advertiseService("manual",&Navi::manual_cb, this);
    stop_srv = nh_.advertiseService("stop", &Navi::stop_cb, this);
    dict_srv = nh_.advertiseService("dict",&Navi::dict_cb, this);
    mode_srv = nh_.advertiseService("mode",&Navi::mode_cb, this);
    get_point_srv = nh_.advertiseService("get_point",&Navi::get_point_cb, this);
    get_point_cli = nh_.serviceClient<navigation_step::PointData>("point_data");
//!!!!!!!!!!!!!!!!!!
    mode = 0x00;
    if (init_dict_load()) mode |= ld_pnts;

}

// void execute_cb(const navigation_step::DestGoalConstPtr &goal)
// {
//
//     ROS_INFO("[Navi]: Waiting for the move_base action server.");
//     move_base_ac.waitForServer()
//     ROS_INFO("[Navi]: Move_base action server has started.");
//
//     //Парсинг goal
//     //Поиск в словаре
//     //Задание точки
//     geometry_msgs::Point point =
//
//     //Отправка точки
//     move_base_msgs::MoveBaseGoal goal_mb;
//     goal.target_pose.header.frame_id = "map";
//     goal.target_pose.header.stamp = ros::Time::now();
//     goal.target_pose.pose.position = point;
//         // tf::Quaternion q(90, 0, 0);
//         // goal.target_pose.pose.orientation.x = (double)q.x();
//         // goal.target_pose.pose.orientation.y = (double)q.y();
//         // goal.target_pose.pose.orientation.z = (double)q.z();
//         // goal.target_pose.pose.orientation.w = (double)q.w();
//     goal.target_pose.pose.orientation.w = 1;
//         // goal.target_pose.pose.orientation.y = 0.707;
//
//     ROS_INFO("[+] Slave: sending goal-point {%f, %f}", point.x, point.y);
//     slave_client.sendGoal(goal_mb);
//     //slave_client.waitForResult(); -- без этого.
//
//     ros::Duration(3).sleep();
//
//     return true;
// }

// void Navi::make_pnt_callback ();
// {
//       //For manual mode.
//       //Here we should call a CV-service which will give as a lettering of point.
//       //After this, we should bind our position with gotten point and store it.
// }

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

bool Navi::get_point_cb (std_srvs::Empty::Request&  req,
                         std_srvs::Empty::Response& res)
    {
        123;//!!!!!!!!!!!!!!!!!!!!!
    }

bool Navi::mode_cb (std_srvs::Empty::Request&  req,
                    std_srvs::Empty::Response& res)
{
    std::string str = std::to_string(mode);
    ROS_INFO("[Navi]: Mode: %s", str.c_str());
    return true;
}


//Режим для ручного управления и работы со словарями
//В запросе должна содержаться строка:
// current / new / reserve / movement_only
//Выход из режима и загрузка словаря вызовом сервиса stop
bool Navi::manual_cb (navigation_step::Manual::Request&  req,
                      navigation_step::Manual::Response& res)
{
    //Проверка текущего режима
    if (((mode & move_to_point) == move_to_point) || ((mode & twisting) == twisting))
    {
        ROS_INFO("[Navi]: Call stop service!");
        return false;
    }
    else
    {
        //Выбрать существующий словарь с приоритетом на основной mode
        if (req.dict == "current")
        {
            //Если точки уже были загружены, то используем основной
            if ((mode & ld_pnts) == ld_pnts)
            {
                dict_fs.open(dfile_path+"/points.dict", std::fstream::out |
                                                        std::fstream::app |
                                                        std::fstream::in);
                if (dict_fs.is_open())
                {
                    mode |= manual;
                    mode &= ~(res_dict);
                    ROS_INFO("[Navi]: Manual mode on.");
                }
                else
                {
                    ROS_INFO("[Navi]: The dictionary exists but file can't be read. MAYDAY!");
                    dict_fs.clear();
                    return false;
                }
            }
            else
            {
                //Иначе работаем с резервным
                struct stat buf;
                std::string dname(dfile_path+"/points.dict.res");
                if (stat(dname.c_str(), &buf) == 0)
                {
                    dict_fs.open(dname, std::fstream::out | std::fstream::app | std::fstream::in);
                    if (dict_fs.is_open())
                    {
                        ROS_INFO("[Navi]: Manual mode on but the reserve dictionary is used.");
                        mode |= manual | res_dict;
                    }
                    else
                    {
                        ROS_INFO("[Navi]: The dictionary exists but can't be read. MAYDAY!");
                        dict_fs.clear();
                        return false;
                    }
                }
            }
        }

        //Используем новый словарь
        else if (req.dict == "new")
        {
            //Проверка наличия директории в пакете
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
