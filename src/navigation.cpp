/*
 * Code reorganization for navigation step.
 *
 *  Created on: 12.05.2016
 *       Email: Nicko_Dema@protonmail.com
 *              ITMO University
 *              Robotics Engineering Department
 */

#include <navigation.h>

Navi::Navi(): nh_("~")
{
    clked_pnt_sub = nh.subscribe("clicked_point", 30, clkd_pnt_callback)
    start_srv = nh.advertiseService("start", start_callback)
}

void Navi::clkd_pnt_callback (const geometry_msgs::PointStamped::ConstPtr& stamped);
{
    //----------------------
}

bool Navi::start_callback (std_srvs::Empty::Request& req, std_srvs::Empty::Response&);
{
    //---------------
}


Navi::~Navi()
{
    clked_pnt_sub.shutdown();
    start_srv.shutdown();
}
