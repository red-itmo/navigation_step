/* main.cpp
 *
 *  Created on: 12.05.2016
 *       Email: Nicko_Dema@protonmail.com
 *              ITMO University
 *              Robotics Engineering Department
 */

 #include <navigation.h>

 int main(int argc, char** argv)
 {
     ros::init(argc, argv, "navi");
     Navi YouBot_navi(ros::this_node::getName());
     YouBot_navi.spin();

     return 0;
 }
