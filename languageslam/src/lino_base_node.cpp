
#include "lino_base.h"

int main(int argc, char** argv )
{
    ros::init(argc, argv, "lino_base_node");
    ros::NodeHandle nh("~");
    std::string param;
    nh.getParam("robot_name", param);
    ROS_WARN("Got parameter : %s", param.c_str());
    LinoBase lino;
    ros::spin();
    return 0;
}