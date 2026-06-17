#include "map_switch_node.hpp"

int main(int argc, char* argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc, argv, "tcp_map_switch_node");
    ros::NodeHandle nh;

    TcpMapSwitchNode node;
    if (!node.init())
    {
        return -1;
    }
    node.start();

    ros::spin();

    node.stop();
    return 0;
}