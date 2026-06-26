#include "map_switch_node.hpp"

int main(int argc, char* argv[])
{
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "tcp_map_switch_node");

    MapSwitchNode node;
    if (!node.init()) {
        return -1;
    }

    ros::spin();

    node.stop();
    return 0;
}
