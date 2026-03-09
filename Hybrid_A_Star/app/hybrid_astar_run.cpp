#include "hybrid_a_star/hybrid_a_star_node.h"

#include "3rd/backward.hpp"

#include <ros/ros.h>

namespace backward {
backward::SignalHandling sh;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "hybrid_astar_node");
    ros::NodeHandle node_handle("~");

    HybridAstarNode hybrid_astar_node(node_handle);

    ros::Rate rate(10);

    while (ros::ok()) {
        hybrid_astar_node.Run();

        ros::spinOnce();
        rate.sleep();
    }

    ros::shutdown();
    return 0;
}