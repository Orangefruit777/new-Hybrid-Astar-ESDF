#include "Astar_path/a_star_node.h"

#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "astar_node");
    ros::NodeHandle node_handle("~");

    AStarNode astar_node(node_handle);

    ros::Rate rate(10);

    while (ros::ok()) {
        astar_node.Run();

        ros::spinOnce();
        rate.sleep();
    }

    ros::shutdown();
    return 0;
}