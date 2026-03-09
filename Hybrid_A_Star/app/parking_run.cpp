
#include "hybrid_a_star/hybrid_parking.h"

#include <ros/ros.h>

int main(int argc, char **argv) {
    ros::init(argc, argv, "hybrid_parking");
    ros::NodeHandle node_handle("~");

    HybridParking hybrid_parking(node_handle);

    ros::Rate rate(10);

    while (ros::ok()) {
        hybrid_parking.Run();

        ros::spinOnce();
        rate.sleep();
    }

    ros::shutdown();
    return 0;
}