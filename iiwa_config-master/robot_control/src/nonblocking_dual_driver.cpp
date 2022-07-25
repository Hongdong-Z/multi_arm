#include <ros/ros.h>
#include "traj_coordinator.h"

const unsigned int FREQUENCY_HZ = 2;

int main(int argc, char** argv) {
    ros::init(argc, argv, "traj_coordinator");
    TrajCoordinator coordinator(ros::this_node::getName());

    coordinator.run(FREQUENCY_HZ);
    return 0;
}