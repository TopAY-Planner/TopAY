#include "interactive_marker/interactive_marker.h"


int main(int argc, char *argv[])
{
    const std::string robotName = "manual";
    // const std::string robotName = "moma";
    ros::init(argc, argv, robotName + "_target");
    ros::NodeHandle nodeHandle;

    InteractiveMarker targetPoseCommand(nodeHandle, robotName);
    targetPoseCommand.publishInteractiveMarker();

    // Successful exit
    return 0;
}