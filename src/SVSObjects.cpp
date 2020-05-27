#include "svs_ros_connector/SVSObjects.h"

void SVSObjects::update(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    curPos.clear();
    curRot.clear();

    for (int i = 0; i < msg->name.size(); i++) {
    }
}
