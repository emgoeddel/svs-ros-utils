#pragma once

#include <map>

#include <ros/ros.h>
#include <tf2/utils.h>
#include "gazebo_msgs/ModelStates.h"

class SVSObjects {
public:
    SVSObjects() {}

    void update(const gazebo_msgs::ModelStates::ConstPtr& msg);

private:
    std::map<std::string, tf2::Vector3> curPos;
    //std::map<std::string, tf2::Vector3> svsPos;

    std::map<std::string, tf2::Quaternion> curRot;
    //std::map<std::string, tf2::Quaternion> svsRot;
}
