#pragma once

#include <map>
#include <string>
#include <sstream>

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <tf2/utils.h>
#include "gazebo_msgs/ModelStates.h"

class SVSObjects {
public:
    SVSObjects() {}

    void update(const gazebo_msgs::ModelStates::ConstPtr& msg);
    std::string getSGELCommandsAndSync();

private:
    static const double POS_THRESH;
    static const double ROT_THRESH;
    bool diffAboveThresh(tf2::Vector3 p1, tf2::Vector3 p2);
    bool diffAboveThresh(tf2::Quaternion r1, tf2::Quaternion r2);

    std::map<std::string, tf2::Vector3> curPos;
    std::map<std::string, tf2::Vector3> svsPos;

    std::map<std::string, tf2::Quaternion> curRot;
    std::map<std::string, tf2::Quaternion> svsRot;

    boost::mutex curMutex;
    boost::mutex svsMutex;
};
