#include "svs_ros_connector/SVSObjects.h"

const double SVSObjects::POS_THRESH = 0.001; // 1 mm
const double SVSObjects::ROT_THRESH = 0.017; // approx 1 deg

void SVSObjects::update(const gazebo_msgs::ModelStates::ConstPtr& msg) {
    boost::lock_guard<boost::mutex> guard(curMutex);
    curPos.clear();
    curRot.clear();

    for (int i = 0; i < msg->name.size(); i++) {
        geometry_msgs::Pose p = msg->pose[i];
        std::string n = msg->name[i];

        tf2::Vector3 v;
        tf2::fromMsg(p.position, v);
        tf2::Quaternion q;
        tf2::fromMsg(p.orientation, q);

        curPos.insert(std::pair<std::string, tf2::Vector3>(n, v));
        curRot.insert(std::pair<std::string, tf2::Quaternion>(n, q));
    }
}

std::string SVSObjects::getSGELCommandsAndSync() {
    boost::lock_guard<boost::mutex> guard1(curMutex);
    boost::lock_guard<boost::mutex> guard2(svsMutex);

    std::stringstream cmds;
    std::vector<std::string> objIds;

    // Add commands
    for (std::map<std::string, tf2::Vector3>::iterator i = curPos.begin();
         i != curPos.end(); i++) {
        if (svsPos.count(i->first) == 0) {
            std::string n = i->first;
            tf2::Vector3 curP = i->second;
            tf2::Matrix3x3 curR(curRot[n]);
            double roll, pitch, yaw;
            curR.getRPY(roll, pitch, yaw);

            cmds << "add " << n << " world ";
            cmds << "p " << curP[0] << " " << curP[1] << " " << curP[2];
            cmds << " r " << roll << " " << pitch << " " << yaw;
            cmds << std::endl;
        }
    }

    for (std::map<std::string, tf2::Vector3>::iterator i = svsPos.begin();
         i != svsPos.end(); i++) {
        // Delete commands
        if (curPos.count(i->first) == 0) {
            cmds << "delete " << i->first << " " << std::endl;
            continue;
        }

        // Change commands
        std::string n = i->first;
        tf2::Vector3 svsP = i->second;
        tf2::Vector3 curP = curPos[n];
        tf2::Quaternion svsQ = svsRot[n];
        tf2::Quaternion curQ = curRot[n];


        if (diffAboveThresh(svsP, curP) || diffAboveThresh(svsQ, curQ)) {
            cmds << "change " << n;
            if (diffAboveThresh(svsP, curPos[n])) {
                cmds << " p " << curP[0] << " " << curP[1] << " " << curP[2];
            }
            if (diffAboveThresh(svsQ, curQ)) {
                tf2::Matrix3x3 rot(curQ);
                double roll, pitch, yaw;
                rot.getRPY(roll, pitch, yaw);
                cmds << " r " << roll << " " << pitch << " " << yaw;
            }
            cmds << std::endl;
        }
    }

    // Sync
    svsPos = curPos;
    svsRot = curRot;

    return cmds.str();
}

bool SVSObjects::diffAboveThresh(tf2::Vector3 p1, tf2::Vector3 p2) {
    if (tf2::tf2Distance(p1, p2) > POS_THRESH) return true;
    return false;
}

bool SVSObjects::diffAboveThresh(tf2::Quaternion r1, tf2::Quaternion r2) {
    if (r1.angleShortestPath(r2) > ROT_THRESH) return true;
    return false;
}
