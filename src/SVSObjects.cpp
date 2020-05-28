#include "svs_ros_connector/SVSObjects.h"

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
            tf2::Vector3 p = i->second;
            tf2::Matrix3x3 rot(curRot[n]);
            double roll, pitch, yaw;
            rot.getRPY(roll, pitch, yaw);

            cmds << "add " << n << " world ";
            cmds << "p " << p[0] << " " << p[1] << " " << p[2];
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
        tf2::Vector3 p = i->second;
        tf2::Quaternion q = svsRot[n];


        if (curPos[n] != p || curRot[n] != q) {
            cmds << "change " << n;
            if (curPos[n] != p) {
                cmds << " p " << p[0] << " " << p[1] << " " << p[2];
            }
            if (curRot[n] != q) {
                tf2::Matrix3x3 rot(q);
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
