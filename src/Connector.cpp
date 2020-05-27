#include <iostream>

#include <ros/ros.h>

#include "sml_Client.h"

class Connector {
public:

    Connector() {
        errorStatus = !setUpSoar();
    }

    // Handles setting up a local Soar agent and loading productions or
    // connecting to a remote Soar agent (ie in the debugger)
    // Returns true if setup was successful, false if an irrecoverable error happens
    bool setUpSoar() {
        // First check if we should spin up the Soar agent or connect to an
        // existing remote agent
        bool remoteSoar;
        if (!n.getParam("/svs_ros_connector/remote_soar", remoteSoar)) {
            ROS_WARN("Missing remote_soar parameter, assuming local agent.");
            remoteSoar = false;
        }

        if (remoteSoar) {
            ROS_INFO("Connecting to remote Soar process.");
            k = sml::Kernel::CreateRemoteConnection();
        } else {
            ROS_INFO("Creating Soar kernel.");
            k = sml::Kernel::CreateKernelInNewThread();
        }

        if (k->HadError()) {
            ROS_ERROR("%s", k->GetLastErrorDescription());
            return false;
        }

        // Get the existing agent if Soar is already running remotely
        if (remoteSoar) {
            int nAgents = k->GetNumberAgents();
            if (nAgents < 1) {
                ROS_ERROR("Remote Soar kernel has no agents.");
                return false;
            } else {
                if (nAgents > 1) {
                    ROS_WARN("Remote Soar kernel has multiple agents.");
                }
                a = k->GetAgentByIndex(0);
                ROS_INFO("Connected to remote Soar agent %s", a->GetAgentName());
            }
        }
        // Otherwise create an agent
        else {
            a = k->CreateAgent("soar");
            if (k->HadError()) {
                ROS_ERROR("%s", k->GetLastErrorDescription());
                return false;
            }
            // And load productions from the file given as parameter
            std::string agentFile;
            if (!n.getParam("/svs_ros_connector/agent_file", agentFile)) {
                ROS_WARN("Missing agent_file parameter, not loading productions.");
            } else if (agentFile.empty()) {
                ROS_WARN("No agent_file provided, not loading productions.");
            } else {
                ROS_INFO("Loading productions from %s", agentFile.c_str());
                bool success = a->LoadProductions(agentFile.c_str(), false);

                if (!success) {
                    ROS_ERROR("Could not load productions from %s", agentFile.c_str());
                    return false;
                }
            }
        }

        return true;
    }

    bool error() { return errorStatus; }

private:
    bool errorStatus;

    ros::NodeHandle n;
    sml::Kernel* k;
    sml::Agent* a;
};

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "svs_ros_connector");
    Connector c;

    if (!c.error()) {
        ROS_INFO("Connector started successfully.");
        ROS_INFO("Starting ROS spinner.");

        ros::AsyncSpinner s(4);
        s.start();
        ros::waitForShutdown();
    }

    return 0;
}
