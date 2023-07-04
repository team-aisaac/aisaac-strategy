#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <consai_msgs/robot_commands_real.h>
#include <aisaac/Shutdown.h>
#include <tf/tf.h>
#include <vector>
#include <sstream>
#include <iomanip>
#include <signal.h>
#include <errno.h>
#include "aisaac_communication/aisaac-bitmap-handler.h"
// #include "aisaac_communication/aisaac-com-topic-struct.h"
#include "aisaac_communication/aisaac-xbee-linux.h"
#include "aisaac_communication/aisaac-wifi-linux.h"
// #include "aisaac_communication/aisaaccommand.pb.h"

#define VEL_MAX 3500

volatile sig_atomic_t errorFlag;

namespace aisaac {
    class Sender : public AisaacBitmapHandler {
    public:
        Sender(int _numRobots, ros::NodeHandle *_n) : AisaacBitmapHandler(_numRobots), n(_n) {
            pub = n->advertise<std_msgs::String>("xbeeUplink", 1000);
        }
        void messageReceiver(int reason, std::vector<unsigned char> message) override {
            std_msgs::String topic_msg;
            std::stringstream ss;
            for (auto mes: message) ss << std::hex << std::setw(2) << std::setfill('0') << std::uppercase << (int)mes;
            topic_msg.data = ss.str();
            pub.publish(topic_msg);
        }
    private:
        ros::NodeHandle *n;
        ros::Publisher pub;
    };
}

class TopicRxHandler {
public:
    TopicRxHandler() : sequenceNumber(0) {}
    void command_real_callback(const consai_msgs::robot_commands_realConstPtr& msg) {
        // Send SSL Vision data
        std::vector<unsigned char> commandvec;
        sender->encodeVisionInfo(msg, commandvec);
        sender->sendCommand(robotID, sequenceNumber, commandvec);
        // Send command message
        sender->encodeCommandMessage(msg, commandvec);
        sender->sendCommand(robotID, sequenceNumber++, commandvec);
    }
    void setSender(aisaac::Sender *_sender) {
        sender = _sender;
    }
    int robotID = 0;
private:
    aisaac::commandToRobot _commands;
    double current_orientation[3] = {0, 0, 0};
    unsigned char sequenceNumber;
    aisaac::Sender *sender;
};

void signalHandler(int signo){
    switch(signo)
    {
        case SIGQUIT:
        case SIGINT:
        case SIGKILL:
        case SIGILL:
        case SIGTERM:
            errorFlag = 1;
            break;
        default:
            break;
    }
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "command_sender");
    ros::NodeHandle nh;
    ros::Rate r(60);

    const int numRobots = 5;

    aisaac::AisaacWifiLinux wifiIF;

    aisaac::Sender sender(numRobots, &nh);
    sender.setWiFiInterface(&wifiIF);
    
    // Address settings
    // std::vector<unsigned char> destAddr = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    // sender.setNodePHYAddress(0, aisaac::phyTechnology::xbee, destAddr); // Node0: ROS PC
    sender.setNodePHYAddress(0, aisaac::phyTechnology::wifi, std::vector<unsigned char>{192, 168, 11, 5});
    sender.setNodePHYAddress(1, aisaac::phyTechnology::wifi, std::vector<unsigned char>{192, 168, 11, 6});
    sender.setNodePHYAddress(2, aisaac::phyTechnology::wifi, std::vector<unsigned char>{192, 168, 11, 7});
    sender.setNodePHYAddress(3, aisaac::phyTechnology::wifi, std::vector<unsigned char>{192, 168, 11, 8});
    sender.setNodePHYAddress(4, aisaac::phyTechnology::wifi, std::vector<unsigned char>{192, 168, 11, 9});

    TopicRxHandler trHandlers[numRobots];
    ros::Subscriber command_sub[numRobots];

    trHandlers[0].robotID = 0;
    trHandlers[0].setSender(&sender);
    command_sub[0] = nh.subscribe("/blue/robot_0/robot_commands_real", 1, &TopicRxHandler::command_real_callback, &trHandlers[0]);
    trHandlers[1].robotID = 1;
    trHandlers[1].setSender(&sender);
    command_sub[1] = nh.subscribe("/blue/robot_1/robot_commands_real", 1, &TopicRxHandler::command_real_callback, &trHandlers[1]);
    trHandlers[2].robotID = 2;
    trHandlers[2].setSender(&sender);
    command_sub[2] = nh.subscribe("/blue/robot_2/robot_commands_real", 1, &TopicRxHandler::command_real_callback, &trHandlers[2]);
    trHandlers[3].robotID = 3;
    trHandlers[3].setSender(&sender);
    command_sub[3] = nh.subscribe("/blue/robot_3/robot_commands_real", 1, &TopicRxHandler::command_real_callback, &trHandlers[3]);
    trHandlers[4].robotID = 4;
    trHandlers[4].setSender(&sender);
    command_sub[4] = nh.subscribe("/blue/robot_4/robot_commands_real", 1, &TopicRxHandler::command_real_callback, &trHandlers[4]);

    errorFlag = 0;

    if (signal(SIGINT, signalHandler) == SIG_ERR)
    {
        printf("\ncan't catch SIGINT\n");
    }
    if (signal(SIGQUIT, signalHandler) == SIG_ERR)
    {
        printf("\ncan't catch SIGQUIT\n");
    }
    if (signal(SIGILL, signalHandler) == SIG_ERR)
    {
        printf("\ncan't catch SIGILL\n");
    }
    if (signal(SIGTERM, signalHandler) == SIG_ERR)
    {
        printf("\ncan't catch SIGTERM\n");
    }

    while(ros::ok() && errorFlag == 0) {
        ros::spinOnce();
        // r.sleep();
    }
    return 0;
}
