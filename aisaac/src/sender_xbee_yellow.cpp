#include <ros/ros.h>
#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <consai_msgs/robot_commands.h>
#include <aisaac/Shutdown.h>
#include <tf/tf.h>
#include <vector>
#include <sstream>
#include <iomanip>
#include <signal.h>
#include <errno.h>
#include "aisaac_communication/aisaac-bitmap-handler.h"
#include "aisaac_communication/aisaac-com-topic-struct.h"
#include "aisaac_communication/aisaac-xbee-linux.h"

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
    void command_callback(const consai_msgs::robot_commandsConstPtr& msg) {
        aisaac::commandToRobot commands;

        int32_t x_tmp = int32_t(msg->vel_surge * 1000);
        int32_t y_tmp = int32_t(msg->vel_sway * 1000);
        float vector_sum = pow(x_tmp,2) + pow(y_tmp,2); 
        if(sqrt(vector_sum) > VEL_MAX){
            float k = sqrt(vector_sum) / VEL_MAX;
            x_tmp = x_tmp / k;
            y_tmp = y_tmp / k;
        }
        commands.x_vector = (int16_t)x_tmp;
        commands.y_vector = (int16_t)y_tmp;
        
        commands.angleTypeSelect = 1;   // omega 
        commands.angle = (uint16_t)int16_t(msg->omega * (180.0 / M_PI) * 10);

        commands.calibrationValid = 0;
        commands.calibrationXPosition = 0;
        commands.calibrationYPosition = 0;

        double current_orientation_deg = current_orientation[2] * (180 / M_PI);
        while(!(0 <= current_orientation_deg && current_orientation_deg < 360)){
            if(current_orientation_deg < 0){
                current_orientation_deg += 360;
            }else{
                current_orientation_deg -= 360;
            }
        }
        commands.calibrationAngle = (uint16_t)uint16_t(current_orientation_deg);

        if(msg->kick_power > 0){
            commands.kickParameter.sensorUse = 1;   // kick_flag
            commands.kickParameter.kickType = 0;    // kick_chip_flag
            commands.kickParameter.kickStrength = 0;
        }else{
            commands.kickParameter.sensorUse = 0;   // kick_flag
            commands.kickParameter.kickType = 0;    // kick_chip_flag
            commands.kickParameter.kickStrength = 0;
        }

        commands.miscByte = 0;

        std::vector<unsigned char> commandvec;
        sender->generateFT4(commands, commandvec);
        sender->sendCommand(robotID, sequenceNumber++, commandvec);
    }
    void odom_callback(const nav_msgs::Odometry& msg) {
        tf::Quaternion quat(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
        tf::Matrix3x3(quat).getRPY(current_orientation[0], current_orientation[1], current_orientation[2]);
    }
    void shutdown_callback(const aisaac::Shutdown& msg) {
        // if(msg.shutdown_flag == true){
        //     shutdown_flag = true;
        // }
        sender->setShutdown(msg.shutdown_flag);
    }
    void setSender(aisaac::Sender *_sender) {
        sender = _sender;
    }
    int robotID = 0;
private:
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
    ros::Rate r(30);

    const int numRobots = 8;

    aisaac::AisaacXBeeLinux xbeeIF("/dev/xbee0");
    xbeeIF.setBroadcastRadius(5);
    std::vector<unsigned char> addr16 = {0xFF, 0xFE};
    xbeeIF.setShortDestAddr(addr16);
    xbeeIF.setXBeeOptions(0, 0, 0);

    aisaac::Sender sender(numRobots, &nh);
    sender.setXBeeInterface(&xbeeIF);
    
    // Address settings
    std::vector<unsigned char> destAddr = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    sender.setNodePHYAddress(0, aisaac::phyTechnology::xbee, destAddr);
    destAddr = {0x00, 0x13, 0xA2, 0x00, 0x00, 0x00, 0x00, 0x01};
    sender.setNodePHYAddress(1, aisaac::phyTechnology::xbee, destAddr);
    destAddr = {0x00, 0x13, 0xA2, 0x00, 0x00, 0x00, 0x00, 0x02};
    sender.setNodePHYAddress(2, aisaac::phyTechnology::xbee, destAddr);
    destAddr = {0x00, 0x13, 0xA2, 0x00, 0x00, 0x00, 0x00, 0x03};
    sender.setNodePHYAddress(3, aisaac::phyTechnology::xbee, destAddr);
    destAddr = {0x00, 0x13, 0xA2, 0x00, 0x00, 0x00, 0x00, 0x04};
    sender.setNodePHYAddress(4, aisaac::phyTechnology::xbee, destAddr);
    destAddr = {0x00, 0x13, 0xA2, 0x00, 0x00, 0x00, 0x00, 0x05};
    sender.setNodePHYAddress(5, aisaac::phyTechnology::xbee, destAddr);
    destAddr = {0x00, 0x13, 0xA2, 0x00, 0x00, 0x00, 0x00, 0x06};
    sender.setNodePHYAddress(6, aisaac::phyTechnology::xbee, destAddr);
    destAddr = {0x00, 0x13, 0xA2, 0x00, 0x00, 0x00, 0x00, 0x07};
    sender.setNodePHYAddress(7, aisaac::phyTechnology::xbee, destAddr);
    destAddr = {0x00, 0x13, 0xA2, 0x00, 0x00, 0x00, 0x00, 0x08};
    sender.setNodePHYAddress(8, aisaac::phyTechnology::xbee, destAddr);

    TopicRxHandler trHandlers[numRobots];
    ros::Subscriber command_sub[numRobots], odom_sub[numRobots], shutdown_sub;
    shutdown_sub = nh.subscribe("/yellow/shutdown", 1, &TopicRxHandler::shutdown_callback, &trHandlers[0]);

    trHandlers[0].robotID = 1;
    trHandlers[0].setSender(&sender);
    command_sub[0] = nh.subscribe("/yellow/robot_0/robot_commands", 1, &TopicRxHandler::command_callback, &trHandlers[0]);
    odom_sub[0] = nh.subscribe("/yellow/robot_0/odom", 1, &TopicRxHandler::odom_callback, &trHandlers[0]);
    trHandlers[1].robotID = 2;
    trHandlers[1].setSender(&sender);
    command_sub[1] = nh.subscribe("/yellow/robot_1/robot_commands", 1, &TopicRxHandler::command_callback, &trHandlers[1]);
    odom_sub[1] = nh.subscribe("/yellow/robot_1/odom", 1, &TopicRxHandler::odom_callback, &trHandlers[1]);
    trHandlers[2].robotID = 3;
    trHandlers[2].setSender(&sender);
    command_sub[2] = nh.subscribe("/yellow/robot_2/robot_commands", 1, &TopicRxHandler::command_callback, &trHandlers[2]);
    odom_sub[2] = nh.subscribe("/yellow/robot_2/odom", 1, &TopicRxHandler::odom_callback, &trHandlers[2]);
    trHandlers[3].robotID = 4;
    trHandlers[3].setSender(&sender);
    command_sub[3] = nh.subscribe("/yellow/robot_3/robot_commands", 1, &TopicRxHandler::command_callback, &trHandlers[3]);
    odom_sub[3] = nh.subscribe("/yellow/robot_3/odom", 1, &TopicRxHandler::odom_callback, &trHandlers[3]);
    trHandlers[4].robotID = 5;
    trHandlers[4].setSender(&sender);
    command_sub[4] = nh.subscribe("/yellow/robot_4/robot_commands", 1, &TopicRxHandler::command_callback, &trHandlers[4]);
    odom_sub[4] = nh.subscribe("/yellow/robot_4/odom", 1, &TopicRxHandler::odom_callback, &trHandlers[4]);
    trHandlers[5].robotID = 6;
    trHandlers[5].setSender(&sender);
    command_sub[5] = nh.subscribe("/yellow/robot_5/robot_commands", 1, &TopicRxHandler::command_callback, &trHandlers[5]);
    odom_sub[5] = nh.subscribe("/yellow/robot_5/odom", 1, &TopicRxHandler::odom_callback, &trHandlers[5]);
    trHandlers[6].robotID = 7;
    trHandlers[6].setSender(&sender);
    command_sub[6] = nh.subscribe("/yellow/robot_6/robot_commands", 1, &TopicRxHandler::command_callback, &trHandlers[6]);
    odom_sub[6] = nh.subscribe("/yellow/robot_6/odom", 1, &TopicRxHandler::odom_callback, &trHandlers[6]);
    trHandlers[7].robotID = 8;
    trHandlers[7].setSender(&sender);
    command_sub[7] = nh.subscribe("/yellow/robot_7/robot_commands", 1, &TopicRxHandler::command_callback, &trHandlers[7]);
    odom_sub[7] = nh.subscribe("/yellow/robot_7/odom", 1, &TopicRxHandler::odom_callback, &trHandlers[7]);

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
