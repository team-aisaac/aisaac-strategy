#pragma once
#include <vector>

#include <consai_msgs/robot_commands.h>

#include "aisaac-xbee-base.h"
#include "aisaac-wifi-base.h"
#include "aisaac-com-topic-struct.h"
#include "aisaac_communication/aisaaccommand.pb.h"


namespace aisaac
{
    enum class phyTechnology {
        undef = 0,
        xbee,
        wifi
    };

    struct nodeCommParam {
        phyTechnology phyTech;
        std::vector<unsigned char> addr;
    };

    class AisaacBitmapHandler {
    public:
        AisaacBitmapHandler(int _numRobots);
        ~AisaacBitmapHandler() {}
        void setXBeeInterface(AisaacXBeeBase*);
        void setWiFiInterface(AisaacWifiBase*);
        int setNodePHYAddress(int, phyTechnology, std::vector<unsigned char>);
        void showInstanceStatus();
        void setShutdown(bool);
        void convertToString(commandToRobot, std::vector<unsigned char>&);
        void convertFromCommandRealToProtobufEncodedString(const consai_msgs::robot_commands_realConstPtr&, std::vector<unsigned char>&);
        void convertFromCommandRealToProtobufEncodedStringVision(const consai_msgs::robot_commands_realConstPtr&, std::vector<unsigned char>&);
        void generateFT4(commandToRobot, std::vector<unsigned char>&);
        int parseFT4(std::vector<unsigned char> in, commandToRobot &out);
        void sendCommand(int, unsigned char, std::vector<unsigned char>);
        void messageHub(std::vector<unsigned char> in);
        virtual void messageReceiver(int reason, std::vector<unsigned char> message);
        // DEBUG
        void sayHello();
    protected:
        bool xbeeActivated = false;
        bool wifiActivated = false;
        bool isShutdown = false;
        int numRobots;
        std::vector<nodeCommParam> nodeList;
        std::vector<commandToRobot> commandList;
        AisaacXBeeBase *xbeeIF;
        AisaacWifiBase *wifiIF;
    };
} // namespace aisaac