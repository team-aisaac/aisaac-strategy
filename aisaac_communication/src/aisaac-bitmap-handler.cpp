#include "aisaac_communication/aisaac-bitmap-handler.h"
#include "aisaac_communication/aisaac-xbee-base.h"
#include "aisaac_communication/aisaac-wifi-base.h"
#include "aisaac_communication/protocol.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <functional>
#include <cstdlib>
#include <cmath>

namespace aisaac
{
    AisaacBitmapHandler::AisaacBitmapHandler(int _numRobots) : numRobots(_numRobots) {
        for (int i=0; i < (numRobots+1); i++) {
            nodeList.push_back({phyTechnology::undef, {0, 0, 0, 0, 0, 0, 0, 0}});
        }
        // for (auto robo : nodeList) std::cout << (int)robo.phyTech << ", " << (int)robo.addr[0] << std::endl;
        // std::cout << "# of Robots: " << numRobots << std::endl;
    }
    // AisaacBitmapHandler::~AisaacBitmapHandler() {}
    void AisaacBitmapHandler::setXBeeInterface(AisaacXBeeBase *_xbeeIF) {
        xbeeIF = _xbeeIF;
        xbeeIF->setCallbackAisaacBitmapHandler(std::bind(&AisaacBitmapHandler::messageHub, this, std::placeholders::_1));
        xbeeActivated = true;
    }
    void AisaacBitmapHandler::setWiFiInterface(AisaacWifiBase *_wifiIF) {
        wifiIF = _wifiIF;
        wifiIF->setCallbackAisaacBitmapHandler(std::bind(&AisaacBitmapHandler::messageHub, this, std::placeholders::_1));
        wifiActivated = true;
    }
    int AisaacBitmapHandler::setNodePHYAddress(int nodeID, phyTechnology phyTech, std::vector<unsigned char> addr) {
        if (nodeID < 0 || nodeID >= numRobots) return -1;
        if (phyTech == phyTechnology::undef) {
            nodeList[nodeID].phyTech = phyTechnology::undef;
            return 0;
        } else if (phyTech == phyTechnology::xbee) {
            if (addr.size() != 8) return -1;    // invalid 64bit xbee address
            nodeList[nodeID].phyTech = phyTechnology::xbee;
            // for (int i=0; i<8; i++) nodeList[nodeID].addr[i] = addr[i];
            nodeList[nodeID].addr = addr;
            return 0;
        } else if (phyTech == phyTechnology::wifi) {
            if (addr.size() != 4) return -1;    // invalid IPv4 address
            nodeList[nodeID].phyTech = phyTechnology::wifi;
            // for (int i=0; i<4; i++) nodeList[nodeID].addr[i] = addr[i];
            nodeList[nodeID].addr = addr;
            return 0;
        }
        return -1;
    }
    void AisaacBitmapHandler::setShutdown(bool shutdownState) {
        isShutdown = shutdownState;
    }
    void AisaacBitmapHandler::showInstanceStatus() {
        std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
        std::cout << "# Robots: " << numRobots << std::endl;
        int index = 0;
        for (auto node : nodeList) {
            std::cout << index++ << ": ";
            if (node.phyTech == phyTechnology::undef) {
                std::cout << "Undefined communication IF" << std::endl;
            } else if (node.phyTech == phyTechnology::xbee) {
                std::cout << "XBee: ";
                for (int i=0; i<7; i++) std::cout << std::hex << std::setw(2) << std::setfill('0') << std::uppercase << (int)node.addr[i] << ":";
                std::cout << std::hex << std::setw(2) << std::setfill('0') << std::uppercase << (int)node.addr[7] << std::endl;
            } else if (node.phyTech == phyTechnology::wifi) {
                std::cout << "WiFi: ";
                for (int i=0; i<3; i++) std::cout << std::dec << (int)node.addr[i] << ".";
                std::cout << std::dec << (int)node.addr[3] << std::endl;

            }
        }
        std::cout << "XBee IF: " << (xbeeActivated ? "Active" : "Inactive") << std::endl;
        std::cout << "WiFi IF: " << (wifiActivated ? "Active" : "Inactive") << std::endl;
        std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~" << std::endl;
    }


    void AisaacBitmapHandler::encodeCommandMessage(const consai_msgs::robot_commands_realConstPtr& msg, std::vector<unsiged char> &out) {
        _strategy_pc_command command;
        command.goal_pose.x = (int32_t)(msg->goal_pose.x * 1000.0); // m -> mm
        command.goal_pose.y = (int32_t)(msg->goal_pose.x * 1000.0); // m -> mm
        command.goal_pose.theta = (int32_t)(msg->goal_pose.x * 18000.0 / M_PI); // rad -> deg x100
        command.middle_goal_pose.x = (int32_t)(msg->middle_goal_pose.x * 1000.0);   // m -> mm
        command.middle_goal_pose.y = (int32_t)(msg->middle_goal_pose.y * 1000.0);   // m -> mm
        command.middle_goal_pose.theta = (int32_t)(msg->middle_goal_pose.theta * 1000.0);   // m -> mm
        command.prohibited_zone_ignore = msg->prohibited_zone_ignore;
        command.middle_target_flag = msg->middle_target_flag;
        command.halt_flag = false;  // ToDo impliment
        // Kick
        command.kick_power = (uint32_t)(msg->kick_power);
        command.ball_kick_state = msg->ball_kick_state;
        command.ball_kick = msg->ball_kick;
        command.ball_kick_active = msg->ball_kick_active;
        command.free_kick_flag = msg->free_kick_flag;
        command.ball_goal.x = (int32_t)(msg->ball_goal.x * 1000.0); // m -> mm
        command.ball_goal.y = (int32_t)(msg->ball_goal.y * 1000.0); // m -> mm
        command.ball_target_allowable_error = msg->ball_target_allowable_error; // m
        command.kick_type = 0;  // ToDo impliment
        // Dribble
        command.dribble_power = (uint32_t)(msg->dribble_power);
        command.dribble_goal.x = (int32_t)(msg->dribble_goal.x * 1000.0);   // m -> mm
        command.dribble_goal.y = (int32_t)(msg->dribble_goal.y * 1000.0);   // m -> mm
        command.dribble_goal.theta = (int32_t)(msg->dribble_goal.theta * 18000.0 / M_PI);   // rad -> deg x100
        command.dribble_complete_distance = msg->dribble_complete_distance; // mm
        command.dribble_state = msg->dribble_state;
        command.dribbler_active = msg->dribbler_active;

        encodeStrategyPcCommand(&command, out.data());
    }

    void AisaacBitmapHandler::encodeVisionInfo(const consai_msgs::robot_commands_realConstPtr& msg, std::vector<unsiged char> &out) {
        _vision_data vision_data;
        vision_data.current_pose.x = (int32_t)(msg->current_pose.x);   // mm
        vision_data.current_pose.y = (int32_t)(msg->current_pose.y);   // mm
        vision_data.current_pose.theta = (int32_t)(msg->current_pose.theta);

        vision_data.ball_position.x = (int32_t)(msg->ball_position.x);
        vision_data.ball_position.y = (int32_t)(msg->ball_position.y);

        auto obstacles = msg->obstacles;
        int index = 0;
        for (auto obstacle : obstacles) {
            vision_data.obstacles[index].x = obstacle.pose.x;
            vision_data.obstacles[index].y = obstacle.pose.y;
            vision_data.obstacles[index].theta = obstacle.pose.theta;
            vision_data.obstacles[index].vx = obstacle.vel.x;
            vision_data.obstacles[index].vy = obstacle.vel.x;
        }
        vision_data.number_of_obstacles = index+1;

        encodeVisionData()
    }

    void AisaacBitmapHandler::generateFT4(commandToRobot command, std::vector<unsigned char> &out) {

        std::cout << "command";
        std::cout << " robotCommandCoordinateSystemType: " << (int)command.robotCommandCoordinateSystemType;
        std::cout << " targetX: " << command.targetX;
        std::cout << " targetY: " << command.targetY;
        std::cout << " targetAngle: " << command.targetAngle;
        std::cout << " visionDataValid: " << (int)command.visionDataValid;
        std::cout << " currentX: " << command.currentX;
        std::cout << " currentY: " << command.currentY;
        std::cout << " currentAngle: " << (int)command.currentAngle;
        std::cout << " kickParameter.sensorUse: " << (int)command.kickParameter.sensorUse;
        std::cout << " kickParameter.kickType: " << (int)command.kickParameter.kickType;
        std::cout << " kickParameter.kickStrength: " << (int)command.kickParameter.kickStrength;
        std::cout << " miscByte: " << (int)command.miscByte;
        std::cout << std::endl;

        out.clear();
        uint8_t tmp = 0;
        if (isShutdown) {
            // x_vector, y_vector, theta(th_vector), omega = 0
            // Index:0 DATAType4 0b100 + x_vector 0x0
            out.push_back(0b10000000);
            // Index:1 x_vector 0x0 
            out.push_back(0x0);
            // Index:2 x_vector 0x0 + y_vector 0x0
            out.push_back(0x0);
            // Index:3 y_vector 0x0
            out.push_back(0x0);
            // Index:4 y_vector + angleTypeSelect:OMEGA(1) + angle 0x10 
            out.push_back(0x10);
            // Index:5 angle 0x0
            out.push_back(0x0);
        } else {
            // Index:0 DATAType4 0b100 + CoordinateSystemType 2bits + X_VECTOR 3bits
            uint16_t uTargetX = std::abs(command.targetX);
            tmp = ((0b100) << 5) | ((command.robotCommandCoordinateSystemType & 0b11) << 3) | (command.targetX >= 0 ? 0 : 0b100) | ((uTargetX >> 11) & 0b11);
            out.push_back(tmp);
            // Index:1 x_vector
            tmp = (uTargetX >> 3) & 0xFF;
            out.push_back(tmp);
            // Index:2 x_vector + y_vector
            uint16_t uTargetY = std::abs(command.targetY);
            tmp = ((uTargetX & 0b111) << 5) | (command.targetY >= 0 ? 0 : 0b10000) | ((uTargetY >> 9) & 0b1111);
            out.push_back(tmp);
            // Index:3 y_vector
            tmp = (uTargetY >> 1) & 0xFF;
            out.push_back(tmp);
            // Index:4 y_vector + angle(7)
            uint16_t uTargetAngle = std::abs(command.targetAngle);
            tmp = ((uTargetY & 0b1) << 7) | (command.targetAngle >= 0 ? 0 : 0b1000000) | ((uTargetAngle >> 8) & 0b111111);
            out.push_back(tmp);
            // Index:5 angle
            tmp = uTargetAngle & 0xFF;
            out.push_back(tmp);
        }
        // Index:6 visionDataValid(1) + currentX(7)
        uint16_t uCurrentX = std::abs(command.currentX);
        tmp = ((command.visionDataValid & 0b1) << 7)  | (command.currentX >= 0 ? 0 : 0b1000000) | ((uCurrentX >> 7) & 0b111111);
        out.push_back(tmp);
        // Index:7 currentX(7) + currentY(1)
        tmp = ((uCurrentX & 0b1111111) << 1) | (command.currentY >= 0 ? 0 : 1);
        out.push_back(tmp);
        // Index:8 currentY(8)
        uint16_t uCurrentY = std::abs(command.currentY);
        tmp = (uCurrentY >> 5) & 0xFF;
        out.push_back(tmp);
        // Index:9 currentY(5) + currentAngle(3)
        uint16_t uCurrentAngle = std::abs(command.currentAngle);
        tmp = ((uCurrentY & 0b11111) << 3) | (command.currentAngle >= 0 ? 0 : 0b100) | ((uCurrentAngle >> 9) & 0b11);
        out.push_back(tmp);
        // Index:10 currentAngle(8)
        tmp = ((uCurrentAngle >> 1) & 0xFF);
        out.push_back(tmp);
        // Index:11 currentAngle(1) + KickParams
        tmp = ((uCurrentAngle & 0b1) << 7) | ((command.kickParameter.sensorUse & 0b111) << 4) | ((command.kickParameter.kickType & 0b1) << 3) | (command.kickParameter.kickStrength & 0b111);
        out.push_back(tmp);
        // Index:12 Misc Byte
        tmp = command.miscByte & 0xFF;
        out.push_back(tmp);
    }
    int AisaacBitmapHandler::parseFT4(std::vector<unsigned char> in, commandToRobot &out) {
        if (in.size() != 13) return -1;
        if ((in[0] >> 5) != 0b100) return -1;
        int16_t tmpShort = 0;
        uint16_t tmpUShort = 0;
        // robotCommandCoordinateSystemType
        out.robotCommandCoordinateSystemType = (uint8_t)((in[0] >> 3) & 0b11);
        // targetX: 14bit
        tmpShort = (int16_t)(in[0] & 0b11);
        tmpShort = (tmpShort << 8) | (int16_t)in[1];
        tmpShort = (tmpShort << 3) | (int16_t)((in[2] & 0b11100000) >> 5);
        out.targetX = tmpShort * ((in[0] & 0b100) == 0b100 ? -1 : 1);
        // targetY: 14bit
        tmpShort = (int16_t)(in[2] & 0b1111);
        tmpShort = (tmpShort << 8) | (int16_t)in[3];
        tmpShort = (tmpShort << 1) | (int16_t)((in[4] & 0b10000000) >> 1);
        out.targetY = tmpShort * ((in[2] & 0b10000) == 0b10000 ? -1 : 1);
        // targetAngle: 12bit
        tmpShort = (uint16_t)(in[4] & 0b1111111);
        tmpShort = (tmpUShort << 8) | (uint16_t)(in[5] & 0xFF);
        out.targetAngle = tmpShort;
        out.visionDataValid = (uint8_t)((in[6] >> 7) & 0b1);
        // currentX: 14bit
        tmpShort = (int16_t)((in[6]) & 0b111111);
        tmpShort = (tmpUShort << 7) | (((int16_t)in[7] & 0b11111110) >> 1);
        out.currentX = tmpShort * ((in[6] & 0b1000000) == 0b1000000 ? -1 : 1);
        // currentY: 14bit
        tmpShort = (int16_t)(in[8] & 0xFF);
        tmpShort = (tmpUShort << 5) | (int16_t)((in[9] & 0b11111000) >> 3);
        out.currentY = tmpShort * ((in[7] & 0b1) == 0b1 ? -1 : 1);
        // currentAngle: 12bit
        tmpShort = (int16_t)(in[9] & 0b11);
        tmpShort = (tmpUShort << 8) | (uint16_t)(in[10] & 0xFF);
        tmpShort = (tmpUShort << 1) | (uint16_t)((in[11] & 0b10000000) >> 7);
        out.currentAngle = tmpShort * ((in[9] & 0b100) == 0b100 ? -1 : 1);
        out.kickParameter.sensorUse = (uint8_t)((in[11] & 0b01110000) >> 4);
        out.kickParameter.kickType = (uint8_t)((in[11] & 0b1000) >> 3);
        out.kickParameter.kickStrength = (uint8_t)(in[11] & 0b111);
        out.miscByte = (uint8_t)in[12];
        return 0;
    }
    void AisaacBitmapHandler::sendCommand(int robotID, unsigned char sequenceNumber, std::vector<unsigned char> in) {
        if (robotID < 0 || robotID > numRobots) return;
        if (nodeList[robotID].phyTech == phyTechnology::undef) {
            return;
        } else if (nodeList[robotID].phyTech == phyTechnology::xbee) {
            if (xbeeActivated) {
                xbeeIF->setLongDestAddr(nodeList[robotID].addr);
                std::vector<unsigned char> comXBee;
                xbeeIF->constructXBeeFrame(in, sequenceNumber, comXBee);
                xbeeIF->send(comXBee);
            } else {
                // Send Via ROS node
                std::cout << "Send XBee node via ROS node(WiFi)" << std::endl;
            }
        } else if (nodeList[robotID].phyTech == phyTechnology::wifi) {
            if (wifiActivated) {
                wifiIF->send(nodeList[robotID].addr, robotID, sequenceNumber, in);
            } else {
                // Send Via ROS node
                std::cout << "Send WiFi node via ROS node(XBee)" << std::endl;
            }
        }
    }
    void AisaacBitmapHandler::messageHub(std::vector<unsigned char> in) {
        if (in.size() < 1) return;
        int messageType = (in[0] >> 5);
        if (messageType == 4) {
            messageReceiver(0, in);
        } else if (messageType == 0) {
            // int 
        }
    }
    void AisaacBitmapHandler::messageReceiver(int reason, std::vector<unsigned char> message) {
        std::cout << "Impliment messageReceiver(reason, message)" << std::endl;
        for (auto mes : message) std::cout << std::hex << std::setw(2) << std::setfill('0') << std::uppercase << (int)mes << " ";
        std::cout << std::endl;
    }
    // DEBUG
    void AisaacBitmapHandler::sayHello() {
        if (xbeeActivated) xbeeIF->sayHello();
    }
}
