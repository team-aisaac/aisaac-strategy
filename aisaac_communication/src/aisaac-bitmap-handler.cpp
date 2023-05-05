#include "aisaac_communication/aisaac-bitmap-handler.h"
#include "aisaac_communication/aisaac-xbee-base.h"
#include "aisaac_communication/aisaac-wifi-base.h"
#include <iostream>
#include <iomanip>
#include <vector>
#include <functional>
#include <cstdlib>

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
    void AisaacBitmapHandler::convertToString(commandToRobot command, std::vector<unsigned char> &out) {
        // std::cout << "command";
        // std::cout << " robotCommandCoordinateSystemType: " << (int)command.robotCommandCoordinateSystemType;
        // std::cout << " targetX: " << command.targetX;
        // std::cout << " targetY: " << command.targetY;
        // std::cout << " targetAngle: " << command.targetAngle;
        // std::cout << " visionDataValid: " << (int)command.visionDataValid;
        // std::cout << " currentX: " << command.currentX;
        // std::cout << " currentY: " << command.currentY;
        // std::cout << " currentAngle: " << (int)command.currentAngle;
        // std::cout << " kickParameter.sensorUse: " << (int)command.kickParameter.sensorUse;
        // std::cout << " kickParameter.kickType: " << (int)command.kickParameter.kickType;
        // std::cout << " kickParameter.kickStrength: " << (int)command.kickParameter.kickStrength;
        // std::cout << " miscByte: " << (int)command.miscByte;
        // std::cout << std::endl;

        // out.clear();

        // // Index:0 DATAType5 0b101 + 0b00000
        // out.push_back(0b10100000);

        // // Construct Protobuf message
        // aisaacpb::SpcCommand cmd_to_robot;
        // aisaacpb::RobotCommandCoordinateSystemType coord_type;
        // if (command.robotCommandCoordinateSystemType == 0) {
        //     coord_type = aisaacpb::RobotCommandCoordinateSystemType::Vector;
        // } else if (command.robotCommandCoordinateSystemType == 1) {
        //     coord_type = aisaacpb::RobotCommandCoordinateSystemType::Coordinate;
        // } else if (command.robotCommandCoordinateSystemType == 2) {
        //     coord_type = aisaacpb::RobotCommandCoordinateSystemType::Relax;
        // }
        // cmd_to_robot.set_robot_command_coordinate_system_type(coord_type);
        // cmd_to_robot.set_vision_data_valid(command.visionDataValid);
        // aisaacpb::Position current_pos;
        // current_pos.set_x(command.currentX);
        // current_pos.set_y(command.currentY);
        // current_pos.set_theta(command.currentAngle);
        // cmd_to_robot.set_allocated_current_pos(&current_pos);
        // aisaacpb::Velocity move_vec;
        // move_vec.set_vx(command.currentX);
        // move_vec.set_vy(command.currentY);
        // move_vec.set_omega(command.currentAngle);
        // cmd_to_robot.set_allocated_move_vec(&move_vec);
        // aisaacpb::Position target_pos;
        // target_pos.set_x(command.targetX);
        // target_pos.set_y(command.targetY);
        // target_pos.set_theta(command.targetAngle);
        // // https://developers.google.com/protocol-buffers/docs/cpptutorial#writing-a-message
        // // ToDo
        // aisaacpb::Obstacle *obstacle = cmd_to_robot.add_obstacles();
        // obstacle->set_x(0);
        // obstacle->set_y(0);
        // obstacle->set_vx(0);
        // obstacle->set_vy(0);
        // aisaacpb::Kick kick_cmd;
        // kick_cmd.set_sensor_type(aisaacpb::Kick::KickType(command.kickParameter.sensorUse));
        // kick_cmd.set_kick_method(aisaacpb::Kick::KickMethod(command.kickParameter.kickType));
        // kick_cmd.set_kick_strength((int32_t)command.kickParameter.kickStrength);
        // aisaacpb::Position ball_waypoint;
        // ball_waypoint.set_x(0);
        // ball_waypoint.set_y(0);
        // kick_cmd.set_allocated_ball_waypoint(&ball_waypoint);
        // aisaacpb::Position ball_pos;
        // ball_pos.set_x(0);
        // ball_pos.set_y(0);
        // kick_cmd.set_allocated_ball_pos(&ball_pos);
        // aisaacpb::Velocity ball_vel;
        // ball_vel.set_vx(0);
        // ball_vel.set_vy(0);
        // kick_cmd.set_allocated_ball_vel(&ball_vel);
        // cmd_to_robot.set_allocated_kick(&kick_cmd);
        // cmd_to_robot.set_prohibited_zone_ignore(false);
        
        // std::string serialized_str;
        // // Encode
        // cmd_to_robot.SerializeToString(&serialized_str);

        // for (unsigned char c : serialized_str) {
        //     out.push_back(c);
        // }
    }
    void AisaacBitmapHandler::convertFromCommandRealToProtobufEncodedString(const consai_msgs::robot_commands_realConstPtr& msg, std::vector<unsigned char> &out) {
        out.clear();

        // Index:0 DATAType5, 0 0b101 + 0b00000
        out.push_back(0b10100000);

        // Construct Protobuf message
        aisaacpb::SpcCommand command_to_robot;
        // aisaacpb::RobotCommandCoordinateSystemType coord_type;
        // if (command.robotCommandCoordinateSystemType == 0) {
        //     coord_type = aisaacpb::RobotCommandCoordinateSystemType::Vector;
        // } else if (command.robotCommandCoordinateSystemType == 1) {
        //     coord_type = aisaacpb::RobotCommandCoordinateSystemType::Coordinate;
        // } else if (command.robotCommandCoordinateSystemType == 2) {
        //     coord_type = aisaacpb::RobotCommandCoordinateSystemType::Relax;
        // }
        // command_to_robot.set_robot_command_type(coord_type);

        // goal_pose
        aisaacpb::Position goal_pose;
        goal_pose.set_x((int32_t)msg->goal_pose.x);
        goal_pose.set_y((int32_t)msg->goal_pose.y);
        goal_pose.set_theta((int32_t)msg->goal_pose.theta);
        command_to_robot.set_allocated_middle_goal_pose(&goal_pose);

        // prohibited_zone_ignore
        command_to_robot.set_prohibited_zone_ignore(msg->prohidited_zone_ignore);

        // middle_target_flag
        command_to_robot.set_middle_target_flag(msg->middle_target_flag);

        // middle_goal_pose
        aisaacpb::Position middle_goal_pose;
        middle_goal_pose.set_x((int32_t)msg->middle_goal_pose.x);
        middle_goal_pose.set_y((int32_t)msg->middle_goal_pose.y);
        middle_goal_pose.set_theta((int32_t)msg->middle_goal_pose.theta);
        command_to_robot.set_allocated_middle_goal_pose(&middle_goal_pose);

        // Dribble
        aisaacpb::Dribble dribble;
        // -dribble_power
        dribble.set_dribble_power((double)msg->dribble_power);
        // -dribble_state
        dribble.set_dribble_state(msg->dribble_state);
        // -dribbler_active
        dribble.set_dribbler_active(msg->dribbler_active);
        // -dribble_goal
        aisaacpb::Position dribble_goal;
        dribble_goal.set_x((int32_t)msg->dribble_goal.x);
        dribble_goal.set_y((int32_t)msg->dribble_goal.y);
        dribble_goal.set_theta((int32_t)msg->dribble_goal.theta);
        dribble.set_allocated_dribble_goal(&dribble_goal);
        // -dribble_complete_distance
        dribble.set_dribble_complete_distance(msg->dribble_complete_distance);
        command_to_robot.set_allocated_dribble(&dribble);

        // Kick
        aisaacpb::Kick kick;
        // -kick_power
        kick.set_kick_power(msg->kick_power);
        // -ball_kick_state
        kick.set_ball_kick_state(msg->ball_kick_state);
        // -ball_kick
        kick.set_ball_kick(msg->ball_kick);
        // -ball_kick_active
        kick.set_ball_kick_active(msg->ball_kick_active);
        // -free_kick_flag
        kick.set_free_kick_flag(msg->free_kick_flag);
        // -ball_goal
        aisaacpb::Position ball_goal;
        ball_goal.set_x((int32_t)msg->ball_goal.x);
        ball_goal.set_y((int32_t)msg->ball_goal.y);
        // msg->ball_goal.vel_x;
        // msg->ball_goal.vel_y;
        kick.set_allocated_ball_goal(&ball_goal);
        // -ball_target_allowable_error
        kick.set_ball_target_allowable_error(msg->ball_target_allowable_error);
        command_to_robot.set_allocated_kick(&kick);
        
        std::string serialized_str;
        // Encode
        command_to_robot.SerializeToString(&serialized_str);

        for (unsigned char c : serialized_str) {
            out.push_back(c);
        }
    }
    void AisaacBitmapHandler::convertFromCommandRealToProtobufEncodedStringVision(const consai_msgs::robot_commands_realConstPtr& msg, std::vector<unsigned char> &out) {
        out.clear();

        // Index:0 DATAType5, 1 0b101 + 0b00001
        out.push_back(0b10100000);

        aisaacpb::VisionData vision_data;

        // current_pose, own_machine_position
        aisaacpb::Position own_machine_position;
        own_machine_position.set_x((int32_t)msg->current_pose.x);
        own_machine_position.set_y((int32_t)msg->current_pose.y);
        own_machine_position.set_theta((int32_t)msg->current_pose.theta);
        vision_data.set_allocated_own_machine_position(&own_machine_position);
        // ball_position
        aisaacpb::Position ball_position;
        ball_position.set_x((int32_t)msg->ball_position.x);
        ball_position.set_y((int32_t)msg->ball_position.y);
        vision_data.set_allocated_ball_position(&ball_position);
        // obstacles
        auto obstacles = msg->obstacles;
        // for (auto rx_obstacle in msg) {
        //     aisaacpb::Obstacle* obstacle = vision_data.add_obstacles();
        //     obstacle->set_x(rx_obstacle.pose.x);
        //     obstacle->set_y(rx_obstacle.pose.y);
        //     obstacle->set_theta(rx_obstacle.pose.theta);
        //     obstacle->set_vx(rx_obstacle.vel.x);
        //     obstacle->set_vy(rx_obstacle.vel.y);
        // }
        
        std::string serialized_str;
        // Encode
        vision_data.SerializeToString(&serialized_str);

        for (unsigned char c : serialized_str) {
            out.push_back(c);
        }
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
