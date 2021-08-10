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
    void AisaacBitmapHandler::generateFT4(commandToRobot command, std::vector<unsigned char> &out) {
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
            // Index:0 DATAType4 0b100 + X_VECTOR 5bits
            uint16_t uxvector = std::abs(command.x_vector);
            tmp = ((0b100) << 5) | (command.x_vector >= 0 ? 0 : 0b10000) | ((uxvector >> 11)&0b1111);
            out.push_back(tmp);
            // Index:1 x_vector 0x0
            tmp = (uxvector >> 3) & 0xFF;
            out.push_back(tmp);
            // Index:2 x_vector + y_vector
            uint16_t uyvector = std::abs(command.y_vector);
            tmp = ((uxvector & 0b111) << 5) | (command.y_vector >= 0 ? 0 : 0b10000) | ((uyvector >> 11) & 0b1111);
            out.push_back(tmp);
            // Index:3 y_vector
            tmp = (uyvector >> 3) & 0xFF;
            out.push_back(tmp);
            // Index:4 y_vector + angleTypeSelect(1) + angle(4)
            tmp = ((uyvector & 0b111) << 5) | ((command.angleTypeSelect << 4) & 0b10000) | ((command.angle >> 8) & 0b1111);
            out.push_back(tmp);
            // Index:5 angle
            tmp = command.angle & 0xFF;
            out.push_back(tmp);
        }
        // Index:6 calibrationValid(1) + calibrationXPosition(7)
        tmp = (command.calibrationValid << 7) | ((command.calibrationXPosition >> 7) & 0b1111111);
        out.push_back(tmp);
        // Index:7 calibrationXPosition(7) + calibrationYPosition(1)
        tmp = ((command.calibrationXPosition & 0b11111110) << 1) | ((command.calibrationYPosition >> 13) & 0b1);
        out.push_back(tmp);
        // Index:8 calibrationYPosition(8)
        tmp = (command.calibrationYPosition >> 5) & 0xFF;
        out.push_back(tmp);
        // Index:9 calibrationYPosition(5) + calibrationAngle(3)
        tmp = ((command.calibrationYPosition & 0b11111) << 3) | ((command.calibrationAngle >> 9) & 0b111);
        out.push_back(tmp);
        // Index:10 calibrationAngle(8)
        tmp = ((command.calibrationAngle >> 1) & 0xFF);
        out.push_back(tmp);
        // Index:11 calibrationAngle(1) + KickParams
        tmp = ((command.calibrationAngle & 0b1) << 7) | ((command.kickParameter.sensorUse & 0b111) << 4) | ((command.kickParameter.kickType & 0b1) << 3) | (command.kickParameter.kickStrength & 0b111);
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
        tmpShort = (short)(in[0] & 0xF);
        tmpShort = (tmpShort << 8) | (short)in[1];
        tmpShort = (tmpShort << 3) | (short)((in[2] & 0b11100000) >> 5);
        out.x_vector = tmpShort * ((in[0] & 0b10000) == 0b10000 ? -1.0 : 1.0);
        tmpShort = (short)(in[2] & 0xF);
        tmpShort = (tmpShort << 8) | (unsigned short)in[3];
        tmpShort = (tmpShort << 3) | (unsigned short)((in[4] & 0b11100000) >> 5);
        out.y_vector = tmpShort * ((in[2] & 0b10000) == 0b10000 ? -1.0 : 1.0);
        out.angleTypeSelect = (unsigned char)((in[4] >> 4) & 0b1);
        tmpUShort = (unsigned short)(in[4] & 0xF);
        tmpUShort = (tmpUShort << 8) | (unsigned short)(in[5] & 0xFF);
        out.angle = tmpUShort;
        out.calibrationValid = (unsigned char)((in[6] >> 7) & 0b1);
        tmpUShort = (unsigned short)((in[6] >> 1) & 0b1111111);
        tmpUShort = (tmpUShort << 7) | (((unsigned short)in[7] & 0b11111110) >> 1);
        out.calibrationXPosition = tmpUShort;
        tmpUShort = (unsigned short)(in[7] & 0b1);
        tmpUShort = (tmpUShort << 8) | (unsigned short)(in[8] & 0xFF);
        tmpUShort = (tmpUShort << 5) | (unsigned short)((in[9] & 0b11111000) >> 3);
        out.calibrationYPosition = tmpUShort;
        tmpUShort = (unsigned short)(in[9] & 0b111);
        tmpUShort = (tmpUShort << 8) | (unsigned short)(in[10] & 0xFF);
        tmpUShort = (tmpUShort << 1) | (unsigned short)((in[11] & 0b10000000) >> 7);
        out.calibrationAngle = tmpUShort;
        out.kickParameter.sensorUse = (unsigned char)((in[11] & 0b01110000) >> 4);
        out.kickParameter.kickType = (unsigned char)((in[11] & 0b1000) >> 3);
        out.kickParameter.kickStrength = (unsigned char)(in[11] & 0b111);
        out.miscByte = (unsigned char)in[12];
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
