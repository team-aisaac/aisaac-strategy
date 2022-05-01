#include "aisaac_communication/aisaac-xbee-base.h"
#include <iostream>
#include <vector>
#include <functional>
#include <iomanip>

namespace aisaac {
    int AisaacXBeeBase::setLongDestAddr(std::vector<unsigned char>&longAddr) {
        if (longAddr.size() != 8) return -1;
        for (int i = 0; i < 8; i++) longDestAddr[i] = longAddr[i];
        return 0;
    }
    int AisaacXBeeBase::setShortDestAddr(std::vector<unsigned char>&shortAddr) {
        if (shortAddr.size() != 2) return -1;
        shortDestAddr[0] = shortAddr[0];
        shortDestAddr[1] = shortAddr[1];
        return 0;
    }
    void AisaacXBeeBase::setBroadcastRadius(int broadcastRad) {
        broadcastRadius = broadcastRad;
    }
    void AisaacXBeeBase::setXBeeOptions(int disableRetries, int enableAPSEncryption, int useExtRxTimeout) {
        xbeeOptions = 0;
        if (disableRetries) xbeeOptions += 0x01;
        if (enableAPSEncryption) xbeeOptions += 0x20;
        if (useExtRxTimeout) xbeeOptions += 0x40;
    }
    void AisaacXBeeBase::escape(unsigned char in, unsigned char &checkSum, std::vector<unsigned char>& out) {
        checkSum = (checkSum + in) & 0xFF;
        if (in == 0x11) {
            out.push_back(0x7D);
            out.push_back(0x31);
        } else if (in == 0x13) {
            out.push_back(0x7D);
            out.push_back(0x33);
        } else if (in == 0x7D) {
            out.push_back(0x7D);
            out.push_back(0x5D);
        } else if (in == 0x7E) {
            out.push_back(0x7D);
            out.push_back(0x5E);
        } else {
            out.push_back(in);
        }
    }
    void AisaacXBeeBase::constructXBeeFrame(std::vector<unsigned char>&in, unsigned char sequenceNumber, std::vector<unsigned char>&out) {
        constructTransmitRequestFrame(in, sequenceNumber, out);
    }
    void AisaacXBeeBase::constructTransmitRequestFrame(std::vector<unsigned char>& in, unsigned char sequenceNumber, std::vector<unsigned char>& out) {
        out.clear();    // Clear output vector
        out.push_back(0x7E);    // Start delimiter
        unsigned char checkSum = 0;
        // Count length
        int frameLength = 14 + in.size(); // Frame Type~Options
        escape(frameLength >> 8, checkSum, out);    // Length MSB
        escape(frameLength & 0xFF, checkSum, out);  // Length LSB
        checkSum = 0;   // Reset check sum
        // API Frame
        escape(0x10, checkSum, out); // Frame type 0x10 Transmit Request
        escape(sequenceNumber, checkSum, out);   // Frame ID
        // 64bit destination address
        for (int i = 0; i < 8; i++) escape(longDestAddr[i], checkSum, out);
        // 16bit destination address
        escape(shortDestAddr[0], checkSum, out);
        escape(shortDestAddr[1], checkSum, out);
        escape(broadcastRadius, checkSum, out);     // Broadcast radius
        escape(xbeeOptions, checkSum, out);         // Options
        for (unsigned int data : in) escape(data, checkSum, out);   // RF Data
        // Decide checkSum
        checkSum = 0xFF - checkSum;
        // out.push_back(checkSum);
        unsigned char temp = 0;
        escape(checkSum, temp, out);
    }

    // int send(std::vector<unsigned char> in) {
    //     return 0;
    // }

    int AisaacXBeeBase::decodeXBeeFrame(std::vector<unsigned char>&in) {
        // decoce XBee Frame
        // frame type 0x90 -> call ABH
        // callbackABH(in);
        // frame type 0x8B -> comm. status
        // frame type 0x8A -> XBee Network status
        int frameLength = ((int)in[1] << 8) + (int)in[2];
        unsigned char frameType = in[3];
        if (frameType == 0x90) {
            callbackABH(std::vector<unsigned char>(in.begin()+15, in.begin()+2+frameLength));
        } else if (frameType == 0x8A) {
            std::cout << "[Modem status] Status:" << std::hex << std::setw(2) << std::setfill('0') << std::uppercase << (int)in[4] << std::endl;
        } else if (frameType == 0x8B) {
            std::cout << "[ACK Status] Retry: " << std::dec << (int)in[7] << "times" << std::endl;
            std::cout << "Tx Status: " << std::hex << std::setw(2) << std::setfill('0') << std::uppercase << (int)in[8] << std::endl;
            std::cout << "Discovery Status: " << std::hex << std::setw(2) << std::setfill('0') << std::uppercase << (int)in[9] << std::endl;
        }
        return 0;
    }
    void AisaacXBeeBase::setCallbackAisaacBitmapHandler(std::function<void(ucvector)> func) {
        callbackABH = func;
    }
    // Test functions
    void AisaacXBeeBase::sayHello() {
        std::cout << "Hello from XBee Base" << std::endl;
    }
    void AisaacXBeeBase::callbackTest(std::vector<unsigned char> in) {
        callbackABH(in);
    }
}