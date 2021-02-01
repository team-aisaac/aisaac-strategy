#pragma once
#include <vector>
#include <string>
#include <termios.h>
#include "aisaac-xbee-base.h"

namespace aisaac {
    class AisaacXBeeLinux : public AisaacXBeeBase {
    public:
        AisaacXBeeLinux(std::string serialPortName) : AisaacXBeeBase() {
            openPort(serialPortName);
        }
        ~AisaacXBeeLinux();
        void openPort(std::string serialPortName);
        int send(std::vector<unsigned char> in) override;
        void receive();
    private:
        int serialDesc;
        struct termios oldTio, newTio;
        std::vector<unsigned char> receiveBuffer;
        bool escapeState;
        unsigned char checkSum;
        int frameLength;
        int receivedByteLength;
    };
}