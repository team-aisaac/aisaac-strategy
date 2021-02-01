#include <vector>
#include <string>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdlib.h>
#include "aisaac_communication/aisaac-xbee-linux.h"

namespace aisaac {
    int AisaacXBeeLinux::send(std::vector<unsigned char> in) {
        // for (auto datum : in) write(serialDesc, datum, 1);
        write(serialDesc, (char *)&in[0], in.size());
        return 0;
    }
    void AisaacXBeeLinux::receive() {
        // Call in ros::ok()
        unsigned char in;
        int ret = read(serialDesc,&in, 1);
        if (ret < 0) return;
        if (in == 0x7E) {   // Start delimiter
            receiveBuffer.clear();
            receiveBuffer.push_back(in);
            escapeState = false;
            checkSum = 0;
            frameLength = 0;
            receivedByteLength = 1;
        } else if (in == 0x7D){ // Escape
            escapeState = true;
        } else {
            unsigned char datum;
            if (escapeState) {
                escapeState = false;
                datum = in ^ 0x20;
            } else {
                datum = in;
            }
            receiveBuffer.push_back(datum);

            if (receivedByteLength == (3 + frameLength)) {
                // Verify checksum and call decodeXBeeFrame()
                if ((checkSum + datum) == 0xFF) decodeXBeeFrame(receiveBuffer);
            } else if (receivedByteLength == 1) {  // MSB
                frameLength = (int)datum;
            } else if (receivedByteLength == 2) {   // LSB
                frameLength == (frameLength << 8) + (int)datum;
            } else {
                checkSum = (checkSum + datum) & 0xFF;
            }
            receivedByteLength++;
        }
    }
    void AisaacXBeeLinux::openPort(std::string serialPortName) {
        serialDesc = open(serialPortName.c_str(), O_RDWR);
        if (serialDesc < 0) {
            perror("Failed to open serial port.");
            exit(1);
        }

        // ioctl(serialDesc, TCGETS, &oldTio);
        tcgetattr(serialDesc, &oldTio);
        newTio = oldTio;
        newTio.c_cflag = B57600|CREAD|CS8|CLOCAL; // See termions(3)
        newTio.c_cc[VMIN] = 0;
        newTio.c_cc[VTIME] = 5; // timeout 500ms
        // ioctl(serialDesc, TCSETS, &newTio);
        tcsetattr(serialDesc, TCSANOW, &newTio);
    }
    AisaacXBeeLinux::~AisaacXBeeLinux() {
        ioctl(serialDesc, TCSETS, &oldTio);
        close(serialDesc);
    }
}