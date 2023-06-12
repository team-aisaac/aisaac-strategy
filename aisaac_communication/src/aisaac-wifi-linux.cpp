#include <vector>
#include "aisaac_communication/aisaac-wifi-linux.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <cstring>
#include <iostream>

namespace aisaac {
    AisaacWifiLinux::~AisaacWifiLinux() {
        if (udpSock != -1) close(udpSock);
    }
    int AisaacWifiLinux::send(std::vector<unsigned char> ipaddr, int robotID, unsigned char sequenceNumber, std::vector<unsigned char> in) {
        // Set destination address
        std::string addrStr = std::to_string(ipaddr[0]) + "." + std::to_string(ipaddr[1]) + "." + std::to_string(ipaddr[2]) + "." + std::to_string(ipaddr[3]);
        addr.sin_addr.s_addr = inet_addr(addrStr.c_str());
        
        sendto(udpSock, in.data(), in.size(), 0, (struct sockaddr *)&addr, sizeof(addr));
        return 0;
    }
    void AisaacWifiLinux::wifiInit() {
        std::cout << "Init wifi Debug" << std::endl;
        udpSock = socket(AF_INET, SOCK_DGRAM, 0);
        if (udpSock == -1) std::cerr << "Failed: open udpSock" << std::endl;
        memset(&addr, 0, sizeof(addr));
        addr.sin_family = AF_INET;
        addr.sin_port = htons(11312);
    }
}
