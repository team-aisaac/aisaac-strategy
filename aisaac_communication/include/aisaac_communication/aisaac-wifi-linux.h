#pragma once
#include <vector>
#include <netinet/in.h>
#include "aisaac-wifi-base.h"

namespace aisaac {
    class AisaacWifiLinux : public AisaacWifiBase {
    public:
        AisaacWifiLinux() : AisaacWifiBase() {
            wifiInit();
        }
        ~AisaacWifiLinux();
        int send(std::vector<unsigned char> ipaddr, int robotID, unsigned char sequenceNumber, std::vector<unsigned char> in) override;
        void wifiInit();
    private:
        int udpSock;
        struct sockaddr_in addr;
    };
}