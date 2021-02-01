#pragma once
#include <vector>
#include "aisaac-wifi-base.h"

namespace aisaac {
    class AisaacWifiLinux : public AisaacWifiBase {
    public:
        AisaacWifiLinux() : AisaacWifiBase() {}
        ~AisaacWifiLinux() {}
        int send(std::vector<unsigned char> ipaddr, std::vector<unsigned char> in) override;
    private:
    };
}