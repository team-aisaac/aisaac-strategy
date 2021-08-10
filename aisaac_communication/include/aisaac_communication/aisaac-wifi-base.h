#pragma once
#include <vector>
#include <functional>

typedef std::vector<unsigned char> ucvector;

namespace aisaac{
    class AisaacWifiBase
    {
    public:
        AisaacWifiBase(/* args */);
        virtual ~AisaacWifiBase();
        virtual int send(std::vector<unsigned char> ipaddr, int robotID, unsigned char sequenceNumber, std::vector<unsigned char> in) = 0;
        void setCallbackAisaacBitmapHandler(std::function<void(ucvector)> func);
    protected:
        std::function<void(ucvector)> callbackABH;
    };
}