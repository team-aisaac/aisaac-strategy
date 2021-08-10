#pragma once
#include <vector>
#include <iostream>
#include <functional>

typedef std::vector<unsigned char> ucvector;

namespace aisaac {
    class AisaacXBeeBase {
    public:
        AisaacXBeeBase() : xbeeOptions(0x0) {}
        virtual int send(std::vector<unsigned char> in) {return 0;};
        int receiveHandler();
        int setLongDestAddr(std::vector<unsigned char>&);
        int setShortDestAddr(std::vector<unsigned char>&);
        void setBroadcastRadius(int);
        void setXBeeOptions(int, int, int);
        void constructXBeeFrame(std::vector<unsigned char>&, unsigned char, std::vector<unsigned char>&);
        int decodeXBeeFrame(std::vector<unsigned char>&);
        void sayHello();
        void callbackTest(std::vector<unsigned char>);
        void setCallbackAisaacBitmapHandler(std::function<void(ucvector)> func);
    protected:
        unsigned char longDestAddr[8];
        unsigned char shortDestAddr[2];
        unsigned char xbeeOptions;
        unsigned char broadcastRadius;
        std::function<void(ucvector)> callbackABH;
        void escape(unsigned char, unsigned char &, std::vector<unsigned char>&);
        void constructTransmitRequestFrame(std::vector<unsigned char>&, unsigned char, std::vector<unsigned char>&);
    };
}