#include <functional>
#include "aisaac_communication/aisaac-wifi-base.h"

namespace aisaac {
    AisaacWifiBase::AisaacWifiBase(/* args */)
    {
    }

    AisaacWifiBase::~AisaacWifiBase()
    {
    }

    void AisaacWifiBase::setCallbackAisaacBitmapHandler(std::function<void(ucvector)> func) {
        callbackABH = func;
    }
}