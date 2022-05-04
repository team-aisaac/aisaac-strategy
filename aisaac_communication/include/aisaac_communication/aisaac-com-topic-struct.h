#pragma once

namespace aisaac {
    struct kickParamStruct {
        uint8_t sensorUse : 3;
        uint8_t kickType : 1;
        uint8_t kickStrength : 3;
    };

    struct commandToRobot {
        uint8_t robotCommandCoordinateSystemType : 2;
        int16_t targetX : 14;
        int16_t targetY : 14;
        int16_t targetAngle : 15;
        uint8_t visionDataValid : 1;
        int16_t currentX : 14;
        int16_t currentY : 14;
        int16_t currentAngle : 12;
        kickParamStruct kickParameter;
        uint8_t miscByte;
    };
}