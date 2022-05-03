#pragma once

namespace aisaac {
    struct kickParamStruct {
        uint8_t sensorUse : 3;
        uint8_t kickType : 1;
        uint8_t kickStrength : 3;
    };

    struct commandToRobot {
        uint8_t robotCommandCoordinateSystemType : 2;
        int16_t x_vector : 15;
        int16_t y_vector : 15;
        uint8_t angleTypeSelect : 1;
        uint16_t angle : 12;
        uint8_t calibrationValid : 1;
        int16_t calibrationXPosition : 14;
        int16_t calibrationYPosition : 14;
        uint16_t calibrationAngle : 12;
        kickParamStruct kickParameter;
        uint8_t miscByte;
    };
}