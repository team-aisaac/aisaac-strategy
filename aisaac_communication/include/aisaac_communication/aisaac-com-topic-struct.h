#pragma once

namespace aisaac {
    struct kickParamStruct {
        unsigned char sensorUse;
        bool kickType;
        unsigned char kickStrength;
    };

    struct commandToRobot {
        unsigned short int x_vector;
        unsigned short int y_vector;
        unsigned short int theta;
        unsigned short int omega;
        unsigned short int calibrationData;
        kickParamStruct kickParameter;
        unsigned short int worldCoordinateAngle;
    };
}